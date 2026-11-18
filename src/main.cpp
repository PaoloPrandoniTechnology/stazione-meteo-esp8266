#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <Updater.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <user_interface.h>

#ifndef UPGRADE_FLAG_IDLE
#  define UPGRADE_FLAG_IDLE     0x00
#endif
#ifndef UPGRADE_FLAG_START
#  define UPGRADE_FLAG_START    0x01
#endif
#ifndef UPGRADE_FLAG_FINISH
#  define UPGRADE_FLAG_FINISH   0x02
#endif

#ifdef __cplusplus
extern "C" {
#endif
    void system_upgrade_reboot(void);
    void system_upgrade_flag_set(uint8_t flag);
    uint8_t system_upgrade_flag_check(void);
#ifdef __cplusplus
}
#endif

// ============================================================================
// Pin I2C - Wemos D1 Mini (ESP8266)
// SDA = D2 / GPIO4
// SCL = D1 / GPIO5
// ============================================================================
#define I2C_SDA_PIN   D2
#define I2C_SCL_PIN   D1
// 50 kHz: a 100 kHz le letture BME280 sbagliano un bit (chip-id 0x40 invece di 0x60, 2 su 5); da 80 kHz in giù 25/25 corrette.
#define I2C_CLOCK_HZ  50000UL

// ============================================================================
// BME280 - Indirizzi I2C possibili
// ============================================================================
#define BME280_ADDR_0      0x76
#define BME280_ADDR_1      0x77
#define BME280_SAMPLE_MS   100     // Intervallo di campionamento T/P/U

// ============================================================================
// SHT31-D Indirizzo I2C fisso e mappa comandi (driver I2C raw senza wrapper)
//   Datasheet Sensirion SHT31-DIS · Accuracy ±0.3°C / ±2%RH
//   Decisione utente: indirizzo fisso 0x44, Single-Shot 1Hz High Repeatability,
//                    CRC8 check abilitato, Heater auto RH>98% on / RH<95% off
// ============================================================================
#define SHT31_ADDR                 0x44
#define SHT31_SAMPLE_MS            1000
#define SHT31_CMD_SS_HIGH_NOCSB    0x2400  // Single-Shot · High Repeatability · No Clock Stretch
#define SHT31_CMD_SOFT_RESET       0x30A2
#define SHT31_CMD_CLEAR_STATUS     0x3041
#define SHT31_CMD_READ_STATUS      0xF32D
#define SHT31_CMD_HEATER_ENABLE    0x306D
#define SHT31_CMD_HEATER_DISABLE   0x3066
#define SHT31_CONV_MAX_MS          16      // High Repeatability · max 15 ms datasheet
#define SHT31_HEATER_RH_ON         98.0f   // soglia accensione heater (anti-condensa)
#define SHT31_HEATER_RH_OFF        95.0f   // soglia spegnimento
#define SHT31_HEATER_MIN_ON_MS     30000   // tempo minimo acceso per evitare cycling
#define SHT31_HEATER_MIN_OFF_MS    60000   // tempo minimo spento tra un ciclo e l'altro
// Status register flags (bit mask 16-bit lettura 0xF32D):
#define SHT31_STATUS_ALERT_MASK    0x8000  // Alert pending (T o RH fuori allerta)
#define SHT31_STATUS_HEATER_MASK   0x2000  // Heater acceso
#define SHT31_STATUS_RH_ALERT      0x0800  // Humidity alert
#define SHT31_STATUS_T_ALERT       0x0400  // Temp alert
#define SHT31_STATUS_RESET_MASK    0x0010  // Reset detected (clear dopo write 0x3041)
#define SHT31_STATUS_CMD_MASK      0x0002  // Last command failed
#define SHT31_STATUS_CRC_MASK      0x0001  // Write data CRC failed

// ============================================================================
// Pin Hall A3144 - Wemos D1 Mini (ESP8266)
// OUT del sensore (con pull-up) -> D5 / GPIO14
// ============================================================================
#define HALL_PIN              D5

// ============================================================================
// Parametri anemometro Hall - CONFIGURABILI
// (sensore a effetto Hall A3144 unipolare, polo SUD triggerante)
// ============================================================================
#define HALL_MAGNETS_PER_REV   1        // Numero di magneti sull'albero rotante
#define HALL_MIN_PERIOD_US     100      // Anti-bounce: ignora fronti < 100µs (~120'000 RPM max)
#define ANEMO_RADIUS_M         0.060f   // Raggio tazze anemometro in metri (default 60mm)
#define ANEMO_CALIB_K          1.00f    // Fattore di calibrazione empirico
// -----------------------------------------------------------------------------
// [SAFETY FIX · Vento lento 3s/giro]
// HALL_WATCHDOG_MS vecchio = 500ms → troppo corto: a 3s/giro il segnale veniva
// dichiarato "morto" tra un trigger e l'altro. Ora: 5000ms (5s) = min ~0.125 m/s
// misurabile con sicurezza (~12 RPM). Dopo 5s SENZA impulsi dichiariamo
// quiete (vento=0), ma tra un trigger e l'altro HOLD del valore.
#define HALL_WATCHDOG_MS       5000
// -----------------------------------------------------------------------------
// Filtro EMA istantaneo: applicato ALL'ARRIVO di un NUOVO periodo
// (alla vecchia frequenza 20Hz questo alpha veniva ri-applicato troppo spesso
// e uccideva lo smoothing col decay). Ora: aggiornamento "event-driven" solo
// quando new_period_copy == true.
#define HALL_EMA_ALPHA         0.45f
// -----------------------------------------------------------------------------
// Decay MOLTO LENTO tra impulsi: 0.998 ogni 50ms → al secondo 0.96 → in 3s
// ancora ~89% del valore → non si vede lo "zero tra trigger" e la curva è
// costante/lineare tra tic e tic.
#define HALL_HOLD_DECAY        0.998f

// ============================================================================
// Pin Rain Gauge Bascula - Wemos D1 Mini (ESP8266)
// OUT Hall della bascula (pull-up) → D7 / GPIO13
// ============================================================================
#define RAIN_PIN                 D7

// ============================================================================
// Parametri Pluviometro Bascula - CONFIGURABILI
//   Imbuto: ø85mm; Bascula inverte a 9.24g; Hall genera impulso digitale
//   mm_per_tick = (9.24g / (π·(4.25cm)²)) · 10 = 1.628 mm
// ============================================================================
#define RAIN_FUNNEL_DIA_MM_DEF     85.0f   // diametro imbuto millimetri (default)
#define RAIN_TIP_MASS_G_DEF        9.24f   // massa inversione bascula grammi (default)
#define RAIN_CALIB_K_DEF           1.00f   // fattore calibrazione empirico
#define RAIN_DEAD_MS_DEF           150     // anti-bounce meccanico (ms)
#define RAIN_RATE_EMA_ALPHA        0.25f   // filtro EMA rain rate
#define RAIN_WB_INTERVAL_MS        3600000UL // write-back EEPROM ogni 1 ora
#define RAIN_WB_MM_THRESHOLD       25.0f     // oppure ogni 25 mm cumulativi
#define RAIN_PERSIST_EEPROM_OFFSET 2048      // offset RainPersist_t (separato da Config_t)
#define RAIN_PERSIST_MAGIC         0x5241494EUL // 'RAIN'

// ============================================================================
// AS5600 Indirizzo I2C fisso
// ============================================================================
#define AS5600_ADDR       0x36

// ============================================================================
// AS5600 Mappa registri
// ============================================================================
#define AS5600_REG_ZPOS      0x01
#define AS5600_REG_MPOS      0x03
#define AS5600_REG_MANG      0x05
#define AS5600_REG_CONF      0x07
#define AS5600_REG_RAWANGLE  0x0C
#define AS5600_REG_STATUS    0x0B
#define AS5600_REG_ANGLE     0x0E
#define AS5600_REG_AGC       0x1A
#define AS5600_REG_MAGNITUDE 0x1B

// ============================================================================
// AS5600 bit registro STATUS (0x0B)
// ============================================================================
#define AS5600_ST_MH         0x20   // bit5 = Magnet Too Strong
#define AS5600_ST_ML         0x10   // bit4 = Magnet Too Weak
#define AS5600_ST_MD         0x08   // bit3 = Magnet Detected

// ============================================================================
// Parametri AS5600 (BUSSOLA - I2C) - CONFIGURABILI
// ============================================================================
#define AS5600_CONF_PM       0b00   // Power Mode: 00=NOM, 01=LPM1, 10=LPM2, 11=LPM3
#define AS5600_CONF_HYST     0b10   // Hysteresis: 00=OFF, 01=1LSB, 10=2LSB, 11=3LSB
#define AS5600_CONF_OUTS     0b10   // Output Stage: 00=ANALOG_FULL, 01=ANALOG_REDUCED, 10=PWM
#define AS5600_CONF_PWMF     0b00   // PWM Freq: 00=115Hz, 01=230Hz, 10=460Hz, 11=920Hz
#define AS5600_CONF_SF       0b000  // Slow Filter: 000=16x ... 100=1x
#define AS5600_CONF_FTH      0b110  // Fast Filter: 000=SLOW_ONLY, 001..111 = soglie LSB (110=6LSB)
#define AS5600_CONF_WD       0b0    // Watchdog: 0=OFF (free-running), 1=ON idle 60s
#define AS5600_ZPOS          0x0000
#define AS5600_MPOS          0x0FFF
#define AS5600_MANG          0x0000

// ============================================================================
// Parametri qualita' AS5600 (Default per soglie Gating)
// ============================================================================
#define AS5600_AGC_MIN_DEF   16     // AGC < 16  → troppo forte (MH)
#define AS5600_AGC_MAX_DEF   240    // AGC > 240 → troppo debole (ML)
#define AS5600_MAG_MIN_DEF   1024   // Magnitude < 1024 → segnale troppo basso (~25% fondo scala)
#define AS5600_QUALITY_OK    2.0f   // Valido
#define AS5600_QUALITY_WARN  1.0f   // Warning (non perfetto ma accettabile)
#define AS5600_QUALITY_FAIL  0.0f   // Rifiutato

// ============================================================================
// [NUOVO] Heading Filter · 3 stadi configurabili
//   Stage 1: Spike Rejection (soglia delta gradi)
//   Stage 2: Circular Moving Average · Media Vettoriale N campioni
//   Stage 3: Output Decimation (periodo ms)
// Persistenza: HeadingFilterConfig_t in EEPROM offset 3072 (zona libera
// dopo RainPersist_t a 2048; evita collisioni con Config_t ~0-1024).
// ============================================================================
#define HDRFILT_EEPROM_OFFSET          3072
#define HDRFILT_MAGIC                  0x48444746UL  // 'HDGF'
#define HDRFILT_VERSION_CURR           3       // v3: aggiunto filter_enable (disable = bypass totale → raw AS5600)
#define HDRFILT_FILTER_EN_DEF          1       // default: filtro attivo (backward compat v1/v2 caricati → ENABLED)
#define HDRFILT_SPIKE_DEG_DEF_X100     1000    // 10.00 gradi (soglia aggressiva)
#define HDRFILT_WINDOW_DEF             80      // 80 campioni @20Hz = 4 sec
#define HDRFILT_OUTPUT_MS_DEF          2000    // 0.5 Hz (molto lento, stabile)
#define HDRFILT_EMA_ALPHA_DEF_X100     10      // α EMA = 10/100 = 0.10 (cost. tempo ~10s @20Hz)
#define HDRFILT_WINDOW_MAX             240     // max 240 campioni @20Hz = 12 sec (uint8 safe, buffer circolare 240 floats ~960B)
#define HDRFILT_WINDOW_MIN             1       // min 1 campione (1 = bypass totale media vettoriale: 1 elemento = identità)
#define HDRFILT_OUTPUT_MS_MIN          50      // min 50 ms → 20 Hz (stessa frequenza loop tick, nessuna decimazione)
#define HDRFILT_OUTPUT_MS_MAX          60000   // max 60 secondi (un valore ogni minuto)
#define HDRFILT_SPIKE_DEG_MIN_X100     0       // 0.00° = DISATTIVA RIFIUTO PICCHI (sempre vero, accetta TUTTI i campioni)
#define HDRFILT_SPIKE_DEG_MAX_X100     18000   // 180.00° = DISATTIVA RIFIUTO PICCHI (delta circolare max = 180° → sempre <= soglia, mai scartato) [18000 int16_t ok, no overflow]
#define HDRFILT_EMA_ALPHA_MIN_X100     0       // 0.00 = DISATTIVA EMA (pass-through, stage 2.5 saltato)
#define HDRFILT_EMA_ALPHA_MAX_X100     100     // 1.00 = zero smoothing EMA (out = in pieno, nessun ritardo)


// ============================================================================
// Sensor Filter Config - Parametri filtri vento + rain rate (EEPROM offset 3584)
//   Struttura separata NON tocca Config_t → CRC retrocompatibile 100%
//   Interi scalati (x1000 o x10000) evitano float in struct CRC-calcolata.
// ============================================================================
#define SFIL_EEPROM_OFFSET             3584
#define SFIL_MAGIC                     0x5346494CUL   // 'SFIL'
#define SFIL_VERSION_CURR              1
// Hall Wind EMA Alpha: x1000 (0.001 step) → 0.01..1.00 (10..1000)
#define SFIL_HALL_EMA_DEF_X1000        450     // 0.450 (default HALL_EMA_ALPHA)
#define SFIL_HALL_EMA_MIN_X1000        10      // 0.010 (max smoothing)
#define SFIL_HALL_EMA_MAX_X1000        1000    // 1.000 (nessuno smoothing)
// Hall Wind Hold Decay: x10000 (0.0001 step) → 0.9000..0.9999 (9000..9999)
#define SFIL_HALL_DECAY_DEF_X10000     9980    // 0.9980 (default HALL_HOLD_DECAY)
#define SFIL_HALL_DECAY_MIN_X10000     9000    // 0.9000 (decay veloce)
#define SFIL_HALL_DECAY_MAX_X10000     9999    // 0.9999 (decay quasi nullo)
// Rain Rate EMA Alpha: x1000 → 0.01..1.00
#define SFIL_RRATE_DEF_X1000           250     // 0.250 (default RAIN_RATE_EMA_ALPHA)
#define SFIL_RRATE_MIN_X1000           10      // 0.010
#define SFIL_RRATE_MAX_X1000           1000    // 1.000

// ============================================================================
// LTR390 Indirizzo I2C fisso e mappa registri
// ============================================================================
#define LTR390_ADDR             0x53
#define LTR390_REG_MAIN_CTRL    0x00
#define LTR390_REG_MEAS_RATE    0x04
#define LTR390_REG_GAIN         0x05
#define LTR390_REG_PART_ID      0x06
#define LTR390_REG_MAIN_STATUS  0x07
// Output ALS (singolo canale, già filtrato a spettro visibile ~occhio umano, 450-700nm)
#define LTR390_REG_ALS_DATA_LSB 0x0D
#define LTR390_REG_ALS_DATA_MSB 0x0E
#define LTR390_REG_ALS_DATA_XTB 0x0F
// Output UV (singolo canale, 280-430nm UVA+UVB)
#define LTR390_REG_UVS_DATA_LSB 0x10
#define LTR390_REG_UVS_DATA_MSB 0x11
#define LTR390_REG_UVS_DATA_XTB 0x12
// MAIN_CTRL comandi (Datasheet sez. 6.1 + AUTO-DETECT clone/chip vario):
//   bit3=UVS_Mode, bit2=RESERVED (sul clone/chip utente DEVE essere 1=ON per abilitare
//                                 il latch di MAIN_CTRL; con 0 il write non attecchisce;
//                                 AUTO-DETECT prova tutte le combinazioni).
//   bit1=ALS/UVS_Enable. ALS e UVS sono mutuamente esclusivi, switch modale.
#define LTR390_MODE_STANDBY     0x00
#define LTR390_SW_RESET         0x10  // bit4 SW Reset (write 1 self-cleared after ~5ms)
// Modalità attive = VARIABILI (sovrascrivibili dall'auto-detect, non più hardcoded)
// Default (pattern già verificato funzionante sul chip utente PRE-AUDIT):
static uint8_t g_ltr390_mode_als_active = 0x06;  // bit2=1 + bit1=1 + ALS (default)
static uint8_t g_ltr390_mode_uvs_active = 0x0E;  // bit3=1 + bit2=1 + bit1=1 (default)
// Timestamp boot LTR390 per auto-recovery
static unsigned long g_ltr390_boot_ts = 0;
// Contatore auto-recovery strong
static uint32_t      g_ltr390_recovery_count = 0;
// MAIN_STATUS flag:
#define LTR390_ST_ALS_DRDY      0x04  // bit2 = ALS data ready
#define LTR390_ST_UVS_DRDY      0x08  // bit3 = UVS data ready

// ============================================================================
// Parametri LTR390 (I2C 0x53) - CONFIGURABILI / AUTO-DETECT
//   Gain  : 0=1x 1=3x 2=6x 3=9x 4=18x  (3x default buono indoor+outdoor)
//   Res/It: 0=20bit/400ms 1=19bit/200ms 2=18bit/100ms 3=17bit/50ms
//           4=16bit/25ms  5=16bit/25ms  6=17bit/50ms  7=18bit/100ms
//           (2=100ms + 18bit coerente col BME280)
//   Rate  : misurazione ripetuta 0=25ms..6=2000ms (3=100ms default)
// ============================================================================
// Valori di default (prima di AUTO-DETECT GAIN/RES)
#define LTR390_GAIN_ID_DEFAULT   2       // 6x — Indoor normale
#define LTR390_RES_IT_ID_DEFAULT 1       // 200ms / ~19bit
#define LTR390_MEAS_RATE          0x00   // 100ms tra misure continuous (default datasheet)
#define LTR390_SAMPLE_MS          200    // Lettura FW ogni 200ms (allineata a integrazione HW)
#define LTR390_UVI_SENSITIVITY    2300.0f // datasheet @18x 100ms (noi normalizziamo)
// Variabili runtime: sovrascrivibili da auto-detect
static uint8_t g_ltr390_gain_id    = LTR390_GAIN_ID_DEFAULT;
static uint8_t g_ltr390_res_it_id  = LTR390_RES_IT_ID_DEFAULT;
// Alias mantenuti per retrocompatibilità con define usati nel codice
#define LTR390_GAIN_ID        g_ltr390_gain_id
#define LTR390_RES_IT_ID      g_ltr390_res_it_id
// SAFE-READ-LOOP: timestamp ultima lettura forzata ALS/UVS anche senza DRDY
static unsigned long g_ltr390_forced_read_als_ts = 0;
static unsigned long g_ltr390_forced_read_uvs_ts = 0;
#define LTR390_FORCED_READ_MS  500   // ogni 500ms leggiamo comunque il registro (DRDY buggato su alcuni clone)

// ============================================================================
// AS3935 Lightning Detector · RIMOSSO (2026-09-23)
//   - Problemi hardware breakout CJMCU-3935: indirizzi I²C 0x02/0x03 non raggiungibili
//   - Per ripristinare: riattivare il blocco defines/variabili/funzioni as3935_* e
//     chiamate as3935_init() / as3935_poll() in setup() e loop()
// ============================================================================
static const uint32_t  g_snap_lightning_total_count   = 0;
static const float     g_snap_lightning_distance_km   = NAN;
static const uint32_t  g_snap_lightning_energy_raw    = 0;
static const float     g_snap_lightning_last_age_min  = NAN;
static const uint8_t   g_snap_lightning_noise_floor   = 0;
static const uint8_t   g_snap_lightning_watchdog      = 0;
static const uint8_t   g_snap_lightning_spike_rej     = 0;
static const uint16_t  g_snap_lightning_tuning_cap_pf = 0;
static const uint8_t   g_snap_lightning_min_trigger   = 1;
static const uint8_t   g_snap_lightning_osc_status    = 0;
static const uint32_t  g_snap_lightning_last_epoch_ms = 0;

// ============================================================================
// Crash Logger / Supervisore / WiFi Robusto - Costanti Principali
// ============================================================================
#define RTC_MEM_HEARTBEAT_START_W  48
#define RTC_MEM_HEARTBEAT_END_W    63
#define RTC_MEM_CRASH_START_W      64
#define RTC_MEM_CRASH_END_W        90
#define RTC_MAGIC_1                0xCA55CADEUL
#define HEARTBEAT_PERIOD_MS        500

#define WIFI_STA_RETRY_COUNT          10
#define WIFI_STA_PER_TRY_TIMEOUT_MS   15000
#define WIFI_STA_INTER_RETRY_DELAY_MS 2000
#define WIFI_STA_HARD_RESET_OFF_MS    500
#define WIFI_FALLBACK_POLL_PERIOD_MS  30000

#define SUPV_WDT_HW_MS             8000
#define SUPV_CONN_STALL_MS         10000
#define SUPV_PUB_STALL_MS          10000
#define SUPV_L1_COOLDOWN_MS        3000
#define SUPV_L2_COOLDOWN_MS        5000
#define SUPV_L3_COOLDOWN_MS        8000
#define SUPV_L3_OFF_MS             1200
#define SUPV_L3_POST_MS            100
#define SUPV_L4_RESET_MS           120000

#define MQTT_KEEPALIVE_S           10
#define MQTT_SOCKET_TIMEOUT_S      5
#define MQTT_CLIENT_BUFFER_SIZE    2048
#define MQTT_ONLINE_REFRESH_MS     30000
#define TCP_KA_IDLE_S              5
#define TCP_KA_INTERVAL_S          3
#define TCP_KA_COUNT               3
#define MQTT_CONNECT_BACKOFF_MIN_MS  250
#define MQTT_CONNECT_BACKOFF_MAX_MS  2000

// ============================================================================
// EEPROM - Layout, Config_t, Magic/CRC/Migrazione
// ============================================================================
#define EEPROM_SIZE_BYTES       4096
#define CONFIG_MAGIC            0x55AA55AAUL
#define CONFIG_VERSION_CURRENT  5
#define CONFIG_VERSION_LEGACY   1
#define CONFIG_VERSION_PREV2    2
#define CONFIG_VERSION_PREV     3
#define CONFIG_VERSION_LAST     4

struct Config_t {
  uint32_t magic;
  uint8_t  version;
  uint8_t  as5600_agc_min;       // ex reserved1 (default 16)
  uint8_t  as5600_agc_max;       // ex reserved2 (default 240)
  uint8_t  as5600_mag_min_lo;    // ex reserved3  (LSB uint16_t magnitude min)

  char     device_name[40];
  char     sensor_as5600_name[40];
  char     sensor_as3935_name[40];
  char     sensor_bme280_name[40];
  char     sensor_hall_name[40];
  char     sensor_ltr390_name[40];
  char     sensor_sht31_name[40];

  char     wifi_ssid[64];
  char     wifi_password[64];
  char     ap_ssid[32];
  char     ap_password[32];

  char     mqtt_server[64];
  uint16_t mqtt_port;
  uint8_t  as5600_mag_min_hi;    // ex reserved_mqtt1 (MSB uint16_t magnitude min)
  uint8_t  reserved_mqtt2;
  char     mqtt_user[48];
  char     mqtt_password[64];
  char     mqtt_device_id[64];

  uint16_t as5600_zpos;
  uint16_t as5600_mpos;
  uint16_t as5600_mang;
  int16_t  as5600_offset_deg_100x;

  float    bme280_altitude_ref_m;
  float    bme280_sealevel_hpa;

  uint8_t  ltr390_gain_id;
  uint8_t  ltr390_res_it_id;
  uint8_t  ltr390_reserved1;
  uint8_t  ltr390_reserved2;
  float    ltr390_window_factor;

  uint8_t  hall_magnets_per_rev;
  uint8_t  hall_reserved1;
  uint8_t  hall_reserved2;
  uint8_t  hall_reserved3;
  float    hall_anemo_radius_m;
  float    hall_anemo_calib_k;

  char     sensor_rain_name[40];
  float    rain_funnel_dia_mm;
  float    rain_tip_mass_g;
  float    rain_calib_k;
  uint16_t rain_dead_ms;
  uint8_t  rain_reserved1;
  uint8_t  rain_reserved2;

  uint32_t crc32;
};

static_assert(sizeof(Config_t) <= EEPROM_SIZE_BYTES, "Config_t exceeds EEPROM size");

// ============================================================================
// HeadingFilterConfig_t · Parametri filtro heading 3 stadi (EEPROM offset 3072)
//   Non aggiungiamo a Config_t per mantenere CRC retrocompatibile.
//   Campi scalati x100 o interi per evitare float in strutture CRC-calcolate.
//   Define HDRFILT_* e offset HDRFILT_EEPROM_OFFSET sono nella sezione AS5600 sopra.
// ============================================================================
typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  ema_alpha_x100;          // v2: α EMA 2° stage = α/100 (es. 10 → 0.10)
    uint16_t reserved_2;
    int16_t  spike_threshold_deg_x100;   // es. 2000 = 20.00°
    uint8_t  movavg_window;              // N campioni 1..240
    uint8_t  filter_enable;              // v3: 1 = filtri 4 stadi ATTIVI; 0 = BYPASS TOTALE → raw AS5600 solo invert+offset
    uint16_t output_period_ms;           // 50..60000 ms
    uint32_t reserved_4;
    uint32_t crc32;
} HeadingFilterConfig_t;

static_assert(sizeof(HeadingFilterConfig_t) <= 128,
    "HeadingFilterConfig_t eccede riserva offset 3072-3200 in EEPROM");

// ============================================================================
// SensorFilterConfig_t · Filtri Anemometro + RainRate (EEPROM offset 3584)
//   Analogamente a HeadingFilterConfig_t: struttura separata, non tocca Config_t.
// ============================================================================
typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  reserved_1;
    uint16_t hall_ema_alpha_x1000;    // α vento /1000 (0.010..1.000)
    uint16_t hall_hold_decay_x10000;  // decay vento /10000 (0.9000..0.9999)
    uint16_t rain_rate_ema_x1000;     // α rain rate /1000 (0.010..1.000)
    uint16_t reserved_2;
    uint32_t reserved_3;
    uint32_t crc32;
} SensorFilterConfig_t;

static_assert(sizeof(SensorFilterConfig_t) <= 128,
    "SensorFilterConfig_t eccede riserva offset 3584-3712 in EEPROM");

// ---- Valori DEFAULT ----
#define DEFAULT_DEVICE_NAME           "Stazione Meteo Esterna"
#define DEFAULT_SENSOR_AS5600         "Bussola AS5600"
#define DEFAULT_SENSOR_AS3935         "Rilevatore Fulmini AS3935"
#define DEFAULT_SENSOR_BME280         "Modulo BME280"
#define DEFAULT_SENSOR_HALL           "Anemometro Hall"
#define DEFAULT_SENSOR_LTR390         "Luce/UV LTR390"
#define DEFAULT_SENSOR_SHT31          "Sensore SHT31 (Temp/Umid)"
#define DEFAULT_SENSOR_RAIN           "Pluviometro Bascula Hall"
#define DEFAULT_WIFI_SSID             "IL_TUO_SSID"
#define DEFAULT_WIFI_PASSWORD         "LA_TUA_PASSWORD_WIFI"
#define DEFAULT_AP_SSID               "MeteoESP-Config"
#define DEFAULT_AP_PASSWORD           "cambiami123"
#define DEFAULT_MQTT_SERVER           "192.168.1.10"
#define DEFAULT_MQTT_PORT             1883
#define DEFAULT_MQTT_USER             "utente_mqtt"
#define DEFAULT_MQTT_PASSWORD         "password_mqtt"
#define DEFAULT_MQTT_DEVICE_ID        "stazione_meteo_esterna"
#define DEFAULT_BME_SEALEVEL_HPA      1013.25f
#define DEFAULT_BME_ALTITUDE_M        0.0f

static Config_t g_config;
static bool     g_mqtt_config_changed = false;
static bool     g_mqtt_was_connected = false;
static bool     g_wifi_config_changed = false;
// [BUG FIX heading offset] Flag: prossima iterazione scheduler MQTT → forza publish
// di TUTTI i valori (force=true). Necessario per applicare immediatamente offset
// as5600, parametri anemometro, ecc. anche quando la variazione è < soglia on-change.
static bool     g_mqtt_force_repub_all = false;
static uint32_t g_diag_lastMqttWifiWord = 0;
static uint32_t g_diag_lastSensorWord = 0;
static uint8_t  g_diag_bootCount = 0;
static uint32_t g_baselineSP = 0;
static uint32_t g_lowestSP = 0xFFFFFFFF;

typedef enum {
  RST_POWERON = 0,
  RST_EXTERNAL_PIN = 1,
  RST_HW_WATCHDOG = 2,
  RST_SW_WATCHDOG = 3,
  RST_EXCEPTION_PANIC = 4,
  RST_SOFTWARE_RESTART = 5,
  RST_DEEPSLEEP_WAKE = 6,
  RST_BROWNOUT_DETECT = 7,
  RST_UNKNOWN = 99
} RstReasonCode;

typedef struct {
  RstReasonCode code; const char* label;
} RstReasonInfo;
static const RstReasonInfo PROGMEM RST_TABLE[] = {
  { RST_POWERON,           "Power-On" },
  { RST_EXTERNAL_PIN,      "External PIN" },
  { RST_HW_WATCHDOG,       "HW Watchdog" },
  { RST_SW_WATCHDOG,       "SW Watchdog" },
  { RST_EXCEPTION_PANIC,   "Exception Panic" },
  { RST_SOFTWARE_RESTART,  "Software Restart" },
  { RST_DEEPSLEEP_WAKE,    "Deep-Sleep Wake" },
  { RST_BROWNOUT_DETECT,   "Brown-Out" }
};

typedef enum {
  SUPV_IDLE = 0,
  SUPV_L1_TRIGGERED,
  SUPV_L1_WAIT_COOLDOWN,
  SUPV_L2_TRIGGERED,
  SUPV_L2S_WAIT_COOLDOWN,
  SUPV_L3_TRIGGERED,
  SUPV_L4_DUE
} supv_state_t;

static bool        g_supvTripped = false;
static uint32_t    g_supvTripMs = 0;
static supv_state_t g_supvState = SUPV_IDLE;
static uint32_t    g_supvLastActionMs = 0;
static uint32_t    g_supvL1Count = 0;
static uint32_t    g_supvL2Count = 0;
static uint32_t    g_supvL3Count = 0;
static uint32_t    g_supvResetCount = 0;
static uint32_t    g_mqtt_lastGoodConnMs = 0;
static uint32_t    g_mqtt_lastGoodPubMs = 0;
static uint8_t     g_mqtt_pub_consec_fail = 0;
static uint8_t     g_mqtt_consec_failures = 0;
static bool        g_net_fallback_mode = false;
static uint32_t    g_net_fallbackLastPollMs = 0;

// ============================================================================
// Parametri di sistema
// ============================================================================
#define SERIAL_BAUDRATE     115200
#define FW_BUILD_STR        __DATE__ " " __TIME__
#define SAMPLE_PERIOD_MS    50          // 20 Hz loop master + WS (era 10ms = 100Hz)
#define HTTP_PORT            80
#define WS_PORT              81
#define P0_SEALEVEL_HPA       1013.25f

// --- MQTT Scheduler Multi-Gruppo ---
// GRUPPO A: Alta dinamica (2 Hz = 500 ms) — Vento / Direzione
#define MQTT_GRP_A_PERIOD_MS   250   // vento: 4Hz = 250ms (come decay UI)
#define MQTT_WIND_FORCE_EVERY_MS  250   // safety-net: pubblica wind TUTTE le volte (nessun
                                          // delay max) perché il decay lento del WS si vede
                                          // solo se pubblichiamo con la stessa frequenza UI.
// GRUPPO B: Ambientali lente + soglie on-change (0.2 Hz = 5 s) — T/RH/P/Lux/UV/Indici
#define MQTT_GRP_B_PERIOD_MS   5000
// GRUPPO C: Diagnostica / Stringhe / Config (0.1 Hz = 10 s)
#define MQTT_GRP_C_PERIOD_MS   10000
// Availability heartbeat: MQTT_ONLINE_REFRESH_MS invariato (30 s)

// --- On-Change Dead-band (soglie semantiche |Δ| < soglia = skip publish) ---
#define THRESH_TEMP_C         0.20f   // °C
#define THRESH_RH_PCT         1.0f    // %RH
#define THRESH_PRESSURE_HPA   0.20f   // hPa
#define THRESH_WIND_MS        0.001f  // m/s  (risoluzione sensore Hall 1µs periodo ≈ 0.001 m/s)
#define THRESH_WIND_KMH       (THRESH_WIND_MS * 3.6f)
#define THRESH_WIND_KN        (THRESH_WIND_MS * 1.9438445f)
#define THRESH_HEADING_DEG    2.0f    // gradi
#define THRESH_LUX_PCT        5.0f    // % variazione lux
#define THRESH_UVI            0.20f   // UVI
#define THRESH_ALT_M          0.5f    // m altitudine barometrica
#define THRESH_VPD_KPA        0.05f   // kPa
#define THRESH_RAIN_MM        0.01f   // mm (1 tick ≈ 1.6mm → passa sempre)

// --- WebSocket Binary Layout Invariante · 168 byte (42 float32 little-endian) ---
#define WS_PAYLOAD_BYTES        168
static_assert(WS_PAYLOAD_BYTES == 42 * sizeof(float), "WS payload MUST be 42×float32 = 168 bytes");
static_assert(WS_PAYLOAD_BYTES == 168, "Retrocompat WS v5: dimensione 168 byte INALTERABILE per client legacy");

// --- ISR Rain dead-time cached (configurabile via EEPROM, min 150ms safety) ---
static volatile uint32_t g_rain_isr_dead_ms = RAIN_DEAD_MS_DEF;

// --- Cache indici SHT calcolati a 1 Hz (non 20 Hz nel loop master) ---
// Popolate nel gate SHT31_SAMPLE_MS; usate nel sentinel e MQTT publish senza ricalcolo
static float s_sht_cache_t    = NAN;
static float s_sht_cache_rh   = NAN;
static float s_sht_cache_dp   = NAN;
static float s_sht_cache_fp   = NAN;
static float s_sht_cache_wb   = NAN;
static float s_sht_cache_vpd  = NAN;
static float s_sht_cache_ah   = NAN;
static float s_sht_cache_hi   = NAN;
static float s_sht_cache_hx   = NAN;
static bool  s_sht_cache_ok   = false;

// ============================================================================
// Front-end inline: Tabbed UI · Dashboard Live + Configurazione Completa
// ============================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="it">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Stazione Meteo · ESP8266</title>
<style>
  :root{
    --bg:#0e0f11; --panel:#15171b; --panel-h:#1c1f25; --border:#2a2e36;
    --ink:#e6e8ec; --ink-dim:#a7adb6; --ink-more:#7b828c;
    --ok:#66bb6a; --warn:#ffb300; --err:#ef5350; --info:#42a5f5; --primary:#3b82f6;
    --mono:"Courier New", ui-monospace, monospace;
    --tab-bg:#101318; --tab-bg-act:#1c1f25; --tab-bot:#3b82f6;
  }
  *{box-sizing:border-box}
  html,body{margin:0;padding:0;min-height:100%;background:var(--bg);color:var(--ink);
    font-family:ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial}
  .wrap{padding:18px;max-width:1150px;margin:0 auto}
  header{display:flex;align-items:center;justify-content:space-between;gap:14px;margin-bottom:12px;flex-wrap:wrap}
  header h1{margin:0;font-size:17px;letter-spacing:.3px;font-weight:600;color:#e9ecef;display:inline-flex;align-items:baseline;gap:10px;flex-wrap:wrap}
  header .fw-tag{display:inline-block;background:linear-gradient(180deg,rgba(59,130,246,.22),rgba(59,130,246,.10));
    border:1px solid rgba(59,130,246,.45);color:#9ec2ff;font-family:var(--mono);font-size:11px;
    padding:3px 9px;border-radius:999px;letter-spacing:.3px;vertical-align:middle;white-space:nowrap}
  header .sub{color:var(--ink-more);font-size:12px;margin-top:2px}
  .status{display:flex;align-items:center;gap:8px;color:var(--ink-dim);font-size:12px;font-family:var(--mono)}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--err);box-shadow:0 0 8px var(--err)}
  .dot.ok{background:var(--ok);box-shadow:0 0 8px var(--ok)}
  .tabs{display:flex;border-bottom:2px solid var(--border);gap:2px;margin-bottom:14px}
  .tab{padding:10px 18px 9px;font-size:13px;font-weight:600;color:var(--ink-dim);
    cursor:pointer;background:var(--tab-bg);border:1px solid var(--border);border-bottom:none;
    border-radius:8px 8px 0 0;user-select:none;transition:color .15s, background .15s}
  .tab:hover{color:var(--ink)}
  .tab.active{color:#fff;background:var(--tab-bg-act);border:1px solid var(--border);position:relative}
  .tab.active::after{content:"";position:absolute;left:8px;right:8px;bottom:-2px;height:3px;background:var(--tab-bot);border-radius:3px}
  .tab .cnt{font-family:var(--mono);font-size:10px;padding:1px 6px;margin-left:6px;border-radius:10px;background:rgba(59,130,246,.15);color:#8db7ff;border:1px solid rgba(59,130,246,.3)}
  .tab.active .cnt{background:rgba(102,187,106,.15);color:#a5e7a7;border-color:rgba(102,187,106,.3)}
  .pane{display:none}
  .pane.active{display:block}

  /* ===== Dashboard ===== */
  table.dati{width:100%;border-collapse:collapse;background:var(--panel);
    border:1px solid var(--border);border-radius:10px;overflow:hidden;box-shadow:0 8px 28px rgba(0,0,0,.35)}
  table.dati thead th{background:linear-gradient(180deg,var(--panel-h),#14161a);
    color:#dde2e8;text-align:left;font-weight:600;font-size:11px;letter-spacing:1px;
    text-transform:uppercase;padding:10px 14px;border-bottom:1px solid var(--border);position:sticky;top:0;z-index:2}
  table.dati thead th.right{text-align:right}
  table.dati tbody td{padding:7px 14px;border-bottom:1px solid #1e2128;font-size:13px;vertical-align:middle}
  table.dati tbody tr:last-child td{border-bottom:none}
  table.dati tbody tr.cat td{background:linear-gradient(180deg,#242830,#1b1e25);
    color:#f4f6f9;font-weight:700;font-size:11px;letter-spacing:1.2px;text-transform:uppercase;
    padding:8px 14px;border-bottom:1px solid var(--border);border-top:1px solid #0a0b0d}
  table.dati .k{color:var(--ink-dim)}
  table.dati .v{color:#f6f8fa;font-family:var(--mono);font-size:13px;text-align:right;font-variant-numeric:tabular-nums}
  table.dati .u{color:var(--ink-more);font-family:var(--mono);font-size:11px;padding-left:6px}
  table.dati .note{color:var(--ink-more);font-size:11px}
  table.dati .v.bad{color:var(--err)}
  table.dati .v.warn{color:var(--warn)}
  table.dati .v.ok{color:var(--ok)}
  .footer{margin-top:12px;color:var(--ink-more);font-size:11px;display:flex;justify-content:space-between;flex-wrap:wrap;gap:6px;font-family:var(--mono)}

  /* ===== Config ===== */
  .grid-2{display:grid;grid-template-columns:1fr 1fr;gap:14px}
  @media(max-width:820px){.grid-2{grid-template-columns:1fr}}
  .card{background:var(--panel);border:1px solid var(--border);border-radius:10px;padding:16px;box-shadow:0 4px 16px rgba(0,0,0,.25)}
  .card h2{margin:0 0 12px 0;font-size:13px;letter-spacing:.8px;text-transform:uppercase;color:#dde2e8;
    font-weight:700;padding-bottom:8px;border-bottom:1px solid var(--border)}
  .card h3{margin:14px 0 8px 0;font-size:11px;letter-spacing:1px;text-transform:uppercase;color:var(--ink-dim);font-weight:700}
  .f{display:flex;flex-direction:column;gap:4px;margin-bottom:11px}
  .f label{font-size:12px;color:var(--ink-dim);font-weight:600}
  .f .hint{font-size:10px;color:var(--ink-more);font-family:var(--mono)}
  .f input,.f select{background:#0b0d10;border:1px solid #343a44;border-radius:6px;padding:8px 10px;color:var(--ink);
    font-size:13px;font-family:var(--mono);outline:none;transition:border-color .15s}
  .f input:focus,.f select:focus{border-color:var(--info);box-shadow:0 0 0 3px rgba(66,165,245,.12)}
  .f input.invalid{border-color:var(--err);box-shadow:0 0 0 3px rgba(239,83,80,.12)}
  .row-2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  .row-3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}
  .actions{display:flex;gap:10px;flex-wrap:wrap;margin-top:16px;align-items:center}
  button{border:1px solid #343a44;background:#1a1e24;color:var(--ink);padding:9px 16px;border-radius:7px;
    font-size:13px;font-weight:600;cursor:pointer;transition:all .15s;display:inline-flex;align-items:center;gap:6px}
  button:hover{background:#222831;border-color:#4a5260}
  button.primary{background:#1e3a8a;border-color:#2a4caf;color:#dbeafe}
  button.primary:hover{background:#1f44b0}
  button.warn{background:#7c2d12;border-color:#a0401b;color:#fed7aa}
  button.warn:hover{background:#99361a}
  button.danger{background:#7f1d1d;border-color:#a52a2a;color:#fecaca}
  button.danger:hover{background:#991b1b}
  button:disabled{opacity:.5;cursor:not-allowed}
  button.b-primary{background:rgba(59,130,246,.18);border-color:rgba(59,130,246,.55);color:#c7ddff}
  button.b-primary:hover{background:rgba(59,130,246,.30);border-color:rgba(59,130,246,.75)}
  .flash{padding:10px 14px;border-radius:7px;font-size:13px;font-weight:600;margin-bottom:12px;display:none;border:1px solid transparent}
  .flash.show{display:block}
  .flash.ok{background:rgba(102,187,106,.12);border-color:rgba(102,187,106,.3);color:var(--ok)}
  .flash.err{background:rgba(239,83,80,.12);border-color:rgba(239,83,80,.3);color:var(--err)}
  .flash.warn{background:rgba(255,179,0,.12);border-color:rgba(255,179,0,.3);color:var(--warn)}
  .topic-preview{background:#0b0d10;border:1px solid #343a44;border-radius:6px;padding:10px 12px;
    font-family:var(--mono);font-size:11px;color:var(--ink-dim);word-break:break-all;line-height:1.7}
  .topic-preview b{color:#90caf9;font-weight:500}
  .note{margin-top:10px;padding:12px;border-left:4px solid #26a69a;
    background:#122322;border:1px solid rgba(38,166,154,.4);
    border-left:4px solid #26a69a;border-radius:6px;color:#e6e8ec;font-size:12px;line-height:1.7;font-family:var(--mono)}
  .note b{color:#ffffff !important;font-weight:700}
  .note i{color:#5ed4c1 !important;font-style:normal !important;font-weight:600}
  button.b-primary{background:rgba(59,130,246,.22);border:1px solid rgba(59,130,246,.65);color:#cfe0ff;font-weight:600}
  button.b-primary:hover{background:rgba(59,130,246,.40);border-color:rgba(59,130,246,.85);color:#eaf1ff}
  @media(max-width:640px){
    table.dati thead th,table.dati tbody td{padding:6px 8px;font-size:12px}
    table.dati .v{font-size:12px}
    header h1{font-size:15px}
    .row-2,.row-3{grid-template-columns:1fr}
  }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div>
      <h1 id="h-title">Stazione Meteo ESP8266 <span class="fw-tag" id="h-fwbuild" style="display:none" title="Data/ora compilazione firmware">FW: --</span></h1>
      <div class="sub" id="h-sub">Sensori Ambientali · Dashboard Live</div>
    </div>
    <div class="status">
      <span id="dot" class="dot"></span>
      <span id="stat">connessione...</span>
    </div>
  </header>

  <div class="tabs">
    <div class="tab active" data-tab="dash">📊 Dashboard <span class="cnt" id="cnt-dash">LIVE</span></div>
    <div class="tab" data-tab="cfg">⚙ Configurazione <span class="cnt" id="cnt-cfg">EEPROM</span></div>
    <div class="tab" data-tab="ota">💾 Firmware OTA <span class="cnt" id="cnt-ota">OTA</span></div>
  </div>

  <!-- =================== DASHBOARD =================== -->
  <div class="pane active" id="p-dash">
    <table class="dati" id="tab">
      <thead>
        <tr>
          <th style="width:38%">Parametro</th>
          <th class="right" style="width:34%">Valore</th>
          <th style="width:28%">Note / Unità</th>
        </tr>
      </thead>
      <tbody id="rows"></tbody>
    </table>
    <div class="footer">
      <div id="f-host"></div>
      <div id="f-build" style="color:var(--primary)"></div>
      <div id="f-ts"></div>
    </div>
  </div>

  <!-- =================== CONFIG =================== -->
  <div class="pane" id="p-cfg">
    <div id="flash" class="flash"></div>

    <div class="grid-2">
      <!-- ===== SEZIONE 1: IDENTIFICATIVI ===== -->
      <div class="card">
        <h2>1 · Identificativi Dispositivo & Sensori</h2>
        <div class="f"><label>Nome dispositivo (device_name)</label>
          <input id="cf-device" type="text" maxlength="39" placeholder="Sensori Temp/Umid">
          <div class="hint">Comparirà in Home Assistant · max 39 char</div></div>
        <h3>Nomi Sensori (per MQTT Discovery)</h3>
        <div class="f"><label>Bussola AS5600</label><input id="cf-as5600" type="text" maxlength="39"></div>

        <div class="f"><label>SHT31-D (Temp / Umid.)</label><input id="cf-sht31" type="text" maxlength="39"></div>
        <div class="f"><label>Modulo BME280 (T/P/U)</label><input id="cf-bme280" type="text" maxlength="39"></div>
        <div class="f"><label>Anemometro Hall</label><input id="cf-hall" type="text" maxlength="39"></div>
        <div class="f"><label>Luce/UV LTR390</label><input id="cf-ltr390" type="text" maxlength="39"></div>
        <div class="f"><label>Pioggia Bascula Hall</label><input id="cf-rainname" type="text" maxlength="39"></div>
      </div>

      <!-- ===== SEZIONE 2: CONNETTIVITA' ===== -->
      <div class="card">
        <h2>2 · Connettività WiFi + Access Point</h2>
        <div class="f"><label>SSID Rete Principale (STA)</label>
          <input id="cf-wssid" type="text" maxlength="63" placeholder="Rete WiFi 2.4GHz"></div>
        <div class="f"><label>Password WiFi STA</label>
          <input id="cf-wpass" type="password" maxlength="63" placeholder="password rete"></div>
        <h3>Access Point di Emergenza (fallback)</h3>
        <div class="row-2">
          <div class="f"><label>SSID AP</label><input id="cf-aps" type="text" maxlength="31"></div>
          <div class="f"><label>Password AP (min 8 char)</label><input id="cf-app" type="password" maxlength="63"></div>
        </div>
      </div>

      <!-- ===== SEZIONE 3: MQTT ===== -->
      <div class="card" style="grid-column:1 / -1">
        <h2>3 · Broker MQTT (Home Assistant Discovery)</h2>
        <div class="row-3">
          <div class="f"><label>Server / IP Broker</label>
            <input id="cf-mip" type="text" maxlength="63" placeholder="192.168.1.10"></div>
          <div class="f"><label>Porta TCP</label>
            <input id="cf-mport" type="number" min="1" max="65535" value="1883"></div>
          <div class="f"><label>Device ID (topic)</label>
            <input id="cf-mid" type="text" maxlength="63" pattern="[a-zA-Z0-9_-]+" placeholder="sensori_temperatura"></div>
        </div>
        <div class="row-2">
          <div class="f"><label>Username MQTT</label><input id="cf-muser" type="text" maxlength="47"></div>
          <div class="f"><label>Password MQTT</label><input id="cf-mpass" type="password" maxlength="63"></div>
        </div>
        <div class="topic-preview" id="mqtt-preview">
          <div><b>Base topic:</b> homeassistant/sensor/<span id="tp-base">device_id</span>/</div>
          <div><b>Availability:</b> homeassistant/sensor/<span id="tp-base2">device_id</span>/availability</div>
          <div><b>Esempio Temperatura (SHT31 primaria):</b> homeassistant/sensor/<span id="tp-base3">device_id</span>/sht31_temp_c/state</div>
        </div>
      </div>

      <!-- ===== SEZIONE 4: BUSSOLA AS5600 ===== -->
      <div class="card">
        <h2>4 · Bussola AS5600 (I2C 0x36)</h2>
        <div class="row-2">
          <div class="f"><label>ZPOS (Start Position 0–4095)</label>
            <input id="cf-zpos" type="number" min="0" max="4095" value="0"></div>
          <div class="f"><label>MPOS (Stop Position 0–4095)</label>
            <input id="cf-mpos" type="number" min="0" max="4095" value="4095"></div>
        </div>
        <div class="row-2">
          <div class="f"><label>MANG (Max Angle 0–4095, 0=disabilitato)</label>
            <input id="cf-mang" type="number" min="0" max="4095" value="0"></div>
          <div class="f"><label>Offset calibrazione bussola (gradi)</label>
            <input id="cf-offset" type="number" step="0.01" min="-360" max="360" value="0.00" placeholder="es. -20.00 = sottrai 20°"></div>
        </div>
        <h3>4b · Filtro Heading (anti-picchi + smoothing)</h3>
        <div class="row-2">
          <div class="f">
            <label style="display:flex;align-items:center;gap:10px;"><input id="cf-hdgf-enable" type="checkbox" checked> <b style="font-size:1.05em;">Abilita catena filtri heading 4 stadi</b></label>
            <div class="hint">⚠ Disattiva = bypass completo (output raw sensore)</div>
          </div>
          <div class="f">
            <label>Preset Combinati (clicca per caricare, poi Salva)</label>
            <div id="hdgf-preset-row" style="display:flex;flex-wrap:wrap;gap:6px;margin-top:6px;">
              <button type="button" data-hdgf-kind="raw"      onclick="applyHdgfPreset('raw');">🔴 SENZA FILTRI (raw AS5600)</button>
              <button type="button" data-hdgf-kind="light"    onclick="applyHdgfPreset('light');">🟢 Leggero</button>
              <button type="button" data-hdgf-kind="balanced" onclick="applyHdgfPreset('balanced');">🔵 Bilanciato (default)</button>
              <button type="button" data-hdgf-kind="strong"   onclick="applyHdgfPreset('strong');">🟣 Forte</button>
            </div>
            <div class="hint">RAW: off · LEGGERO: spike 30 N=10 α=0.70 out 500ms · BIL: spike 10 N=80 α=0.10 out 2s · FORTE: spike 5 N=200 α=0.03 out 5s.</div>
            <div id="hdgf-preset-feedback" style="margin-top:6px;min-height:1.2em;color:#107e3e;font-weight:600;">&nbsp;</div>
          </div>
        </div>
        <div class="row-3">
          <div class="f"><label>Soglia rifiuto picchi (°) · 0/180 = off</label>
            <input id="cf-hdgf-spike" type="number" step="0.01" min="0" max="180" value="10.00"></div>
          <div class="f"><label>Finestra media vettoriale · 1 = off</label>
            <input id="cf-hdgf-win" type="number" step="1" min="1" max="240" value="80"></div>
          <div class="f"><label>Periodo output (ms) · min 50 = 20Hz</label>
            <input id="cf-hdgf-out" type="number" step="50" min="50" max="60000" value="2000"></div>
        </div>
        <div class="row-3">
          <div class="f"><label>α EMA Vettoriale · 0 = off / 1.00 = no smoothing</label>
            <input id="cf-hdgf-ema" type="number" step="0.01" min="0.00" max="1.00" value="0.10"></div>
          <div class="f"><label>Tau (secondi, stimato)</label>
            <input id="cf-hdgf-tau" type="text" disabled value="≈ 0.5 s · α 0.10"></div>
          <div class="f"><label>Preset α EMA</label>
            <div style="display:flex;flex-wrap:wrap;gap:6px;margin-top:6px;">
              <button type="button" onclick="$('cf-hdgf-ema').value=0.00;calcTauHdgf();">OFF 0.00</button>
              <button type="button" onclick="$('cf-hdgf-ema').value=0.03;calcTauHdgf();">MAX 0.03</button>
              <button type="button" onclick="$('cf-hdgf-ema').value=0.10;calcTauHdgf();" class="b-primary">MEDIO 0.10</button>
              <button type="button" onclick="$('cf-hdgf-ema').value=0.70;calcTauHdgf();">Piano 0.70</button>
              <button type="button" onclick="$('cf-hdgf-ema').value=1.00;calcTauHdgf();">PASS 1.00</button>
            </div></div>
        </div>
        <div class="note"><b>Guida filtri</b>: Preset <b>🔴 SENZA FILTRI</b> → Salva (raw heading raw 20Hz). Riattiva master con spike=0, N=1, α=0.00. Aggiungi <i>uno alla volta</i>: spike → finestra N → α EMA → output ms.</div>
      </div>

      <!-- ===== SEZIONE 5: BME280 ===== -->
      <div class="card">
        <h2>5 · BME280 Pressione (QNH + Altitudine)</h2>
        <div class="row-2">
          <div class="f"><label>Altitudine riferimento (metri)</label>
            <input id="cf-alt" type="number" step="0.1" min="-500" max="9000" value="0"></div>
          <div class="f"><label>Pressione QNH livello mare (hPa)</label>
            <input id="cf-sea" type="number" step="0.01" min="870" max="1085" value="1013.25"></div>
        </div>
      </div>

      <!-- ===== SEZIONE 6: LTR390 ALS/UV ===== -->
      <div class="card">
        <h2>6 · Sensore LTR390 (I2C 0x53 · Luce + UV)</h2>
        <div class="row-3">
          <div class="f"><label>Gain</label>
            <select id="cf-gain">
              <option value="0">1x (outdoor luminoso)</option>
              <option value="1">3x</option>
              <option value="2" selected>6x (mix indoor/outdoor)</option>
              <option value="3">9x</option>
              <option value="4">18x (indoor scuro)</option>
            </select></div>
          <div class="f"><label>Risoluzione / Integrazione</label>
            <select id="cf-res">
              <option value="0">20-bit / 400ms</option>
              <option value="1" selected>19-bit / 200ms</option>
              <option value="2">18-bit / 100ms</option>
              <option value="3">17-bit / 50ms</option>
              <option value="4">16-bit / 25ms</option>
              <option value="5">16-bit / 25ms</option>
              <option value="6">17-bit / 50ms</option>
              <option value="7">18-bit / 100ms</option>
            </select></div>
          <div class="f"><label>Window Factor (vetro ≥1.0)</label>
            <input id="cf-wf" type="number" step="0.01" min="1" max="10" value="1"></div>
        </div>
      </div>

      <!-- ===== SEZIONE 7: ANEMOMETRO HALL ===== -->
      <div class="card">
        <h2>7 · Anemometro Hall A3144</h2>
        <div class="row-3">
          <div class="f"><label>Magneti per giro</label>
            <input id="cf-mag" type="number" min="1" max="32" value="4"></div>
          <div class="f"><label>Raggio tazze (metri)</label>
            <input id="cf-r" type="number" step="0.001" min="0.01" max="1" value="0.100"></div>
          <div class="f"><label>Fattore calibrazione K</label>
            <input id="cf-k" type="number" step="0.01" min="0.1" max="5" value="1"></div>
        </div>
        <h3 style="margin-top:16px;margin-bottom:10px;font-size:15px;color:var(--primary)">Filtri velocità vento · 2 stadi</h3>
        <div class="row-3">
          <div class="f"><label>α EMA Velocità (0.01–1.00)</label>
            <input id="cf-hall-ema" type="number" step="0.01" min="0.01" max="1.00" value="0.45"></div>
          <div class="f"><label>Hold Decay vento debole (0.9000–0.9999)</label>
            <input id="cf-hall-decay" type="number" step="0.0001" min="0.9000" max="0.9999" value="0.9980"></div>
          <div class="f"><label>Preset Filtri Vento</label>
            <div style="display:flex;flex-direction:column;gap:8px;margin-top:6px;">
              <div style="display:flex;gap:8px;">
                <button type="button" onclick="$('cf-hall-ema').value=0.10;$('cf-hall-decay').value=0.9995;">Fluido 0.10</button>
                <button type="button" onclick="$('cf-hall-ema').value=0.45;$('cf-hall-decay').value=0.9980;" class="b-primary">Bilanciato 0.45</button>
                <button type="button" onclick="$('cf-hall-ema').value=0.80;$('cf-hall-decay').value=0.9900;">Reattivo 0.80</button>
              </div></div></div>
        </div>
      </div>

      <!-- ===== SEZIONE 8: PLUVIOMETRO BASCULA HALL ===== -->
      <div class="card">
        <h2>8 · Pluviometro Bascula Hall</h2>
        <div class="row-3">
          <div class="f"><label>Diametro imbuto (mm)</label>
            <input id="cf-rdiam" type="number" step="0.1" min="20" max="500" value="85"></div>
          <div class="f"><label>Massa inversione (g)</label>
            <input id="cf-rmass" type="number" step="0.01" min="1" max="100" value="9.24"></div>
          <div class="f"><label>Fattore calibrazione K</label>
            <input id="cf-rk" type="number" step="0.01" min="0.1" max="5" value="1"></div>
          <div class="f"><label>Dead-time anti-bounce (ms)</label>
            <input id="cf-rdeb" type="number" step="1" min="150" max="2000" value="150">
            <div class="hint">Valore minimo safety 150 ms. Per valori inferiori la ISR forza 150 ms automaticamente.</div></div>
          <div class="f"><label>mm per tick (calcolato)</label>
            <input id="cf-rmmtick" type="number" step="0.0001" readonly></div>
          <div class="f"><label>Tick count (live)</label>
            <input id="cf-rtick" type="number" readonly></div>
          <div class="f"><label>mm Oggi (reset auto a mezzanotte)</label>
            <input id="cf-revmm" type="number" step="0.01" readonly></div>
        </div>
        <h3 style="margin-top:16px;margin-bottom:10px;font-size:15px;color:var(--primary)">Filtro Intensità Pioggia (mm/h) · EMA</h3>
        <div class="row-3">
          <div class="f"><label>α EMA Intensità (0.01–1.00)</label>
            <input id="cf-rainrate-ema" type="number" step="0.01" min="0.01" max="1.00" value="0.25"></div>
          <div class="f"><label>Preset Filtro Pioggia</label>
            <div style="display:flex;flex-direction:column;gap:8px;margin-top:6px;">
              <div style="display:flex;gap:8px;">
                <button type="button" onclick="$('cf-rainrate-ema').value=0.10;">Fluido 0.10</button>
                <button type="button" onclick="$('cf-rainrate-ema').value=0.25;" class="b-primary">Bilanciato 0.25</button>
                <button type="button" onclick="$('cf-rainrate-ema').value=0.70;">Reattivo 0.70</button>
              </div></div></div>
          <div class="f"></div>
        </div>
        <div style="margin-top:12px">
          <button class="warn" id="btn-reset-rain" type="button">🔄 Azzera Evento Pioggia</button>
          <span class="hint" style="margin-left:10px;color:var(--ink-dim);font-size:12px">
            Il conteggio TOTALE non viene azzerato (persiste in EEPROM per statistiche storiche).
          </span>
        </div>
      </div>
    </div>

    <!-- ===== AZIONI ===== -->
    <div class="card" style="margin-top:14px">
      <h2>Azioni</h2>
      <div style="color:var(--ink-dim);font-size:12px;margin-bottom:10px">
        Il salvataggio scrive in EEPROM (4096 byte · ~100k cicli). I parametri MQTT modificati provocano una riconnessione immediata.
        <b style="color:var(--warn)">Le credenziali WiFi modificate richiedono il RIAVVIO del dispositivo.</b>
      </div>
      <div class="actions">
        <button class="primary" id="btn-save">💾 Salva in EEPROM</button>
        <button id="btn-load">↺ Ricarica corrente</button>
        <button class="warn" id="btn-default">⚠ Ripristina Default</button>
        <button class="danger" id="btn-reboot">🔄 Riavvia dispositivo</button>
        <span style="margin-left:auto;font-family:var(--mono);font-size:11px;color:var(--primary)" id="cfg-build"></span>
        <span style="font-family:var(--mono);font-size:11px;color:var(--ink-more)" id="cfg-status">pronto</span>
      </div>
    </div>
  </div>

  <!-- =================== FIRMWARE OTA =================== -->
  <div class="pane" id="p-ota">
    <div id="ota-flash" class="flash"></div>
    <div class="grid-2">
      <div class="card">
        <h2>1 · Stato Firmware Corrente</h2>
        <div class="f"><label>🧠 Versione attualmente in esecuzione</label>
          <div class="topic-preview"><b id="ota-cur-build" style="font-size:13px;color:#90caf9">— rilevamento in corso…</b>
          <div id="ota-cur-hint" style="margin-top:6px">Se compare una data/ora qui, il firmware attuale già supporta il tracciamento build. Se rimane vuoto, hai un FW precedente o la pagina è cachata (Ctrl+Shift+R).</div></div>
        </div>
        <div class="f"><label>🌐 Browser HTTP (canale corrente)</label>
          <div class="hint">Upload file .bin prodotto da PlatformIO (cartella .pio/build/d1_mini/firmware.bin) · drag-drop supportato</div></div>
        <div class="f"><label>🛠 PlatformIO / Arduino IDE (ArduinoOTA)</label>
          <div class="hint"><b>Porta UDP:</b> 8266 · <b>Host MDNS:</b> <code id="ota-host" style="color:#90caf9">—</code>.local<br>
          In PlatformIO CLI: <code>pio run -t upload --upload-port=IP_ESP8266</code> con le direttive OTA in platformio.ini</div></div>
        <div class="f"><label>🧩 Form minimale diretto</label>
          <div class="hint">Accessibile senza JS all'indirizzo <a href="/update" style="color:#90caf9">/update</a></div></div>
      </div>

      <div class="card" style="grid-column:1 / -1">
        <h2>2 · Seleziona file firmware (.bin)</h2>
        <div id="ota-drop" style="border:2px dashed #343a44;background:#0e1116;border-radius:10px;padding:22px 14px;text-align:center;transition:all .15s;cursor:pointer">
          <div style="font-size:13px;color:var(--ink-dim);margin-bottom:8px">
            👉 Trascina qui il file <b style="color:#fff">firmware.bin</b> oppure clicca per selezionarlo
          </div>
          <input id="ota-file" type="file" accept=".bin,.bin.gz" style="display:none"/>
          <div id="ota-fname" style="font-family:var(--mono);font-size:13px;color:var(--ink);margin-top:4px">nessun file selezionato</div>
          <div id="ota-fsize" style="font-family:var(--mono);font-size:11px;color:var(--ink-more);margin-top:2px">—</div>
          <div id="ota-warn" style="display:none;margin-top:10px;padding:8px 12px;border-radius:7px;border:1px solid rgba(255,179,0,.4);background:rgba(255,179,0,.10);color:#ffd180;font-size:12px;text-align:left;font-weight:600"></div>
        </div>

        <h3 style="margin-top:16px">Avanzamento</h3>
        <div style="height:18px;background:#0b0d10;border:1px solid #343a44;border-radius:9px;overflow:hidden;position:relative">
          <div id="ota-bar" style="height:100%;width:0%;background:linear-gradient(90deg,#1e40af,#3b82f6,#60a5fa);transition:width .2s"></div>
          <div id="ota-pct" style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-family:var(--mono);font-size:11px;color:#fff;text-shadow:0 1px 2px #000">0 %</div>
        </div>

        <div class="actions">
          <button class="primary" id="btn-ota-flash" disabled>🚀 Inizia Aggiornamento</button>
          <button id="btn-ota-clear" disabled>🗑 Svuota selezione</button>
          <button class="danger" id="btn-ota-reboot">🔄 Riavvia ESP</button>
          <span style="margin-left:auto;font-family:var(--mono);font-size:11px;color:var(--ink-more)" id="ota-status">pronto</span>
        </div>
      </div>

      <div class="card" style="grid-column:1 / -1">
        <h2>3 · Note Operative</h2>
        <ul style="margin:0;padding-left:18px;color:var(--ink-dim);font-size:12px;line-height:1.8">
          <li>Durata: <b style="color:#fff">10–30s</b>. ⚠ <b style="color:#ef5350">Non spegnere</b> durante l'avanzamento; al termine riavvio automatico.</li>
          <li>Se dopo 3 min la pagina non torna online, flash via cavo USB.</li>
          <li>PlatformIO IDE: <code>upload_protocol = espota</code>, <code>upload_port = IP</code>, <code>--port=8266</code>.</li>
        </ul>
      </div>
    </div>
  </div>
</div>

<script>
(function(){
  "use strict";
  /* ================================================================
     TABS
  ================================================================ */
  const tabs = document.querySelectorAll(".tab");
  const panes = document.querySelectorAll(".pane");
  tabs.forEach(t => t.addEventListener("click", () => {
    tabs.forEach(x => x.classList.remove("active"));
    panes.forEach(x => x.classList.remove("active"));
    t.classList.add("active");
    document.getElementById("p-"+t.dataset.tab).classList.add("active");
    if(t.dataset.tab === "cfg"){
      // Carica config: se già caricato con successo e non è un "Ricarica esplicito",
      // evitiamo la FETCH per non perdere modifiche non salvate dell'utente.
      // Se il caricamento era fallito o non è ancora partito, forziamo il caricamento.
      if(!g_cfgLoaded) cfgLoad();
    }
    if(t.dataset.tab === "ota") otaInit();
  }));

  /* ================================================================
     COSTANTI MATCH FIRMWARE
  ================================================================ */
  const P0_SEALEVEL_HPA = 1013.25;
  const BEAUFORT=[[0.2,"Calm","Aria immobile"],[1.5,"Bava di vento","Fumo verticale"],
    [3.3,"Brezza leggera","Direzione vento dal fumo"],[5.4,"Brezza tesa","Foglie movimento"],
    [7.9,"Vento moderato","Polvere, carte sollevate"],[10.7,"Vento teso","Alberi ondeggiano"],
    [13.8,"Vento fresco","Rami mossi"],[17.1,"Vento forte","Alberi interi oscillano"],
    [20.7,"Burrasca","Rami spezzati"],[24.4,"Burrasca forte","Danni lievi a tetti"],
    [28.4,"Tempesta","Danni considerevoli"],[32.6,"Tempesta violenta","Danni estesi"],
    [Infinity,"Uragano","Distruzione"]];
  const ROSE16=["N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW"];
  const ROSE8=["N","NE","E","SE","S","SW","W","NW"];
  const ROWS=[
    ["cat-compass","cat","BUSSOLA · AS5600"],
    ["compass-raw","row","Raw count","0–4095","12-bit"],
    ["compass-deg","row","Angolo gradi","°","heading 0–360"],
    ["compass-deg3","row","Heading 3-cifre","°","aeronautico"],
    ["compass-rose8","row","Quadrante (8 vie)","",""],
    ["compass-rose16","row","Quadrante (16 vie)","",""],
    ["cat-hdgf","cat","FILTRO HEADING · 4 STADI (EMA Extra)"],
    ["hdgf-thr","row","Soglia Spike","°","Stage1 · reject"],
    ["hdgf-win","row","Finestra Media","N campioni","Stage2 · vettoriale"],
    ["hdgf-ema","row","α EMA Vettoriale","","Stage2.5 · (sin/cos) media ponderata"],
    ["hdgf-out","row","Periodo Output","ms","Stage3 · decimazione"],
    ["hdgf-rate","row","Frequenza Output","Hz","1000 / out_ms"],
    ["hdgf-raw","row","Raw pre-filtro","°","dopo invert+offset"],
    ["hdgf-smoo","row","Smoothed Stage2","°","media vettoriale N"],
    ["hdgf-emaout","row","Smoothed EMA out","°","Stage2.5 · EMA vettoriale"],
    ["hdgf-fill","row","Riemp. buffer","/ N","0..finestra"],
    ["hdgf-spike","row","Spike scartati","count","da avvio"],
    ["cat-anemo","cat","ANEMOMETRO · HALL A3144"],
    ["anem-us","row","Periodo impulso","µs","anti-bounce"],
    ["anem-ms","row","Velocità vento","m/s","EMA filtrato"],
    ["anem-kn","row","Velocità vento","kn","nodi"],
    ["anem-bft","row","Scala Beaufort","","BF 0..12"],
    ["anem-bftname","row","Classe Beaufort","","nome + desc."],
    ["anem-count","row","Impulsi conteggiati","","da avvio"],
    ["cat-sht31","cat","SHT31-D · TEMP / UMID (PRIMARIA)"],
    ["sht-t","row","Temperatura","°C","±0.3°C"],
    ["sht-rh","row","Umidità relativa","%RH","±2%RH"],
    ["sht-dp","row","Punto di rugiada","°C","Magnus-Tetens"],
    ["sht-fp","row","Punto di brina","°C","ghiaccio < 0°C"],
    ["sht-wb","row","Wet Bulb","°C","Stull 2011 + P"],
    ["sht-vpd","row","VPD","kPa","Vapor Press. Deficit"],
    ["sht-ah","row","Umidità assoluta","g/m³",""],
    ["sht-hi","row","Heat Index SHT","°C","NOAA locale"],
    ["sht-hx","row","Humidex SHT","","indice locale"],
    ["sht-heater","row","Heater","","antiappannamento"],
    ["cat-bmet","cat","BME280 · TEMPERATURA INTERNA"],
    ["bmet-c","row","Temperatura","°C","interno modulo"],
    ["cat-bmep","cat","BME280 · PRESSIONE"],
    ["bmep-hpa","row","Pressione","hPa","mbar"],
    ["bmep-mmhg","row","Pressione","mmHg",""],
    ["bmep-bar","row","Pressione","bar",""],
    ["bmep-delta","row","Δ vs livello mare","hPa",""],
    ["bmep-alt","row","Altitudine pressione","m","formula barometrica"],
    ["cat-bmeh","cat","BME280 · UMIDITÀ"],
    ["bmeh-rh","row","Umidità relativa","%","RH"],
    ["bmeh-dp-c","row","Punto di rugiada","°C","Magnus-Tetens"],
    ["bmeh-ah","row","Umidità assoluta","g/m³",""],
    ["cat-derived","cat","INDICI TERMICI DERIVATI"],
    ["idx-hi","row","Heat Index","°C","NOAA"],
    ["idx-wc","row","Wind Chill","°C","Canada"],
    ["idx-at","row","Apparent Temperature","°C","semplificata"],
    ["idx-humidex","row","Humidex","","indice canadese"],
    ["cat-ltr","cat","LUCE / UV · LTR390"],
    ["ltr-ch0","row","ALS Visibile Raw","count","20-bit fotopico"],
    ["ltr-uvraw","row","UV Raw Count","count","20-bit UV"],
    ["ltr-lux","row","Illuminanza","lx","datasheet"],
    ["ltr-light","row","Classe luminosità","",""],
    ["ltr-uvi","row","Indice UV (UVI)","UVI","WHO 0..11+"],
    ["ltr-uvl","row","UV Level Level",""],
    ["cat-rain","cat","PIOGGIA · BASCULA HALL"],
    ["rain-ticks","row","Tick conteggiati","","da avvio"],
    ["rain-evmm","row","Pioggia Oggi","mm","azzerata automaticamente a mezzanotte"],
    ["rain-totmm","row","Pioggia Totale","mm","persistita in EEPROM; reset manuale via MQTT o pagina"],
    ["rain-rate","row","Intensità Rain Rate","mm/h","istantanea inter-tick (no EMA)"],
    ["rain-class","row","Classe Intensità WMO","","Assente/Leggera/Mod./Forte"],
    ["rain-age","row","Ultimo tick","s","età ultima inversione"],
    ["rain-mmtick","row","mm per tick calibrato","mm","imb/massa × K"],
    ["cat-sys","cat","STATO SISTEMA"],
    ["sys-ws","row","Connessione WS","","stato"],
    ["sys-hz","row","Frequenza pacchetti","Hz","RX rate"],
    ["sys-age","row","Età ultimo dato","ms","interarrivo"],
    ["sys-ts","row","Timestamp locale","","PC"],
    ["sys-flag","row","Stato sensori bitmask","","flags OK / NC"],
    ["sys-sensors","row","Sensori collegati","","5 sensori"],
    ["sys-host","row","Host remoto","","WS endpoint"]
  ];

  /* ================================================================
     DASHBOARD BUILD + RENDER
  ================================================================ */
  const tbody=document.getElementById("rows"),ref={};
  (function(){
    for(const r of ROWS){
      const tr=document.createElement("tr");
      if(r[1]==="cat"){
        tr.classList.add("cat");const td=document.createElement("td");td.colSpan=3;td.textContent=r[2];tr.appendChild(td);
      }else{
        const a=document.createElement("td");a.className="k";a.textContent=r[2];
        const b=document.createElement("td");b.className="v";b.textContent="—";b.id="v-"+r[0];b.style.textAlign="right";
        const c=document.createElement("td");
        if(r[3]){const u=document.createElement("span");u.className="u";u.textContent=r[3];c.appendChild(u)}
        if(r[4]){const n=document.createElement("div");n.className="note";n.textContent=r[4];c.appendChild(n)}
        tr.appendChild(a);tr.appendChild(b);tr.appendChild(c);ref[r[0]]=b;
      }
      tbody.appendChild(tr);
    }
  })();
  function setv(id,val,cls){const el=ref[id];if(!el)return;el.textContent=val;el.className="v"+(cls?" "+cls:"")}
  const dot=document.getElementById("dot"),stat=document.getElementById("stat");
  function setStatus(ok,txt){dot.className="dot"+(ok?" ok":"");stat.textContent=txt}
  const fHost=document.getElementById("f-host"),fTs=document.getElementById("f-ts");
  function clamp(v,a,b){return Math.max(a,Math.min(b,v))}
  function pad3(n){return(n<10?"00":n<100?"0":"")+n}
  function cToF(c){return c*9/5+32}function cToK(c){return c+273.15}
  function beaufortOf(ms){for(let i=0;i<BEAUFORT.length;i++)if(ms<BEAUFORT[i][0])return i;return BEAUFORT.length-1}
  function roseOf(deg,arr){const step=360/arr.length;const h=step/2;let a=((deg%360)+360)%360;a=(a+h)%360;return arr[Math.floor(a/step)]}
  function dewPointC(tc,rh){const a=17.27,b=237.7;const alpha=a*tc/(b+tc)+Math.log(clamp(rh,.01,100)/100);return(b*alpha)/(a-alpha)}
  function absHumidity(tc,rh){const T=cToK(tc);const es=6.112*Math.exp(17.67*tc/(tc+243.5));const ea=es*(rh/100);return 0.21667*(ea*100)/T*1000}
  function heatIndexC(tc,rh){const T=cToF(tc),R=clamp(rh,0,100);let HI=0.5*(T+61+((T-68)*1.2)+(R*0.094));if(HI>=80)HI=-42.379+2.04901523*T+10.14333127*R-0.22475541*T*R-6.83783e-3*T*T-5.481717e-2*R*R+1.22874e-3*T*T*R+8.5282e-4*T*R*R-1.99e-6*T*T*R*R;return(HI-32)*5/9}
  function windChillC(tc,vKmh){if(tc>10||vKmh<4.8)return tc;const v16=Math.pow(vKmh,0.16);return 13.12+0.6215*tc-11.37*v16+0.3965*tc*v16}
  function apparentTempC(tc,vKmh,rh){let base=tc;if(vKmh>=4.8&&tc<=10)base=windChillC(tc,vKmh);else if(tc>=20&&rh>=40)base=heatIndexC(tc,rh);return base}
  function humidex(tc,rh){const es=6.112*Math.exp(17.67*tc/(tc+243.5));return tc+5/9*(es*(rh/100)-10)}
  function pressureAltitudeM(hpa){if(!Number.isFinite(hpa)||hpa<=0)return NaN;return 44330*(1-Math.pow(hpa/P0_SEALEVEL_HPA,1/5.255))}
  function ema(prev,val,alpha){if(prev===null)return val;return prev+alpha*(val-prev)}
  let lastTs=0,fpsCount=0,fpsLastReport=performance.now(),fpsValue=0,lastAge=0;
  let sHdg=null,sWind=null,sBmeT=null,sBmeP=null,sBmeH=null,sRaw=null,sHallUs=null,sLtrLux=null,sLtrUvi=null,sLtrCh0=null,sLtrUvRaw=null;
  let sShtT=null,sShtRh=null;
  let sRainTicks=null;
  function frostPointC(tc,rh){const a=21.8745584,b=265.5;const tf=tc<0?tc:0;const alpha=a*tf/(b+tf)+Math.log(clamp(rh,.01,100)/100);return(b*alpha)/(a-alpha)}
  function wetBulbStullC(tc,rh,p_hpa){const T=cToF(tc),R=clamp(rh,0,100),P=Number.isFinite(p_hpa)&&p_hpa>500?p_hpa:P0_SEALEVEL_HPA;const pPsi=P*0.0145037738;const wbF=T*Math.atan(0.151977*Math.sqrt(R+8.313659))+Math.atan(T+R)-Math.atan(R-1.676331)+0.00391838*Math.pow(R,1.5)*Math.atan(0.023101*R)-4.686035;const wbC0=(wbF-32)*5/9;const pCorr=Math.min(1.2,Math.max(0.9,P/P0_SEALEVEL_HPA));return wbC0*pCorr+(pPsi-14.7)*0.025}
  function vaporPressureDeficitKPa(tc,rh){const es=6.112*Math.exp(17.67*tc/(tc+243.5));const ea=es*(clamp(rh,0,100)/100);return(es-ea)/10}
  let CFG = null;
  let g_cfgLoaded = false;
  let g_cfgLoading = false;
  (function(){
    // Disabilita pulsante Salva finché non abbiamo caricato i dati da EEPROM
    const b=document.getElementById("btn-save");if(b)b.disabled=true;
  })();
  function renderAll(r){
    const host=(location.hostname||"192.168.4.1")+":81";
    fHost.textContent="ws://"+host;
    const d=new Date();fTs.textContent="dati ricevuti "+d.toLocaleTimeString("it-IT")+"."+String(d.getMilliseconds()).padStart(3,"0");
    const hFinite=Number.isFinite(r.h);const heading=hFinite?(((r.h%360)+360)%360):NaN;
    if(hFinite)sHdg=sHdg===null?heading:(()=>{let dH=heading-sHdg;while(dH>180)dH-=360;while(dH<-180)dH+=360;return sHdg+0.20*dH})();
    const hdg=sHdg!==null?sHdg:NaN;
    if(hFinite&&Number.isFinite(r.raw))sRaw=Math.round(r.raw);
    setv("compass-raw",sRaw!==null?sRaw.toString():"—");
    setv("compass-deg",Number.isFinite(hdg)?hdg.toFixed(3):"—");
    setv("compass-deg3",Number.isFinite(hdg)?pad3(Math.round(hdg%360))+"°":"—");
    setv("compass-rose8",Number.isFinite(hdg)?roseOf(hdg,ROSE8):"—");
    setv("compass-rose16",Number.isFinite(hdg)?roseOf(hdg,ROSE16):"—");
    // --- sezione Filtro Heading 4 stadi · valori trasmessi su slot 128..164 (ex AS3935) ---
    const hThrF=r.lgNf,hWin=r.lgWd,hOut=r.lgSrej,hRaw=r.lgTun,hSmo=r.lgEnergy,hFill=r.lgAge,hSCnt=r.lgCount;
    const hEmaA=r.lgMin,hEmaOut=r.lgOsc;
    if(Number.isFinite(hThrF))setv("hdgf-thr",hThrF.toFixed(2));else setv("hdgf-thr","—");
    if(Number.isFinite(hWin)&&hWin>0)setv("hdgf-win",Math.round(hWin).toLocaleString("it-IT"));else setv("hdgf-win","—");
    if(Number.isFinite(hEmaA))setv("hdgf-ema",hEmaA.toFixed(2)+"  (τ≈"+(0.05/Math.max(0.01,hEmaA)).toFixed(1)+"s)");else setv("hdgf-ema","—");
    if(Number.isFinite(hOut))setv("hdgf-out",Math.round(hOut).toLocaleString("it-IT"));else setv("hdgf-out","—");
    if(Number.isFinite(hOut)&&hOut>0)setv("hdgf-rate",(1000/hOut).toFixed(2));else setv("hdgf-rate","—");
    const hRawOk=Number.isFinite(hRaw);
    setv("hdgf-raw",hRawOk?hRaw.toFixed(3):"—",hRawOk?"":"warn");
    const hSmoOk=Number.isFinite(hSmo);
    setv("hdgf-smoo",hSmoOk?hSmo.toFixed(3):"—",hSmoOk?"":"warn");
    const hEmaOutOk=Number.isFinite(hEmaOut)&&Math.abs(hEmaOut)<=360;
    setv("hdgf-emaout",hEmaOutOk?hEmaOut.toFixed(3):"—",hEmaOutOk?"":"warn");
    if(Number.isFinite(hFill)&&Number.isFinite(hWin)&&hWin>0){
      const ff=Math.min(Math.round(hFill),Math.round(hWin));
      setv("hdgf-fill",ff+" / "+Math.round(hWin)+"  ("+Math.round(100*ff/hWin)+"%)");
    }else setv("hdgf-fill","—");
    if(Number.isFinite(hSCnt)){
      const c=Math.max(0,Math.round(hSCnt));
      let cls="ok";if(c>100)cls="warn";if(c>1000)cls="bad";
      setv("hdgf-spike",c.toLocaleString("it-IT"),cls);
    }else setv("hdgf-spike","—");
    // --- Fine sezione Filtro Heading ---
    if(Number.isFinite(r.hallUs)&&r.hallUs>0)sHallUs=r.hallUs;
    const wFinite=Number.isFinite(r.w);
    if(wFinite){sWind=ema(sWind,Math.max(0,r.w),0.20);if(sWind<0.01)sWind=0}
    const w=sWind!==null?sWind:0,kmh=w*3.6,kn=w*1.9438445,bf=beaufortOf(w),bfRow=BEAUFORT[bf];
    setv("anem-us",sHallUs!==null?Math.round(sHallUs).toLocaleString("it-IT"):"—");
    setv("anem-ms",w.toFixed(3));setv("anem-kn",kn.toFixed(2));
    setv("anem-bft","BF "+bf);setv("anem-bftname",bfRow[1]+" · "+bfRow[2]);
    setv("anem-count",Number.isFinite(r.hallCnt)?Math.round(r.hallCnt).toLocaleString("it-IT"):"—");
    const shtOk=Number.isFinite(r.sht_t)&&r.sht_t>-80&&r.sht_t<150&&Number.isFinite(r.sht_rh)&&r.sht_rh>=0&&r.sht_rh<=100;
    if(shtOk){sShtT=ema(sShtT,r.sht_t,0.15);sShtRh=ema(sShtRh,r.sht_rh,0.15)}
    const stt=sShtT!==null?sShtT:NaN,srh=sShtRh!==null?sShtRh:NaN;
    const sDp=shtOk?dewPointC(stt,srh):NaN;
    const sFp=shtOk?frostPointC(stt,srh):NaN;
    const sWb=shtOk?wetBulbStullC(stt,srh,sBmeP!==null?sBmeP:P0_SEALEVEL_HPA):NaN;
    const sVpd=shtOk?vaporPressureDeficitKPa(stt,srh):(Number.isFinite(r.sht_vpd)?r.sht_vpd:NaN);
    const sAh=shtOk?absHumidity(stt,srh):NaN;
    const sHi=shtOk?heatIndexC(stt,srh):NaN;
    const sHx=shtOk?humidex(stt,srh):NaN;
    setv("sht-t",shtOk?stt.toFixed(3):"NC",shtOk?"":"bad");
    setv("sht-rh",shtOk?srh.toFixed(2):"NC",shtOk?"":"bad");
    setv("sht-dp",Number.isFinite(sDp)?sDp.toFixed(2):"—");
    setv("sht-fp",Number.isFinite(sFp)?sFp.toFixed(2):"—");
    setv("sht-wb",Number.isFinite(sWb)?sWb.toFixed(2):"—");
    setv("sht-vpd",Number.isFinite(sVpd)?sVpd.toFixed(3):"—");
    setv("sht-ah",Number.isFinite(sAh)?sAh.toFixed(3):"—");
    setv("sht-hi",Number.isFinite(sHi)?sHi.toFixed(2):"—");
    setv("sht-hx",Number.isFinite(sHx)?sHx.toFixed(2):"—");
    const sbSht=Number.isFinite(r.status)?Math.round(r.status):0;
    const htrOn=r.sht_htr===true;const htrReady=shtOk&&(sbSht&0x20)!==0;
    let htrTxt="NC",htrCls="bad";
    if(htrReady){if(htrOn){htrTxt="ACCESO · antiappannamento";htrCls="bad"}else{const rhNow=sShtRh!==null?sShtRh:0;if(rhNow>=95){htrTxt="PRONTO · RH elevata";htrCls="warn"}else{htrTxt="Spento";htrCls="ok"}}}
    else if(!shtOk){htrTxt="sensore SHT31 non disponibile";htrCls="bad"}
    setv("sht-heater",htrTxt,htrCls);
    const btOk=Number.isFinite(r.bmet)&&r.bmet>-100&&r.bmet<150;if(btOk)sBmeT=ema(sBmeT,r.bmet,0.15);
    setv("bmet-c",btOk?sBmeT.toFixed(3):"NC",btOk?"":"bad");
    const bpOk=Number.isFinite(r.bmep)&&r.bmep>=300&&r.bmep<=1200;if(bpOk)sBmeP=ema(sBmeP,r.bmep,0.12);
    const p_hpa=sBmeP!==null?sBmeP:NaN;
    setv("bmep-hpa",bpOk?p_hpa.toFixed(2):"NC",bpOk?"":"bad");
    setv("bmep-mmhg",bpOk?(p_hpa*0.7500616827).toFixed(2):"NC",bpOk?"":"bad");
    setv("bmep-bar",bpOk?(p_hpa/1000).toFixed(5):"NC",bpOk?"":"bad");
    const dp=bpOk?(p_hpa-P0_SEALEVEL_HPA):NaN;setv("bmep-delta",bpOk?((dp>0?"+":"")+dp.toFixed(2)):"—");
    const pa=bpOk?pressureAltitudeM(p_hpa):NaN;setv("bmep-alt",bpOk?pa.toFixed(1):"NC",bpOk?"":"bad");
    const bhOk=Number.isFinite(r.bmeh)&&r.bmeh>=0&&r.bmeh<=100;if(bhOk)sBmeH=ema(sBmeH,r.bmeh,0.15);
    const rh=sBmeH!==null?sBmeH:NaN;setv("bmeh-rh",bhOk?rh.toFixed(2):"NC",bhOk?"":"bad");
    const tForDP=shtOk?sShtT:(btOk?sBmeT:NaN);
    const dpC=(shtOk||btOk)&&bhOk&&Number.isFinite(tForDP)?dewPointC(tForDP,rh):NaN;setv("bmeh-dp-c",Number.isFinite(dpC)?dpC.toFixed(2):"—");
    const ah=(shtOk||btOk)&&bhOk&&Number.isFinite(tForDP)?absHumidity(tForDP,rh):NaN;setv("bmeh-ah",Number.isFinite(ah)?ah.toFixed(3):"—");
    const tIdx=shtOk?sShtT:(btOk?sBmeT:NaN);
    const hiC=Number.isFinite(tIdx)&&bhOk?heatIndexC(tIdx,rh):NaN;setv("idx-hi",Number.isFinite(hiC)?hiC.toFixed(2):"—");
    const wcC=Number.isFinite(tIdx)&&sWind!==null?windChillC(tIdx,kmh):NaN;setv("idx-wc",Number.isFinite(wcC)?wcC.toFixed(2):"—");
    const atC=Number.isFinite(tIdx)&&sWind!==null&&bhOk?apparentTempC(tIdx,kmh,rh):NaN;setv("idx-at",Number.isFinite(atC)?atC.toFixed(2):"—");
    const hx=Number.isFinite(tIdx)&&bhOk?humidex(tIdx,rh):NaN;setv("idx-humidex",Number.isFinite(hx)?hx.toFixed(2):"—");
    const sbLtr=Number.isFinite(r.status)?Math.round(r.status):0,ltrOk=(sbLtr&0x10)!==0;
    const luxFinite=ltrOk&&Number.isFinite(r.ltrLux)&&r.ltrLux>=0;
    const uviFinite=ltrOk&&Number.isFinite(r.ltrUvi)&&r.ltrUvi>=0;
    const alsFinite=ltrOk&&Number.isFinite(r.ltrCh0)&&r.ltrCh0>=0;
    const uvFinite=ltrOk&&Number.isFinite(r.ltrUv)&&r.ltrUv>=0;
    if(luxFinite)sLtrLux=ema(sLtrLux,Math.max(0,r.ltrLux),0.20);
    if(uviFinite)sLtrUvi=ema(sLtrUvi,Math.max(0,r.ltrUvi),0.15);
    if(alsFinite)sLtrCh0=Math.round(r.ltrCh0);if(uvFinite)sLtrUvRaw=Math.round(r.ltrUv);
    const luxV=sLtrLux!==null?sLtrLux:NaN,uviV=sLtrUvi!==null?sLtrUvi:NaN;
    setv("ltr-ch0",sLtrCh0!==null?sLtrCh0.toLocaleString("it-IT"):"NC",alsFinite?"":"bad");
    setv("ltr-uvraw",sLtrUvRaw!==null?sLtrUvRaw.toLocaleString("it-IT"):"NC",uvFinite?"":"bad");
    let luxCls="NC",luxCol="bad";if(luxFinite){
      if(luxV<10){luxCls="Dark";luxCol="warn"}else if(luxV<50){luxCls="Very Dim";luxCol="warn"}
      else if(luxV<200){luxCls="Dim";luxCol=""}else if(luxV<500){luxCls="Normal Indoor";luxCol="ok"}
      else if(luxV<1000){luxCls="Bright Indoor";luxCol="ok"}else if(luxV<10000){luxCls="Overcast Out";luxCol="ok"}
      else if(luxV<25000){luxCls="Cloudy Sun";luxCol="ok"}else if(luxV<100000){luxCls="Direct Sun";luxCol="warn"}
      else{luxCls="Extreme";luxCol="bad"}}
    setv("ltr-lux",luxFinite?luxV.toFixed(1)+" lx":"NC",luxFinite?luxCol:"bad");
    setv("ltr-light",luxFinite?luxCls:"NC",luxFinite?luxCol:"bad");
    let uvLv="NC",uvCol="bad";if(uviFinite){
      if(uviV<0.5){uvLv="None";uvCol="ok"}else if(uviV<3){uvLv="Low";uvCol="ok"}
      else if(uviV<6){uvLv="Moderate";uvCol="warn"}else if(uviV<8){uvLv="High";uvCol="warn"}
      else if(uviV<11){uvLv="Very High";uvCol="bad"}else{uvLv="Extreme";uvCol="bad"}}
    setv("ltr-uvi",uviFinite?uviV.toFixed(2):"NC",uviFinite?uvCol:"bad");
    setv("ltr-uvl",uviFinite?uvLv:"NC",uviFinite?uvCol:"bad");
    // --- Rain Gauge Bascula ---
    const rTicksFinite=Number.isFinite(r.rain_t)&&r.rain_t>=0;
    const rTotalFinite=Number.isFinite(r.rain_mm)&&r.rain_mm>=0;
    // --- rinominato rain_ev → rain_oggi (offset WS 104 invariato, compat) ---
    const rOggiFinite=Number.isFinite(r.rain_oggi)&&r.rain_oggi>=0;
    const rRateFinite=Number.isFinite(r.rain_r)&&r.rain_r>=0;
    const rAgeFinite=Number.isFinite(r.rain_age)&&r.rain_age>=0;
    const rMptFinite=Number.isFinite(r.rain_mmpt)&&r.rain_mmpt>0.0001;
    if(rTicksFinite){sRainTicks=Math.round(r.rain_t)}
    setv("rain-ticks",sRainTicks!==null?sRainTicks.toLocaleString("it-IT"):"—");
    setv("rain-totmm",rTotalFinite?r.rain_mm.toFixed(2)+" mm":"—");
    // ID UI rimasto 'rain-evmm' per compatibilità browser cache, ma label e valore = oggi
    setv("rain-evmm",rOggiFinite?r.rain_oggi.toFixed(2)+" mm":"—");
    let rrTxt="—",rrCls="",rrClassWmo="Assente";
    if(rRateFinite){
      const rr=r.rain_r;
      rrTxt=rr.toFixed(2);
      if(rr<2){rrCls="ok";rrClassWmo="Leggera"}
      else if(rr<=8){rrCls="warn";rrClassWmo="Moderata"}
      else{rrCls="bad";rrClassWmo="Forte"}
      if(rr<0.05){rrClassWmo="Assente";rrCls=""}
    }
    setv("rain-rate",rRateFinite?(rrTxt+" mm/h"):"—",rrCls);
    setv("rain-class",rRateFinite?rrClassWmo:"—",rrCls);
    setv("rain-age",rAgeFinite?Math.round(r.rain_age)+" s":"—");
    setv("rain-mmtick",rMptFinite?r.rain_mmpt.toFixed(4)+" mm":"—");
    // --- Lightning AS3935 · RIMOSSO (2026-09-23) · nessun rendering ---
    setv("sys-hz",fpsValue+" Hz");setv("sys-age",lastAge+" ms");
    const n=new Date();setv("sys-ts",n.toLocaleString("it-IT")+"."+String(n.getMilliseconds()).padStart(3,"0"));
    const sb=Number.isFinite(r.status)?Math.round(r.status):0;
    setv("sys-flag","0x"+sb.toString(16).toUpperCase().padStart(3,"0")+"  b"+(sb&0x100?1:0).toString()+((sb>>>0)&0xFF).toString(2).padStart(8,"0"));
    setv("sys-sensors",((sb&0x001)?"AS5600":"-")+((sb&0x080)?"+Q":"")+" | "+((sb&0x020)?"SHT31":"-")+((sb&0x040)?"·HTR":"")+" | "+((sb&0x004)?"BME280":"-")+" | "+((sb&0x008)?"HALL":"-")+" | "+((sb&0x010)?"LTR390":"-")+" | "+((sb&0x100)?"RAIN":"-"));
    setv("sys-host",host);setv("sys-ws","Attiva","ok");
    try{
      const rt=$("cf-rtick");if(rt){rt.value=rTicksFinite?Math.round(r.rain_t).toLocaleString("it-IT"):""}
      // cf-revmm = pioggia oggi (rimasto id vecchio per compat. browser cache)
      const rev=$("cf-revmm");if(rev){rev.value=rOggiFinite?r.rain_oggi.toFixed(2):""}
    }catch(_){}
  }

  /* ================================================================
     WEBSOCKET CLIENT + ANTI-ZOMBIE (Silence Detector 30s + Backoff 1.5s→15s jitter)
  ================================================================ */
  let ws=null,wsSilenceTimer=null,wsNextDelay=1500,wsConsecFails=0;
  function wsStopSilence(){if(wsSilenceTimer){clearTimeout(wsSilenceTimer);wsSilenceTimer=null}}
  function wsStartSilence(){wsStopSilence();wsSilenceTimer=setTimeout(()=>{
    console.warn("[WS] 30s silenzio → chiudo e riconnetto");try{ws.close()}catch(_){}
  },30000)}
  function wsOnMsg(ev){
    wsStartSilence();
    let h=NaN,w=NaN,d=NaN,bt=NaN,bp=NaN,bh=NaN,raw=NaN,hu=NaN,hc=NaN,st=NaN,llux=NaN,luvi=NaN,lch0=NaN,luvr=NaN;
    let sht_t=NaN,sht_rh=NaN,sht_dp=NaN,sht_fp=NaN,sht_vpd=NaN;
    let lgC=NaN,lgE=NaN,lgA=NaN,lgN=NaN,lgW=NaN,lgS=NaN,lgTpf=NaN,lgM=NaN,lgO=NaN,lgTms=NaN;
    if(typeof ev.data==="string"){const p=String(ev.data).trim().split(/[,\s]+/);
      h=p[0]?parseFloat(p[0]):NaN;w=p[1]?parseFloat(p[1]):NaN;d=p[2]?parseFloat(p[2]):NaN;bt=p[3]?parseFloat(p[3]):NaN;
      bp=p[4]?parseFloat(p[4]):NaN;bh=p[5]?parseFloat(p[5]):NaN;raw=p[6]?parseFloat(p[6]):NaN;hu=p[7]?parseFloat(p[7]):NaN;
      hc=p[8]?parseFloat(p[8]):NaN;st=p[9]?parseFloat(p[9]):NaN;llux=p[10]?parseFloat(p[10]):NaN;luvi=p[11]?parseFloat(p[11]):NaN;
      lch0=p[12]?parseFloat(p[12]):NaN;luvr=p[13]?parseFloat(p[13]):NaN;
      // indices 14,15,16,17 reserved (diagnostica AS5600 legacy: status/agc/mag/quality)
      sht_t=p[18]?parseFloat(p[18]):NaN;sht_rh=p[19]?parseFloat(p[19]):NaN;sht_dp=p[20]?parseFloat(p[20]):NaN;sht_fp=p[21]?parseFloat(p[21]):NaN;sht_vpd=p[22]?parseFloat(p[22]):NaN;
      // WS CSV pos 25: rinominato ev → oggi (stessa posizione, compat)
      var r_t=p[23]?parseFloat(p[23]):NaN,r_mm=p[24]?parseFloat(p[24]):NaN,r_oggi=p[25]?parseFloat(p[25]):NaN,r_r=p[26]?parseFloat(p[26]):NaN,r_age=p[27]?parseFloat(p[27]):NaN,r_mmpt=p[28]?parseFloat(p[28]):NaN;
      lgC=p[29]?parseFloat(p[29]):NaN;lgE=p[30]?parseFloat(p[30]):NaN;lgA=p[31]?parseFloat(p[31]):NaN;lgN=p[32]?parseFloat(p[32]):NaN;lgW=p[33]?parseFloat(p[33]):NaN;lgS=p[34]?parseFloat(p[34]):NaN;lgTpf=p[35]?parseFloat(p[35]):NaN;lgM=p[36]?parseFloat(p[36]):NaN;lgO=p[37]?parseFloat(p[37]):NaN;lgTms=p[38]?parseFloat(p[38]):NaN;
    }else if(ev.data instanceof ArrayBuffer){const v=new DataView(ev.data);
      if(v.byteLength>=4)h=v.getFloat32(0,true);if(v.byteLength>=8)w=v.getFloat32(4,true);
      if(v.byteLength>=12)d=v.getFloat32(8,true);if(v.byteLength>=16)bt=v.getFloat32(12,true);
      if(v.byteLength>=20)bp=v.getFloat32(16,true);if(v.byteLength>=24)bh=v.getFloat32(20,true);
      if(v.byteLength>=28)raw=v.getFloat32(24,true);if(v.byteLength>=32)hu=v.getFloat32(28,true);
      if(v.byteLength>=36)hc=v.getFloat32(32,true);if(v.byteLength>=40)st=v.getFloat32(36,true);
      if(v.byteLength>=44)llux=v.getFloat32(40,true);if(v.byteLength>=48)luvi=v.getFloat32(44,true);
      if(v.byteLength>=52)lch0=v.getFloat32(48,true);if(v.byteLength>=56)luvr=v.getFloat32(52,true);
      // offsets 60,64,68,72 reserved (diagnostica AS5600 legacy: status/agc/mag/quality)
      if(v.byteLength>=80)sht_t=v.getFloat32(76,true);if(v.byteLength>=84)sht_rh=v.getFloat32(80,true);
      if(v.byteLength>=88)sht_dp=v.getFloat32(84,true);if(v.byteLength>=92)sht_fp=v.getFloat32(88,true);
      if(v.byteLength>=96)sht_vpd=v.getFloat32(92,true);
      if(v.byteLength>=100)r_t=v.getFloat32(96,true);if(v.byteLength>=104)r_mm=v.getFloat32(100,true);
      // WS binario: offset 104 = oggi (rinominato, stesso offset per compat v4/v5)
      if(v.byteLength>=108)r_oggi=v.getFloat32(104,true);if(v.byteLength>=112)r_r=v.getFloat32(108,true);
      if(v.byteLength>=116)r_age=v.getFloat32(112,true);if(v.byteLength>=120)r_mmpt=v.getFloat32(116,true);
      if(v.byteLength>=132)lgC=v.getFloat32(128,true);if(v.byteLength>=136)lgE=v.getFloat32(132,true);
      if(v.byteLength>=140)lgA=v.getFloat32(136,true);if(v.byteLength>=144)lgN=v.getFloat32(140,true);
      if(v.byteLength>=148)lgW=v.getFloat32(144,true);if(v.byteLength>=152)lgS=v.getFloat32(148,true);
      if(v.byteLength>=156)lgTpf=v.getFloat32(152,true);if(v.byteLength>=160)lgM=v.getFloat32(156,true);
      if(v.byteLength>=164)lgO=v.getFloat32(160,true);if(v.byteLength>=168)lgTms=v.getFloat32(164,true);
    }
    const now=performance.now();if(lastTs>0)lastAge=Math.round(now-lastTs);lastTs=now;fpsCount++;
    if(now-fpsLastReport>=900){fpsValue=Math.round((fpsCount*1000)/(now-fpsLastReport));fpsCount=0;fpsLastReport=now}
    const sbLocal=Number.isFinite(st)?Math.round(st):0;const sht_htr=(sbLocal&0x40)!==0;
    // renderAll: rain_oggi al posto di rain_ev (stessa semantica, nome nuovo)
    renderAll({h,w,ds18:d,bmet:bt,bmep:bp,bmeh:bh,raw,hallUs:hu,hallCnt:hc,status:st,ltrLux:llux,ltrUvi:luvi,ltrCh0:lch0,ltrUv:luvr,sht_t,sht_rh,sht_dp,sht_fp,sht_vpd,sht_htr,rain_t:r_t,rain_mm:r_mm,rain_oggi:r_oggi,rain_r:r_r,rain_age:r_age,rain_mmpt:r_mmpt,lgDist:d,lgCount:lgC,lgEnergy:lgE,lgAge:lgA,lgNf:lgN,lgWd:lgW,lgSrej:lgS,lgTun:lgTpf,lgMin:lgM,lgOsc:lgO,lgTs:lgTms});
  }
  function wsScheduleReconnect(){
    wsStopSilence();
    const jitter = (Math.random()*600-300)|0;
    const delay = Math.min(15000, wsNextDelay) + jitter;
    wsNextDelay = Math.min(15000, Math.round(wsNextDelay*1.5));
    wsConsecFails++;
    setStatus(false,`disconnesso (${wsConsecFails}) → riconnetto tra ${delay}ms`);
    setv("sys-ws","Interrotta","bad");
    setTimeout(wsConnect, delay);
  }
  function wsConnect(){
    const proto=location.protocol==="https:"?"wss":"ws";
    const host=location.hostname||"192.168.4.1";const url=proto+"://"+host+":81/";
    setStatus(false,"connessione a "+host+" ...");setv("sys-ws","In attesa","warn");
    try{ws=new WebSocket(url)}catch(e){setStatus(false,"URL non valida");wsScheduleReconnect();return}
    ws.binaryType="arraybuffer";
    ws.onopen=()=>{wsNextDelay=1500;wsConsecFails=0;wsStartSilence();setStatus(true,"connesso "+host);setv("sys-ws","Attiva","ok")};
    ws.onclose=()=>wsScheduleReconnect();
    ws.onerror=()=>{try{ws.close()}catch(_){}};
    ws.onmessage=wsOnMsg;
  }
  wsConnect();

  /* ================================================================
     CONFIGURAZIONE: CARICA / SALVA / VALIDA
  ================================================================ */
  const $=id=>document.getElementById(id);
  function uiSetTitle(deviceName){
    const h=$("h-title"); if(!h) return;
    const fwEl=document.getElementById("h-fwbuild");
    h.textContent="";
    h.appendChild(document.createTextNode(deviceName || "Stazione Meteo ESP8266"));
    if(fwEl) h.appendChild(fwEl);
  }
  function cfgStatus(t){$("cfg-status").textContent=t}
  function flash(msg,cls){const f=$("flash");f.textContent=msg;f.className="flash show "+(cls||"ok");setTimeout(()=>f.classList.remove("show"),4500)}
  function mangleStr(s,def,max){s=(s||"").toString().trim();if(s.length===0)s=def;if(s.length>max)s=s.substring(0,max);return s}
  function isPrintable(s){for(let i=0;i<s.length;i++){const c=s.charCodeAt(i);if(c<0x20||c===0x7F)return false}return true}
  function updateMqttPreview(){
    const id=mangleStr($("cf-mid").value,"device_id",63);
    $("tp-base").textContent=id;$("tp-base2").textContent=id;$("tp-base3").textContent=id;
  }
  ["cf-mid"].forEach(id=>$(id).addEventListener("input",updateMqttPreview));
  async function apiGetLabels(){
    try{const r=await fetch("/api/labels",{cache:"no-store"});if(r.ok){const j=await r.json();
      if(j&&j.device_name){uiSetTitle(j.device_name);return j}}}catch(_){}return null
  }
  async function cfgLoad(){
    if(g_cfgLoading){return}
    g_cfgLoading=true;g_cfgLoaded=false;
    const btnSave=$("btn-save");if(btnSave)btnSave.disabled=true;
    cfgStatus("caricamento /api/config ...");
    try{
      const r=await fetch("/api/config",{cache:"no-store"});
      if(!r.ok)throw new Error("HTTP "+r.status);
      const j=await r.json();CFG=j;
      $("cf-device").value=j.device_name||"";
      $("cf-as5600").value=j.sensor_as5600_name||"";
      $("cf-sht31").value=j.sensor_sht31_name||"";
      $("cf-bme280").value=j.sensor_bme280_name||"";
      $("cf-hall").value=j.sensor_hall_name||"";
      $("cf-ltr390").value=j.sensor_ltr390_name||"";
      $("cf-wssid").value=j.wifi_ssid||"";
      $("cf-wpass").value=j.wifi_password||"";
      $("cf-aps").value=j.ap_ssid||"";
      $("cf-app").value=j.ap_password||"";
      $("cf-mip").value=j.mqtt_server||"";
      $("cf-mport").value=j.mqtt_port||1883;
      $("cf-muser").value=j.mqtt_user||"";
      $("cf-mpass").value=j.mqtt_password||"";
      $("cf-mid").value=j.mqtt_device_id||"";
      $("cf-zpos").value=j.as5600_zpos|0;
      $("cf-mpos").value=j.as5600_mpos|0;
      $("cf-mang").value=j.as5600_mang|0;
      $("cf-offset").value=((j.as5600_offset_deg_100x|0)/100).toFixed(2);
      $("cf-hdgf-enable").checked=(j.hdgf_filter_enable===false||j.hdgf_filter_enable===0)?false:true;
      $("cf-hdgf-spike").value=Number((typeof j.hdgf_spike_threshold_deg==="number"?j.hdgf_spike_threshold_deg:10.00)).toFixed(2);
      $("cf-hdgf-win").value=(typeof j.hdgf_movavg_window==="number"?j.hdgf_movavg_window|0:80);
      $("cf-hdgf-out").value=(typeof j.hdgf_output_period_ms==="number"?j.hdgf_output_period_ms|0:2000);
      $("cf-hdgf-ema").value=Number((typeof j.hdgf_ema_alpha==="number"?j.hdgf_ema_alpha:0.10)).toFixed(2);
      if(typeof calcTauHdgf==="function")calcTauHdgf();
      $("cf-alt").value=Number(j.bme280_altitude_ref_m||0).toFixed(1);
      $("cf-sea").value=Number(j.bme280_sealevel_hpa||1013.25).toFixed(2);
      $("cf-gain").value=j.ltr390_gain_id|0;
      $("cf-res").value=j.ltr390_res_it_id|0;
      $("cf-wf").value=Number(j.ltr390_window_factor||1).toFixed(2);
      $("cf-mag").value=j.hall_magnets_per_rev|0;
      $("cf-r").value=Number(j.hall_anemo_radius_m||0.1).toFixed(3);
      $("cf-k").value=Number(j.hall_anemo_calib_k||1).toFixed(2);
      $("cf-hall-ema").value=Number(j.sfil_hall_ema||0.45).toFixed(2);
      $("cf-hall-decay").value=Number(j.sfil_hall_decay||0.9980).toFixed(4);
      $("cf-rainname").value=j.sensor_rain_name||"";
      $("cf-rdiam").value=Number(j.rain_funnel_dia_mm||85).toFixed(1);
      $("cf-rmass").value=Number(j.rain_tip_mass_g||9.24).toFixed(2);
      $("cf-rk").value=Number(j.rain_calib_k||1).toFixed(2);
      $("cf-rdeb").value=j.rain_dead_ms|0;
      $("cf-rainrate-ema").value=Number(j.sfil_rain_rate_ema||0.25).toFixed(2);
      calcRainMmTick();
      updateMqttPreview();
      uiSetTitle(j.device_name || "Stazione Meteo ESP8266");
      const fwEl = $("h-fwbuild");
      if (fwEl) {
        fwEl.textContent = "FW: " + (j.fw_build || "sconosciuto");
        fwEl.style.display = j.fw_build ? "inline-block" : "none";
        fwEl.title = "Data/ora compilazione firmware: " + (j.fw_build || "sconosciuto");
      }
      if($("f-build")) $("f-build").textContent = "FW: " + (j.fw_build || "sconosciuto");
      if($("cfg-build")) $("cfg-build").textContent = "FW: " + (j.fw_build || "sconosciuto") + "  ·";
      g_cfgLoaded=true;
      if(btnSave)btnSave.disabled=false;
      cfgStatus("config corrente caricata ✓");
      flash("Configurazione corrente caricata","ok");
    }catch(e){
      g_cfgLoaded=false;
      if(btnSave)btnSave.disabled=true;
      cfgStatus("ERRORE "+e.message);
      flash("Errore caricamento: "+e.message+" · Usa 'Ricarica corrente' per riprovare","err");
    }finally{
      g_cfgLoading=false;
    }
  }
  function parseFloatLocale(s){const v=String(s==null?"":s).replace(/,/g,".");const n=parseFloat(v);return Number.isFinite(n)?n:NaN}
  function calcTauHdgf(){
    const rawA=parseFloatLocale($("cf-hdgf-ema").value);
    const rawN=parseInt($("cf-hdgf-win")?$("cf-hdgf-win").value:"1",10);
    let a=Number.isFinite(rawA)?rawA:0.10;
    let n=Number.isFinite(rawN)&&rawN>=1?rawN:1;
    if(a<0)a=0; if(a>1)a=1;
    const t=$("cf-hdgf-tau");
    if(!t){ return a; }
    if(a<=0.001||n<=1){
      const tauWin=n<=1?0:(n*0.025);
      if(tauWin<=0.001&&a<=0.001){ t.value="∞ (FILTRI OFF) · N="+n+" α="+a.toFixed(2); return a; }
      const tauTot=tauWin;
      const dec=tauTot>=10?0:1;
      t.value="≈ "+tauTot.toFixed(dec)+" s · AVG("+n+") · EMA OFF · α="+a.toFixed(2);
      return a;
    }
    if(a>=0.999&&n<=1){ t.value="≈ 0 s (PASS-THRU) · N=1 α="+a.toFixed(2); return a; }
    const tauWin =n*0.025;
    const tauEma =(1.0-a)/a*0.050;
    const tauTot=tauWin+tauEma;
    const dec=tauTot>=10?0:1;
    t.value="≈ "+tauTot.toFixed(dec)+" s · AVG("+n+"="+tauWin.toFixed(2)+"s)+EMA("+tauEma.toFixed(2)+"s) · α="+a.toFixed(2);
    return a;
  }
  function _hdgfInputs(){return ["cf-hdgf-spike","cf-hdgf-win","cf-hdgf-out","cf-hdgf-ema"].map(id=>$(id)).filter(Boolean);}
  function hdgfSetEnabled(enable){
    const en=!!enable;
    _hdgfInputs().forEach(el=>{el.disabled=!en;el.style.opacity=en?"1":"0.55";});
  }
  function hdgfHighlightPreset(kind){
    const row=$("hdgf-preset-row"); if(!row)return;
    row.querySelectorAll("button[data-hdgf-kind]").forEach(b=>{
      if(b.getAttribute("data-hdgf-kind")===String(kind||"")){b.classList.add("b-primary");}
      else{b.classList.remove("b-primary");}
    });
  }
  function hdgfFlashFeedback(msg,isErr){
    const fb=$("hdgf-preset-feedback"); if(!fb)return;
    fb.textContent=msg||"\u00A0";
    fb.style.color=isErr?"#a12615":"#107e3e";
    clearTimeout(hdgfFlashFeedback._t);
    hdgfFlashFeedback._t=setTimeout(function(){fb.textContent="\u00A0";},2800);
  }
  function _fire(el,ev){try{if(!el)return;el.dispatchEvent(new Event(ev,{bubbles:true,cancelable:true}));}catch(e){}}
  function applyHdgfPreset(kind){
    const en=$("cf-hdgf-enable"),sp=$("cf-hdgf-spike"),wn=$("cf-hdgf-win"),ot=$("cf-hdgf-out"),em=$("cf-hdgf-ema");
    if(!en||!sp||!wn||!ot||!em){hdgfFlashFeedback("Controlli HDGF non trovati sulla pagina",true);return;}
    const k=String(kind||"").toLowerCase();
    let label="";
    switch(k){
      case "raw":     en.checked=false; sp.value="0.00"; wn.value="1";   ot.value="50";   em.value="0.00"; label="🔴 RAW · nessun filtro"; break;
      case "light":   en.checked=true;  sp.value="30.00"; wn.value="10";  ot.value="500";  em.value="0.70"; label="🟢 Leggero · veloce"; break;
      case "balanced":en.checked=true;  sp.value="10.00"; wn.value="80";  ot.value="2000"; em.value="0.10"; label="🔵 Bilanciato"; break;
      case "strong":  en.checked=true;  sp.value="5.00";  wn.value="200"; ot.value="5000"; em.value="0.03"; label="🟣 Forte · massimo smoothing"; break;
      default: hdgfFlashFeedback("Preset sconosciuto: "+k,true);return;
    }
    hdgfSetEnabled(en.checked);
    hdgfHighlightPreset(k);
    _fire(en,"change"); _fire(en,"input");
    _fire(sp,"input");  _fire(wn,"input");  _fire(ot,"input");  _fire(em,"input");
    try{ calcTauHdgf(); }catch(e){}
    try{ if(typeof cfgValidate==="function")cfgValidate(); }catch(e){}
    hdgfFlashFeedback("Preset caricato: "+label+" — ricorda di cliccare SALVA per rendere persistente");
  }
  if(document.addEventListener){
    document.addEventListener("DOMContentLoaded",function(){
      const en=$("cf-hdgf-enable"),ema=$("cf-hdgf-ema"),wn=$("cf-hdgf-win"),row=$("hdgf-preset-row");
      const sp=$("cf-hdgf-spike"),ot=$("cf-hdgf-out");
      if(en){
        en.addEventListener("change",function(){hdgfSetEnabled(en.checked);});
        en.addEventListener("input", function(){hdgfSetEnabled(en.checked);});
        hdgfSetEnabled(en.checked);
      }
      if(ema)ema.addEventListener("input",calcTauHdgf);
      if(wn)wn.addEventListener("input",calcTauHdgf);
      if(ema&&wn&&ot&&sp&&en){
        const a=parseFloatLocale(ema.value), n=parseInt(wn.value,10), out=parseInt(ot.value,10), spk=parseFloatLocale(sp.value);
        let guess="balanced";
        if(!en.checked || (a<=0.001&&n<=1)) guess="raw";
        else if(Math.abs(a-0.70)<0.01 && n===10) guess="light";
        else if(Math.abs(a-0.03)<0.01 && n===200) guess="strong";
        hdgfHighlightPreset(guess);
      }
      calcTauHdgf();
    });
  }
  function cfgValidate(){
    let ok=true;
    const chkStr=(id,max,required)=>{const el=$(id);let v=mangleStr(el.value,"",max);
      if(required&&v.length===0){el.classList.add("invalid");ok=false;return false}
      if(!isPrintable(v)){el.classList.add("invalid");ok=false;return false}
      el.classList.remove("invalid");return true};
    ["cf-device","cf-as5600","cf-sht31","cf-bme280","cf-hall","cf-ltr390","cf-rainname"].forEach(i=>chkStr(i,39,true));
    chkStr("cf-wssid",63,true);chkStr("cf-wpass",63,false);
    chkStr("cf-aps",31,true);chkStr("cf-app",63,false);
    if($("cf-app").value.length>0&&$("cf-app").value.length<8){$("cf-app").classList.add("invalid");ok=false}
    chkStr("cf-mip",63,true);chkStr("cf-muser",47,false);chkStr("cf-mpass",63,false);
    const mid=$("cf-mid");if(!/^[a-zA-Z0-9_-]{1,63}$/.test(mid.value||"")){mid.classList.add("invalid");ok=false}else mid.classList.remove("invalid");
    const p=parseInt($("cf-mport").value,10);if(!(p>=1&&p<=65535)){$("cf-mport").classList.add("invalid");ok=false}else $("cf-mport").classList.remove("invalid");
    ["cf-zpos","cf-mpos","cf-mang"].forEach(i=>{const n=parseInt($(i).value,10);
      if(!(n>=0&&n<=4095)){$(i).classList.add("invalid");ok=false}else $(i).classList.remove("invalid")});
    const osRaw=$("cf-offset").value;const os=parseFloatLocale(osRaw);
    if(!Number.isFinite(os)||!(os>=-360&&os<=360)){$("cf-offset").classList.add("invalid");ok=false}else $("cf-offset").classList.remove("invalid");
    const hdrfSpike=parseFloatLocale($("cf-hdgf-spike").value);
    if(!Number.isFinite(hdrfSpike)||!(hdrfSpike>=0.0&&hdrfSpike<=180.0)){$("cf-hdgf-spike").classList.add("invalid");ok=false}else $("cf-hdgf-spike").classList.remove("invalid");
    const hdrfWin=parseInt($("cf-hdgf-win").value,10);
    if(!(hdrfWin>=1&&hdrfWin<=240)){$("cf-hdgf-win").classList.add("invalid");ok=false}else $("cf-hdgf-win").classList.remove("invalid");
    const hdrfOut=parseInt($("cf-hdgf-out").value,10);
    if(!(hdrfOut>=50&&hdrfOut<=60000)){$("cf-hdgf-out").classList.add("invalid");ok=false}else $("cf-hdgf-out").classList.remove("invalid");
    const hdrfEma=parseFloatLocale($("cf-hdgf-ema").value);
    if(!Number.isFinite(hdrfEma)||!(hdrfEma>=0.00&&hdrfEma<=1.00)){$("cf-hdgf-ema").classList.add("invalid");ok=false}else $("cf-hdgf-ema").classList.remove("invalid");
    const alt=parseFloat($("cf-alt").value);if(!(alt>=-500&&alt<=9000)){$("cf-alt").classList.add("invalid");ok=false}else $("cf-alt").classList.remove("invalid");
    const sea=parseFloat($("cf-sea").value);if(!(sea>=870&&sea<=1085)){$("cf-sea").classList.add("invalid");ok=false}else $("cf-sea").classList.remove("invalid");
    const gain=parseInt($("cf-gain").value,10);if(!(gain>=0&&gain<=4)){$("cf-gain").classList.add("invalid");ok=false}else $("cf-gain").classList.remove("invalid");
    const res=parseInt($("cf-res").value,10);if(!(res>=0&&res<=7)){$("cf-res").classList.add("invalid");ok=false}else $("cf-res").classList.remove("invalid");
    const wf=parseFloat($("cf-wf").value);if(!(wf>=1&&wf<=10)){$("cf-wf").classList.add("invalid");ok=false}else $("cf-wf").classList.remove("invalid");
    const mag=parseInt($("cf-mag").value,10);if(!(mag>=1&&mag<=32)){$("cf-mag").classList.add("invalid");ok=false}else $("cf-mag").classList.remove("invalid");
    const ra=parseFloat($("cf-r").value);if(!(ra>=0.01&&ra<=1)){$("cf-r").classList.add("invalid");ok=false}else $("cf-r").classList.remove("invalid");
    const k=parseFloat($("cf-k").value);if(!(k>=0.1&&k<=5)){$("cf-k").classList.add("invalid");ok=false}else $("cf-k").classList.remove("invalid");
    const hallEma=parseFloatLocale($("cf-hall-ema").value);
    if(!Number.isFinite(hallEma)||!(hallEma>=0.01&&hallEma<=1.00)){$("cf-hall-ema").classList.add("invalid");ok=false}else $("cf-hall-ema").classList.remove("invalid");
    const hallDecay=parseFloatLocale($("cf-hall-decay").value);
    if(!Number.isFinite(hallDecay)||!(hallDecay>=0.9000&&hallDecay<=0.9999)){$("cf-hall-decay").classList.add("invalid");ok=false}else $("cf-hall-decay").classList.remove("invalid");
    const rdiam=parseFloat($("cf-rdiam").value);if(!(rdiam>=20&&rdiam<=500)){$("cf-rdiam").classList.add("invalid");ok=false}else $("cf-rdiam").classList.remove("invalid");
    const rmass=parseFloat($("cf-rmass").value);if(!(rmass>=1&&rmass<=100)){$("cf-rmass").classList.add("invalid");ok=false}else $("cf-rmass").classList.remove("invalid");
    const rk=parseFloat($("cf-rk").value);if(!(rk>=0.1&&rk<=5)){$("cf-rk").classList.add("invalid");ok=false}else $("cf-rk").classList.remove("invalid");
    const rdeb=parseInt($("cf-rdeb").value,10);if(!(rdeb>=150&&rdeb<=2000)){$("cf-rdeb").classList.add("invalid");ok=false}else $("cf-rdeb").classList.remove("invalid");
    const rainRateEma=parseFloatLocale($("cf-rainrate-ema").value);
    if(!Number.isFinite(rainRateEma)||!(rainRateEma>=0.01&&rainRateEma<=1.00)){$("cf-rainrate-ema").classList.add("invalid");ok=false}else $("cf-rainrate-ema").classList.remove("invalid");
    return ok;
  }
  function cfgGather(){
    return{
      device_name:mangleStr($("cf-device").value,"",39),
      sensor_as5600_name:mangleStr($("cf-as5600").value,"",39),
      sensor_sht31_name:mangleStr($("cf-sht31").value,"",39),
      sensor_bme280_name:mangleStr($("cf-bme280").value,"",39),
      sensor_hall_name:mangleStr($("cf-hall").value,"",39),
      sensor_ltr390_name:mangleStr($("cf-ltr390").value,"",39),
      wifi_ssid:mangleStr($("cf-wssid").value,"",63),
      wifi_password:mangleStr($("cf-wpass").value,"",63),
      ap_ssid:mangleStr($("cf-aps").value,"",31),
      ap_password:mangleStr($("cf-app").value,"",63),
      mqtt_server:mangleStr($("cf-mip").value,"",63),
      mqtt_port:parseInt($("cf-mport").value,10),
      mqtt_user:mangleStr($("cf-muser").value,"",47),
      mqtt_password:mangleStr($("cf-mpass").value,"",63),
      mqtt_device_id:mangleStr($("cf-mid").value,"",63),
      as5600_zpos:parseInt($("cf-zpos").value,10),
      as5600_mpos:parseInt($("cf-mpos").value,10),
      as5600_mang:parseInt($("cf-mang").value,10),
      as5600_offset_deg_100x:Math.round(parseFloatLocale($("cf-offset").value)*100),
      hdgf_filter_enable:!!$("cf-hdgf-enable").checked,
      hdgf_spike_threshold_deg:parseFloatLocale($("cf-hdgf-spike").value),
      hdgf_movavg_window:parseInt($("cf-hdgf-win").value,10),
      hdgf_output_period_ms:parseInt($("cf-hdgf-out").value,10),
      hdgf_ema_alpha:parseFloatLocale($("cf-hdgf-ema").value),
      bme280_altitude_ref_m:parseFloat($("cf-alt").value),
      bme280_sealevel_hpa:parseFloat($("cf-sea").value),
      ltr390_gain_id:parseInt($("cf-gain").value,10),
      ltr390_res_it_id:parseInt($("cf-res").value,10),
      ltr390_window_factor:parseFloat($("cf-wf").value),
      hall_magnets_per_rev:parseInt($("cf-mag").value,10),
      hall_anemo_radius_m:parseFloat($("cf-r").value),
      hall_anemo_calib_k:parseFloat($("cf-k").value),
      sfil_hall_ema:parseFloatLocale($("cf-hall-ema").value),
      sfil_hall_decay:parseFloatLocale($("cf-hall-decay").value),
      sensor_rain_name:mangleStr($("cf-rainname").value,"",39),
      rain_funnel_dia_mm:parseFloat($("cf-rdiam").value),
      rain_tip_mass_g:parseFloat($("cf-rmass").value),
      rain_calib_k:parseFloat($("cf-rk").value),
      rain_dead_ms:parseInt($("cf-rdeb").value,10),
      sfil_rain_rate_ema:parseFloatLocale($("cf-rainrate-ema").value)
    };
  }
  function cfgChangedWifiFields(){
    if(!CFG)return false;
    return CFG.wifi_ssid!==$("cf-wssid").value||CFG.wifi_password!==$("cf-wpass").value
      ||CFG.ap_ssid!==$("cf-aps").value||CFG.ap_password!==$("cf-app").value;
  }
  async function apiSave(){
    // Safety-net: se i dati non sono stati caricati da EEPROM, proviamo prima a caricarli.
    // Questo impedisce la validazione fallita per campi vuoti quando l'utente è troppo veloce.
    if(!g_cfgLoaded){
      cfgStatus("attesa caricamento config da EEPROM ...");
      try{await cfgLoad()}catch(_){}
      if(!g_cfgLoaded){
        cfgStatus("ERRORE: config non caricata");
        flash("Impossibile salvare: caricamento configurazione da EEPROM fallito. Usa 'Ricarica corrente' e riprova.","err");
        return;
      }
    }
    if(!cfgValidate()){cfgStatus("validazione fallita ✗");flash("Validazione fallita · controlla i campi evidenziati in ROSSO","err");return}
    const body=JSON.stringify(cfgGather());cfgStatus("salvataggio POST /api/config ...");
    const needReboot=cfgChangedWifiFields();
    try{
      const r=await fetch("/api/config",{method:"POST",headers:{"Content-Type":"application/json"},body});
      if(!r.ok)throw new Error("HTTP "+r.status);
      const j=await r.json();
      cfgStatus("salvato ✓ EEPROM");
      const wantDeg=parseFloatLocale($("cf-offset").value);
      const want   =Math.round(Number.isFinite(wantDeg)?wantDeg*100:0);
      const got    =typeof j.dbg_offset_deg_100x==="number"?Math.round(j.dbg_offset_deg_100x):null;
      const wantStr=(Number.isFinite(wantDeg)?wantDeg:0).toFixed(2)+"°";
      const gotStr =(got!=null?(got/100).toFixed(2)+"°":"?");
      if(got!=null && want!==got){
        console.warn("[CFG] Offset mismatch: richiesto="+wantStr+" ("+want+"/100) salvato="+gotStr+" ("+got+"/100)");
        flash("⚠ ATTENZIONE: Offset richiesto="+wantStr+" ("+want+"/100) ma firmware ha salvato="+gotStr+" ("+got+"/100). Aprire Seriale 115200 per [CFG-POST] trace.","warn");
      }else{
        flash("Salvato in EEPROM. Offset="+wantStr+" (firmware ha "+gotStr+"). "+(needReboot?"WiFi modificato: RIAVVIO AUTOMATICO tra 2 secondi…":"Parametri applicati a caldo. MQTT publish forzato."),needReboot?"warn":"ok");
      }
      if(needReboot){setTimeout(apiReboot,2000)}
      else{setTimeout(cfgLoad,300)}
    }catch(e){cfgStatus("ERRORE "+e.message);flash("Errore salvataggio: "+e.message,"err")}
  }
  async function apiDefaults(){
    if(!confirm("⚠ Confermi di ripristinare TUTTI i valori ai DEFAULT di fabbrica?\nCredenziali WiFi e MQTT saranno sovrascritte."))return;
    cfgStatus("POST default ...");
    try{const r=await fetch("/api/config",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({__DEFAULT__:true})});if(!r.ok)throw new Error("HTTP "+r.status);
      flash("Ripristinati default · RIAVVIO AUTOMATICO tra 3 secondi…","warn");
      setTimeout(apiReboot,3000)}catch(e){cfgStatus("ERRORE "+e.message);flash("Errore: "+e.message,"err")}
  }
  async function apiReboot(){
    flash("Riavvio ESP8266 in corso… attendere ~10s","warn");cfgStatus("POST /api/restart");
    try{await fetch("/api/restart",{method:"POST"})}catch(_){}
    setTimeout(()=>location.reload(),4500);
  }
  $("btn-save").onclick=apiSave;
  $("btn-load").onclick=cfgLoad;
  $("btn-default").onclick=apiDefaults;
  $("btn-reboot").onclick=()=>{if(confirm("Confermi il RIAVVIO del dispositivo?"))apiReboot()};
  $("btn-reset-rain").onclick=async ()=>{
    if(!confirm("Confermi l'AZZERAMENTO del conteggio PIOGGIA TOTALE?\n\n• PIOGGIA TOTALE → torna a 0 (persistito in EEPROM; dopo riavvio resta 0)\n• PIOGGIA OGGI → NON viene modificata; si azzera AUTOMATICAMENTE a mezzanotte"))return;
    try{
      const r=await fetch("/api/rain/reset",{method:"POST"});
      if(!r.ok)throw new Error("HTTP "+r.status);
      flash("Reset PIOGGIA TOTALE: OK · salvato in EEPROM ✓","ok");
      cfgStatus("reset pioggia totale EEPROM ✓");
    }catch(e){flash("Errore reset pioggia: "+e.message,"err");cfgStatus("ERR reset "+e.message)}
  };

  function calcRainMmTick(){
    const d=parseFloat($("cf-rdiam").value)||0, m=parseFloat($("cf-rmass").value)||0, K=parseFloat($("cf-rk").value)||0;
    if(d<=0||m<=0||K<=0){$("cf-rmmtick").value="";return}
    const r_cm=d/20, v_cm3=m/1;
    const mmpt=(v_cm3/(Math.PI*r_cm*r_cm))*10*K;
    $("cf-rmmtick").value=Number(mmpt).toFixed(4);
  }
  ["cf-rdiam","cf-rmass","cf-rk"].forEach(id=>{const el=$(id);if(el)el.addEventListener("input",calcRainMmTick)});

  /* ================================================================
     OTA UPLOAD JS (drag-drop, file picker, progress XMLHttpRequest)
  ================================================================ */
  let OTA={file:null,xhr:null,busy:false,curBuild:null};
  function otaStatus(t){const s=$("ota-status");if(s)s.textContent=t}
  function otaShowWarn(msg,mode){
    const w=$("ota-warn");if(!w)return;if(!msg){w.style.display="none";w.textContent="";return;}
    const ok = !!mode&&mode==="ok";
    w.style.display="block";
    w.textContent = msg;
    w.style.background = ok ? "rgba(102,187,106,.12)" : "rgba(255,179,0,.12)";
    w.style.borderColor = ok ? "rgba(102,187,106,.4)" : "rgba(255,179,0,.45)";
    w.style.color = ok ? "#a5d6a7" : "#ffd180";
  }
  function otaSetProgress(pct,bytes,tot){
    const bar=$("ota-bar"),elPct=$("ota-pct");
    const pctClamped=Math.max(0,Math.min(100,pct|0));
    if(bar)bar.style.width=pctClamped+"%";
    const bytesOk=bytes!=null,totOk=tot!=null;
    if(elPct){
      let txt=pctClamped+" %";
      if(bytesOk&&totOk)txt+="  ·  "+formatBytes(bytes)+" / "+formatBytes(tot);
      else if(bytesOk)txt+="  ·  "+formatBytes(bytes);
      elPct.textContent=txt;
    }
  }
  function formatBytes(n){if(!Number.isFinite(n)||n<0)return"0 B";if(n<1024)return n+" B";if(n<1048576)return(n/1024).toFixed(1)+" KiB";return(n/1048576).toFixed(2)+" MiB"}
  function otaFlash(msg,cls){const f=$("ota-flash");if(!f)return;f.textContent=msg||"";f.className="flash show "+(cls||"ok");setTimeout(()=>{if(f)f.classList.remove("show")},5000)}
  function otaClear(){
    OTA.file=null;OTA.busy=false;
    $("ota-fname").textContent="nessun file selezionato";
    $("ota-fsize").textContent="—";
    $("btn-ota-flash").disabled=true;
    $("btn-ota-clear").disabled=true;
    otaSetProgress(0);otaStatus("pronto");
  }
  function otaUseFile(f){
    if(!f)return;
    const nameOk=/\.bin(\.gz)?$/i.test(f.name);
    if(!nameOk){otaFlash("Formato file non valido: usa un file .bin (o .bin.gz se abilitato)","err");return}
    OTA.file=f;
    $("ota-fname").textContent=f.name;
    $("ota-fsize").textContent="dimensione: "+formatBytes(f.size)+"  ·  ultimo mod.: "+(f.lastModified?new Date(f.lastModified).toLocaleString("it-IT"):"ND");
    $("btn-ota-flash").disabled=false;
    $("btn-ota-clear").disabled=false;
    otaFlash("File pronto: verifica la dimensione prima di procedere","ok");
    otaStatus("file pronto · "+f.name);
    // Warning anti-sbaglio: confronta data file con fw attuale
    otaShowWarn(null);
    if(OTA.curBuild && f.lastModified){
      try{
        const fwParts = OTA.curBuild.match(/^(\w{3})\s+(\d{1,2})\s+(\d{4})\s+(\d{2}):(\d{2}):(\d{2})$/);
        if(fwParts){
          const months={Jan:0,Feb:1,Mar:2,Apr:3,May:4,Jun:5,Jul:6,Aug:7,Sep:8,Oct:9,Nov:10,Dec:11};
          const fwTs = Date.UTC(+fwParts[3],months[fwParts[1]]|0,+fwParts[2],+fwParts[4],+fwParts[5],+fwParts[6]);
          const fileTs = f.lastModified;
          if(fileTs < fwTs - 5*60*1000){
            otaShowWarn("⚠ ATTENZIONE: il file .bin selezionato ("+new Date(fileTs).toLocaleString("it-IT")+") è PIÙ VECCHIO del firmware attualmente in esecuzione ("+OTA.curBuild+"). Probabilmente hai caricato il file sbagliato!");
          }else{
            otaShowWarn("✅ File .bin più recente del firmware attuale · aggiornamento corretto.","ok");
          }
        }
      }catch(_){}
    }else{
      otaShowWarn("ℹ Impossibile confrontare data file con FW corrente (FW vecchio? Non supporta fw_build). Procedi con cautela.");
    }
  }
  function otaAbortXhr(){if(OTA.xhr){try{OTA.xhr.abort()}catch(_){}OTA.xhr=null;OTA.busy=false;otaStatus("annullato");otaFlash("Upload annullato","warn")}}
  function otaDoUpload(){
    if(!OTA.file){otaFlash("Nessun file selezionato","warn");return}
    if(OTA.busy)return;
    OTA.busy=true;
    $("btn-ota-flash").disabled=true;
    $("btn-ota-clear").disabled=true;
    otaSetProgress(0,0,OTA.file.size);
    otaStatus("upload in corso ...");
    const fd=new FormData();
    fd.append("firmware",OTA.file,OTA.file.name);
    const xhr=new XMLHttpRequest();
    OTA.xhr=xhr;
    xhr.open("POST","/update",true);
    xhr.upload.onprogress=(ev)=>{
      if(ev.lengthComputable){const pct=(ev.loaded*100)/ev.total;otaSetProgress(pct,ev.loaded,ev.total)}
    };
    xhr.onload=()=>{
      OTA.xhr=null;OTA.busy=false;
      let ok=false,msg="";
      try{const j=JSON.parse(xhr.responseText||"{}");ok=!!j.ok;msg=(j.error||j.msg||(xhr.status!==200?"HTTP "+xhr.status:""))}catch(_){msg=xhr.status!==200?"HTTP "+xhr.status:"risposta non JSON"}
      if(xhr.status===200&&ok){
        otaSetProgress(100,OTA.file.size,OTA.file.size);
        const rebootMs=(window.OTA&&window.OTA.rebootMs)||3500;
        otaFlash("✓ Firmware scritto correttamente. Riavvio automatico tra "+(rebootMs/1000)+" secondi… Attesa conferma nuovo FW fino a 75s.","ok");
        otaStatus("OK · reboot in corso");
        setTimeout(()=>{otaStatus("attesa riavvio ESP8266 (fino a 75s)… polling attivo")},rebootMs+200);
        // Dopo l'avvenuto reboot: polling /api/config finché build != OTA.curBuild
        const startTime = Date.now();
        const oldBuild = OTA.curBuild || "";
        let attempts = 0;
        function pollNewBuild(){
          attempts++;
          const elapsed = Date.now()-startTime;
          if(elapsed > 75000){
            otaFlash("Timeout dopo 75s: non sono riuscito a confermare il nuovo FW (potrebbe essere ancora avviamento). Forza ricarica manuale (Ctrl+Shift+R).","warn");
            otaStatus("timeout polling · ricarica manuale");
            setTimeout(()=>location.reload(true),3000);
            return;
          }
          fetch("/api/config",{cache:"no-store",credentials:"same-origin"}).then(r=>{
            if(!r.ok) throw new Error("HTTP "+r.status);
            return r.json();
          }).then(j=>{
            const nb = j.fw_build || "";
            otaStatus("poll #"+attempts+" · build trovata: "+(nb||"FW vecchio"));
            if(nb && nb !== oldBuild){
              // NUOVO FW CONFERMATO. Hard reload no cache.
              otaFlash("✅ NUOVO FIRMWARE rilevato (build "+nb+" · precedente "+(oldBuild||"sconosciuto")+"). Ricarico pagina senza cache…","ok");
              otaStatus("OK · ricarico pagina (hard refresh)");
              setTimeout(()=>{
                try{
                  if('caches' in window){caches.keys().then(keys=>Promise.all(keys.map(k=>caches.delete(k)))).catch(()=>{})}
                }catch(_){}
                window.location.replace(window.location.origin+window.location.pathname+"?_="+Date.now());
              },1200);
            }else{
              setTimeout(pollNewBuild, 3000);
            }
          }).catch(()=>{
            // ESP probabilmente ancora in reboot. Attendi.
            setTimeout(pollNewBuild, 2500);
          });
        }
        setTimeout(pollNewBuild, Math.max(rebootMs+1500, 5000));
      }else{
        otaFlash("ERRORE: "+msg+"","err");
        otaStatus("upload fallito");
        $("btn-ota-flash").disabled=false;
        $("btn-ota-clear").disabled=false;
      }
    };
    xhr.onerror=()=>{OTA.xhr=null;OTA.busy=false;otaFlash("Errore di rete / connessione persa durante upload","err");otaStatus("network error");$("btn-ota-flash").disabled=false;$("btn-ota-clear").disabled=false};
    xhr.onabort=()=>{OTA.xhr=null;OTA.busy=false;otaStatus("annullato");$("btn-ota-flash").disabled=false;$("btn-ota-clear").disabled=false};
    try{xhr.send(fd)}catch(e){otaFlash("Errore invio: "+e.message,"err");OTA.busy=false;$("btn-ota-flash").disabled=false;$("btn-ota-clear").disabled=false}
  }
  function otaInit(){
    // Wire solo una volta
    const drop=$("ota-drop"),file=$("ota-file"),flash=$("btn-ota-flash"),clear=$("btn-ota-clear"),reboot=$("btn-ota-reboot");
    if(!drop||!file||!flash||!clear||!reboot)return;
    if(window.__otaInited)return;window.__otaInited=true;
    // (A) Carica build FW corrente da /api/config per mostrare a video + confronto dopo OTA
    (async () => {
      const el = $("ota-cur-build");
      const hint = $("ota-cur-hint");
      try {
        const r = await fetch("/api/config", {cache:"no-store"});
        if (!r.ok) throw new Error("HTTP "+r.status);
        const j = await r.json();
        if (j && j.fw_build) {
          OTA.curBuild = j.fw_build;
          el.textContent = "FW: " + j.fw_build;
          el.style.color = "#9ccc65";  // verde
          if (hint) hint.innerHTML = "Versione letta in tempo reale da <b>/api/config</b>. Aggiorna la pagina dopo OTA (lo script lo fa automaticamente) per vedere una build diversa qui.";
        } else {
          el.textContent = "FW: (non supportato o risposta vecchia)";
          el.style.color = "#ffb74d";
          if (hint) hint.innerHTML = "⚠ Il firmware attuale NON invia il campo <code>fw_build</code>. Probabilmente è un firmware precedente all'aggiunta della versione build: <b>devi flashare per attivare il controllo</b>. Se invece hai appena fatto l'OTA → Ctrl+Shift+R e controlla di nuovo.";
        }
      } catch (_) {
        el.textContent = "⚠ errore lettura config /api/config";
        el.style.color = "#ef5350";
      }
    })();
    // (B) Hostname MDNS (lo ricaviamo da api/labels se possibile; fallback a location.hostname)
    (async()=>{try{const r=await fetch("/api/labels",{cache:"no-store"});if(r.ok){const j=await r.json();const name=j.device_name||"";const h=name.toLowerCase().replace(/[^a-z0-9-]+/g,"-").replace(/^-+|-+$/g,"")||"esp";const el=$("ota-host");if(el)el.textContent=h}}catch(_){}const h=(location.hostname||"esp").toLowerCase().replace(/[^a-z0-9-]+/g,"-").replace(/^-+|-+$/g,"");const el=$("ota-host");if(el&&el.textContent==="—")el.textContent=h})();
    // Wire pulsanti
    drop.addEventListener("click",()=>file.click());
    file.addEventListener("change",()=>{if(file.files&&file.files[0])otaUseFile(file.files[0])});
    flash.addEventListener("click",()=>{if(!OTA.busy)otaDoUpload()});
    clear.addEventListener("click",()=>{if(!OTA.busy)otaClear()});
    reboot.addEventListener("click",()=>{if(confirm("Confermi il RIAVVIO del dispositivo?")){otaStatus("riavvio in corso…");otaFlash("Riavvio ESP8266…","warn");fetch("/api/restart",{method:"POST"}).then(()=>setTimeout(()=>location.reload(),4500)).catch(()=>setTimeout(()=>location.reload(),4500))}});
    // Drag & drop
    ["dragenter","dragover"].forEach(ev=>drop.addEventListener(ev,(e)=>{e.preventDefault();e.stopPropagation();drop.style.borderColor="#42a5f5";drop.style.background="#0f172a"}));
    ["dragleave","drop"].forEach(ev=>drop.addEventListener(ev,(e)=>{e.preventDefault();e.stopPropagation();drop.style.borderColor="#343a44";drop.style.background="#0e1116"}));
    drop.addEventListener("drop",(e)=>{if(e.dataTransfer&&e.dataTransfer.files&&e.dataTransfer.files[0])otaUseFile(e.dataTransfer.files[0])});
    otaClear();
  }

  /* INIT: Normalizzatore virgola → punto (tastiera italiana) per TUTTI i campi decimali */
  ["cf-offset","cf-alt","cf-sea","cf-wf","cf-r","cf-k","cf-rdiam","cf-rmass","cf-rk"].forEach(id=>{
    const el=$(id);if(!el)return;
    el.addEventListener("input",()=>{if(el.value.indexOf(",")>=0)el.value=el.value.replace(/,/g,".")});
  });
  apiGetLabels();
  // Carica subito la config da EEPROM in background, così quando l'utente
  // passa al tab "Configurazione" i campi sono GIÀ popolati.
  // Impedisce l'errore "caselle vuote" se l'utente modifica un solo valore.
  cfgLoad();
  // Esporta funzioni / simboli usati da handler onclick="..." nel DOM (scope globale)
  // altrimenti onclick inline (che girano su window) ReferenceError
  if(typeof window!=="undefined"){
    if(typeof $==="function")window.$=$;
    if(typeof applyHdgfPreset==="function")window.applyHdgfPreset=applyHdgfPreset;
    if(typeof calcTauHdgf==="function")window.calcTauHdgf=calcTauHdgf;
  }
})();
</script>
</body>
</html>
)rawliteral";

// ============================================================================
// CRC32 IEEE (senza LUT, ~8 µs per Config_t 460B su ESP8266@80MHz)
// ============================================================================
static uint32_t util_crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFUL;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 1UL) crc = (crc >> 1) ^ 0xEDB88320UL;
            else           crc = (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

// ============================================================================
// Helpers stringhe safe (senza Arduino String per mem safety)
// ============================================================================
static bool util_isPrintable(const char* s, size_t maxLen) {
    if (s == nullptr) return false;
    for (size_t i = 0; i < maxLen && s[i] != '\0'; i++) {
        uint8_t c = (uint8_t)s[i];
        if (c < 0x20 || c == 0x7F) return false;
    }
    return true;
}
static bool util_isPrintableSensorId(const char* s, size_t maxLen) {
    if (s == nullptr || *s == '\0') return false;
    for (size_t i = 0; i < maxLen && s[i] != '\0'; i++) {
        char c = s[i];
        if (!( (c>='a'&&c<='z') || (c>='A'&&c<='Z') || (c>='0'&&c<='9') || c=='_' || c=='-' )) return false;
    }
    return true;
}
static void util_scpy(char* dst, const char* src, size_t dstSize) {
    if (dst == nullptr || dstSize == 0) return;
    dst[0] = '\0';
    if (src == nullptr) return;
    size_t i = 0;
    while (i + 1 < dstSize && src[i] != '\0') { dst[i] = src[i]; i++; }
    dst[i] = '\0';
}
static void util_trimInPlace(char* s) {
    if (!s) return;
    size_t len = strlen(s);
    while (len > 0 && (s[len-1]==' '||s[len-1]=='\t'||s[len-1]=='\r'||s[len-1]=='\n')) { s[--len]='\0'; }
    size_t start = 0;
    while (s[start]==' '||s[start]=='\t') start++;
    if (start > 0) memmove(s, s + start, len - start + 1);
}

// ============================================================================
// Forward declarations: helper AS5600 magnitude (split 2-byte)
// ============================================================================
static inline uint16_t cfg_getAs5600MagMin(const Config_t* c);
static inline void cfg_setAs5600MagMin(Config_t* c, uint16_t v);

// ============================================================================
// Config_t: loadDefaults, sanitize, migrateLegacy1, loadFromEeprom, saveToEeprom
// ============================================================================
static void config_setDefaults(Config_t* c) {
    if (c == nullptr) return;
    memset(c, 0, sizeof(Config_t));
    c->magic   = CONFIG_MAGIC;
    c->version = CONFIG_VERSION_CURRENT;
    util_scpy(c->device_name,          DEFAULT_DEVICE_NAME,          sizeof(c->device_name));
    util_scpy(c->sensor_as5600_name,   DEFAULT_SENSOR_AS5600,        sizeof(c->sensor_as5600_name));
    util_scpy(c->sensor_as3935_name,   DEFAULT_SENSOR_AS3935,        sizeof(c->sensor_as3935_name));
    util_scpy(c->sensor_bme280_name,   DEFAULT_SENSOR_BME280,        sizeof(c->sensor_bme280_name));
    util_scpy(c->sensor_hall_name,     DEFAULT_SENSOR_HALL,          sizeof(c->sensor_hall_name));
    util_scpy(c->sensor_ltr390_name,   DEFAULT_SENSOR_LTR390,        sizeof(c->sensor_ltr390_name));
    util_scpy(c->sensor_sht31_name,    DEFAULT_SENSOR_SHT31,         sizeof(c->sensor_sht31_name));
    util_scpy(c->sensor_rain_name,     DEFAULT_SENSOR_RAIN,          sizeof(c->sensor_rain_name));
    c->rain_funnel_dia_mm = RAIN_FUNNEL_DIA_MM_DEF;
    c->rain_tip_mass_g    = RAIN_TIP_MASS_G_DEF;
    c->rain_calib_k       = RAIN_CALIB_K_DEF;
    c->rain_dead_ms       = RAIN_DEAD_MS_DEF;
    c->rain_reserved1     = 0;
    c->rain_reserved2     = 0;
    util_scpy(c->wifi_ssid,            DEFAULT_WIFI_SSID,            sizeof(c->wifi_ssid));
    util_scpy(c->wifi_password,        DEFAULT_WIFI_PASSWORD,        sizeof(c->wifi_password));
    util_scpy(c->ap_ssid,              DEFAULT_AP_SSID,              sizeof(c->ap_ssid));
    util_scpy(c->ap_password,          DEFAULT_AP_PASSWORD,          sizeof(c->ap_password));
    util_scpy(c->mqtt_server,          DEFAULT_MQTT_SERVER,          sizeof(c->mqtt_server));
    c->mqtt_port = DEFAULT_MQTT_PORT;
    util_scpy(c->mqtt_user,            DEFAULT_MQTT_USER,            sizeof(c->mqtt_user));
    util_scpy(c->mqtt_password,        DEFAULT_MQTT_PASSWORD,        sizeof(c->mqtt_password));
    util_scpy(c->mqtt_device_id,       DEFAULT_MQTT_DEVICE_ID,       sizeof(c->mqtt_device_id));
    c->as5600_zpos = 0;
    c->as5600_mpos = 4095;
    c->as5600_mang = 0;
    c->as5600_offset_deg_100x = 0;
    c->as5600_agc_min = (uint8_t)AS5600_AGC_MIN_DEF;
    c->as5600_agc_max = (uint8_t)AS5600_AGC_MAX_DEF;
    cfg_setAs5600MagMin(c, (uint16_t)AS5600_MAG_MIN_DEF);
    c->bme280_altitude_ref_m = DEFAULT_BME_ALTITUDE_M;
    c->bme280_sealevel_hpa   = DEFAULT_BME_SEALEVEL_HPA;
    c->ltr390_gain_id        = 2;
    c->ltr390_res_it_id      = 1;
    c->ltr390_window_factor  = 1.0f;
    c->hall_magnets_per_rev  = 4;
    c->hall_anemo_radius_m   = 0.100f;
    c->hall_anemo_calib_k    = 1.0f;
    c->reserved_mqtt2 = 0;
    c->crc32 = 0;
    c->crc32 = util_crc32((const uint8_t*)c, offsetof(Config_t, crc32));
}

static bool config_sanitize(Config_t* c) {
    if (!c) return false;
    bool changed = false;
    auto repairStr = [&](char* d, size_t sz, const char* def) {
        util_trimInPlace(d);
        if (!util_isPrintable(d, sz) || strlen(d) == 0) {
            util_scpy(d, def, sz); changed = true;
        }
    };
    repairStr(c->device_name,        sizeof(c->device_name),        DEFAULT_DEVICE_NAME);
    repairStr(c->sensor_as5600_name, sizeof(c->sensor_as5600_name), DEFAULT_SENSOR_AS5600);
    repairStr(c->sensor_as3935_name, sizeof(c->sensor_as3935_name), DEFAULT_SENSOR_AS3935);
    repairStr(c->sensor_bme280_name, sizeof(c->sensor_bme280_name), DEFAULT_SENSOR_BME280);
    repairStr(c->sensor_hall_name,   sizeof(c->sensor_hall_name),   DEFAULT_SENSOR_HALL);
    repairStr(c->sensor_ltr390_name, sizeof(c->sensor_ltr390_name), DEFAULT_SENSOR_LTR390);
    repairStr(c->sensor_sht31_name,  sizeof(c->sensor_sht31_name),  DEFAULT_SENSOR_SHT31);
    repairStr(c->sensor_rain_name,   sizeof(c->sensor_rain_name),   DEFAULT_SENSOR_RAIN);
    repairStr(c->wifi_ssid,          sizeof(c->wifi_ssid),          DEFAULT_WIFI_SSID);
    repairStr(c->ap_ssid,            sizeof(c->ap_ssid),            DEFAULT_AP_SSID);
    repairStr(c->mqtt_server,        sizeof(c->mqtt_server),        DEFAULT_MQTT_SERVER);
    repairStr(c->mqtt_device_id,     sizeof(c->mqtt_device_id),     DEFAULT_MQTT_DEVICE_ID);
    // mqtt_device_id deve essere sensor_id valido (alphanum + _ + -)
    if (!util_isPrintableSensorId(c->mqtt_device_id, sizeof(c->mqtt_device_id))) {
        util_scpy(c->mqtt_device_id, DEFAULT_MQTT_DEVICE_ID, sizeof(c->mqtt_device_id));
        changed = true;
    }
    // WiFi password: solo se non vuota deve essere stampabile
    util_trimInPlace(c->wifi_password);
    if (strlen(c->wifi_password) > 0 && !util_isPrintable(c->wifi_password, sizeof(c->wifi_password))) {
        c->wifi_password[0] = '\0'; changed = true;
    }
    // AP password: se presente min 8 char, altrimenti svuota -> default
    util_trimInPlace(c->ap_password);
    if (strlen(c->ap_password) > 0 && (strlen(c->ap_password) < 8 || !util_isPrintable(c->ap_password, sizeof(c->ap_password)))) {
        util_scpy(c->ap_password, DEFAULT_AP_PASSWORD, sizeof(c->ap_password));
        changed = true;
    }
    // Numerici + clamps
    auto clampU = [&](uint16_t& v, uint16_t lo, uint16_t hi){ if(v<lo||v>hi){v=(v<lo?lo:hi);changed=true;} };
    auto clampS = [&](int16_t& v, int16_t lo, int16_t hi){ if(v<lo||v>hi){v=(v<lo?lo:hi);changed=true;} };
    auto clampF = [&](float& v, float lo, float hi){ if(!isfinite(v)||v<lo||v>hi){v=(isfinite(v)?(v<lo?lo:hi):((lo+hi)/2.0f));changed=true;} };
    auto clampU8 = [&](uint8_t& v, uint8_t lo, uint8_t hi){ if(v<lo||v>hi){v=(v<lo?lo:hi);changed=true;} };
    clampU(c->as5600_zpos, 0, 4095);
    clampU(c->as5600_mpos, 0, 4095);
    clampU(c->as5600_mang, 0, 4095);
    clampS(c->as5600_offset_deg_100x, (int16_t)-32768, (int16_t)32767);
    // AS5600 soglie qualita' (ex reserved) → default se fuori range o zero (migrazione)
    {
      uint8_t amin = c->as5600_agc_min;
      uint8_t amax = c->as5600_agc_max;
      uint16_t mmin = cfg_getAs5600MagMin(c);
      bool need = false;
      if (amin == 0 || amin > 128) { amin = (uint8_t)AS5600_AGC_MIN_DEF; need = true; }
      if (amax == 0 || amax < 128) { amax = (uint8_t)AS5600_AGC_MAX_DEF; need = true; }
      if (amin >= amax) { amin = (uint8_t)AS5600_AGC_MIN_DEF; amax = (uint8_t)AS5600_AGC_MAX_DEF; need = true; }
      if (mmin < 256 || mmin > 4000) { mmin = (uint16_t)AS5600_MAG_MIN_DEF; need = true; }
      if (need) {
        c->as5600_agc_min = amin;
        c->as5600_agc_max = amax;
        cfg_setAs5600MagMin(c, mmin);
        changed = true;
      }
    }
    if (c->mqtt_port < 1 || c->mqtt_port > 65535) { c->mqtt_port = DEFAULT_MQTT_PORT; changed = true; }
    clampF(c->bme280_altitude_ref_m, -500.0f, 9000.0f);
    clampF(c->bme280_sealevel_hpa,   870.0f,   1085.0f);
    clampU8(c->ltr390_gain_id, 0, 4);
    clampU8(c->ltr390_res_it_id, 0, 7);
    clampF(c->ltr390_window_factor, 1.0f, 10.0f);
    clampU8(c->hall_magnets_per_rev, 1, 32);
    clampF(c->hall_anemo_radius_m, 0.01f, 1.0f);
    clampF(c->hall_anemo_calib_k,  0.1f,  5.0f);
    clampF(c->rain_funnel_dia_mm, 20.0f,  500.0f);
    clampF(c->rain_tip_mass_g,     1.0f,  100.0f);
    clampF(c->rain_calib_k,        0.1f,    5.0f);
    {
      uint16_t rd = c->rain_dead_ms;
      if (rd < 10 || rd > 2000) { rd = RAIN_DEAD_MS_DEF; changed = true; }
      c->rain_dead_ms = rd;
    }
    c->magic   = CONFIG_MAGIC;
    c->version = CONFIG_VERSION_CURRENT;
    c->reserved_mqtt2 = 0;
    c->hall_reserved1 = c->hall_reserved2 = c->hall_reserved3 = 0;
    c->ltr390_reserved1 = c->ltr390_reserved2 = 0;
    c->rain_reserved1 = c->rain_reserved2 = 0;
    c->crc32 = 0;
    c->crc32 = util_crc32((const uint8_t*)c, offsetof(Config_t, crc32));
    return changed;
}

static void config_saveToEeprom(const Config_t* c) {
    if (!c) return;
    EEPROM.begin(EEPROM_SIZE_BYTES);
    EEPROM.put(0, *c);
    EEPROM.commit();
    EEPROM.end();
}
static bool config_loadFromEeprom(Config_t* out) {
    if (!out) return false;
    EEPROM.begin(EEPROM_SIZE_BYTES);
    Config_t tmp; memset(&tmp, 0, sizeof(tmp));
    EEPROM.get(0, tmp);
    EEPROM.end();
    if (tmp.magic != CONFIG_MAGIC) return false;
    if (tmp.version == CONFIG_VERSION_LEGACY) {
        config_setDefaults(out);
        config_sanitize(out);
        Serial.println(F("[CONFIG] Migrazione v1→v5: applicati default compatibili."));
        return true;
    }
    if (tmp.version == CONFIG_VERSION_PREV2) {
        // Migrazione v2 (senza SHT31) → v5 (con AS3935 rename campo ds18)
        // Procedimento SAFE identico a v2→v3: defaults → copia campi v2
        Config_t v5out;
        config_setDefaults(&v5out);
        v5out.magic = tmp.magic;
        v5out.version = CONFIG_VERSION_CURRENT;
        v5out.as5600_agc_min    = tmp.as5600_agc_min;
        v5out.as5600_agc_max    = tmp.as5600_agc_max;
        v5out.as5600_mag_min_lo = tmp.as5600_mag_min_lo;
        util_scpy(v5out.device_name,         tmp.device_name,         sizeof(v5out.device_name));
        util_scpy(v5out.sensor_as5600_name,  tmp.sensor_as5600_name,  sizeof(v5out.sensor_as5600_name));
        util_scpy(v5out.sensor_as3935_name,  tmp.sensor_as3935_name,  sizeof(v5out.sensor_as3935_name));
        util_scpy(v5out.sensor_bme280_name,  tmp.sensor_bme280_name,  sizeof(v5out.sensor_bme280_name));
        util_scpy(v5out.sensor_hall_name,    tmp.sensor_hall_name,    sizeof(v5out.sensor_hall_name));
        util_scpy(v5out.sensor_ltr390_name,  tmp.sensor_ltr390_name,  sizeof(v5out.sensor_ltr390_name));
        // sensor_sht31_name + tutti i campi sensor_rain_* restano a default
        util_scpy(v5out.wifi_ssid,           tmp.wifi_ssid,           sizeof(v5out.wifi_ssid));
        util_scpy(v5out.wifi_password,       tmp.wifi_password,       sizeof(v5out.wifi_password));
        util_scpy(v5out.ap_ssid,             tmp.ap_ssid,             sizeof(v5out.ap_ssid));
        util_scpy(v5out.ap_password,         tmp.ap_password,         sizeof(v5out.ap_password));
        util_scpy(v5out.mqtt_server,         tmp.mqtt_server,         sizeof(v5out.mqtt_server));
        v5out.mqtt_port = tmp.mqtt_port;
        v5out.as5600_mag_min_hi = tmp.as5600_mag_min_hi;
        v5out.reserved_mqtt2    = tmp.reserved_mqtt2;
        util_scpy(v5out.mqtt_user,           tmp.mqtt_user,           sizeof(v5out.mqtt_user));
        util_scpy(v5out.mqtt_password,       tmp.mqtt_password,       sizeof(v5out.mqtt_password));
        util_scpy(v5out.mqtt_device_id,      tmp.mqtt_device_id,      sizeof(v5out.mqtt_device_id));
        v5out.as5600_zpos = tmp.as5600_zpos;
        v5out.as5600_mpos = tmp.as5600_mpos;
        v5out.as5600_mang = tmp.as5600_mang;
        v5out.as5600_offset_deg_100x = tmp.as5600_offset_deg_100x;
        v5out.bme280_altitude_ref_m = tmp.bme280_altitude_ref_m;
        v5out.bme280_sealevel_hpa   = tmp.bme280_sealevel_hpa;
        v5out.ltr390_gain_id        = tmp.ltr390_gain_id;
        v5out.ltr390_res_it_id      = tmp.ltr390_res_it_id;
        v5out.ltr390_reserved1      = tmp.ltr390_reserved1;
        v5out.ltr390_reserved2      = tmp.ltr390_reserved2;
        v5out.ltr390_window_factor  = tmp.ltr390_window_factor;
        v5out.hall_magnets_per_rev  = tmp.hall_magnets_per_rev;
        v5out.hall_reserved1        = tmp.hall_reserved1;
        v5out.hall_reserved2        = tmp.hall_reserved2;
        v5out.hall_reserved3        = tmp.hall_reserved3;
        v5out.hall_anemo_radius_m   = tmp.hall_anemo_radius_m;
        v5out.hall_anemo_calib_k    = tmp.hall_anemo_calib_k;
        // campi rain_* + sensor_sht31_name = a default (da config_setDefaults)
        config_sanitize(&v5out);
        *out = v5out;
        config_saveToEeprom(out);
        Serial.println(F("[CONFIG] Migrazione v2→v5 OK · salvata in EEPROM."));
        return true;
    }
    if (tmp.version == CONFIG_VERSION_PREV) {
        // Migrazione v3 → v5 (rinominato campo sensor_ds18_name → sensor_as3935_name)
        // Siccome i campi sensore sono a dimensione fissa char[40], tutti gli offset
        // da magic fino a rain_reserved2 sono IDENTICI tra v3 e v5 (a parità di semantica byte).
        // Strategia SAFE: defaults v5 → copia per-field tutti i campi v3 da tmp
        // (nuovo rinominato as3935 eredita i byte di ds18; rain + sensor_sht31_name = già a default).
        Config_t v5out;
        config_setDefaults(&v5out);
        v5out.magic = tmp.magic;
        v5out.version = CONFIG_VERSION_CURRENT;
        v5out.as5600_agc_min    = tmp.as5600_agc_min;
        v5out.as5600_agc_max    = tmp.as5600_agc_max;
        v5out.as5600_mag_min_lo = tmp.as5600_mag_min_lo;
        v5out.as5600_mag_min_hi = tmp.as5600_mag_min_hi;
        v5out.reserved_mqtt2    = tmp.reserved_mqtt2;
        util_scpy(v5out.device_name,         tmp.device_name,         sizeof(v5out.device_name));
        util_scpy(v5out.sensor_as5600_name,  tmp.sensor_as5600_name,  sizeof(v5out.sensor_as5600_name));
        util_scpy(v5out.sensor_as3935_name,  tmp.sensor_as3935_name,  sizeof(v5out.sensor_as3935_name));
        util_scpy(v5out.sensor_bme280_name,  tmp.sensor_bme280_name,  sizeof(v5out.sensor_bme280_name));
        util_scpy(v5out.sensor_hall_name,    tmp.sensor_hall_name,    sizeof(v5out.sensor_hall_name));
        util_scpy(v5out.sensor_ltr390_name,  tmp.sensor_ltr390_name,  sizeof(v5out.sensor_ltr390_name));
        util_scpy(v5out.sensor_sht31_name,   tmp.sensor_sht31_name,   sizeof(v5out.sensor_sht31_name));
        util_scpy(v5out.wifi_ssid,           tmp.wifi_ssid,           sizeof(v5out.wifi_ssid));
        util_scpy(v5out.wifi_password,       tmp.wifi_password,       sizeof(v5out.wifi_password));
        util_scpy(v5out.ap_ssid,             tmp.ap_ssid,             sizeof(v5out.ap_ssid));
        util_scpy(v5out.ap_password,         tmp.ap_password,         sizeof(v5out.ap_password));
        util_scpy(v5out.mqtt_server,         tmp.mqtt_server,         sizeof(v5out.mqtt_server));
        v5out.mqtt_port = tmp.mqtt_port;
        util_scpy(v5out.mqtt_user,           tmp.mqtt_user,           sizeof(v5out.mqtt_user));
        util_scpy(v5out.mqtt_password,       tmp.mqtt_password,       sizeof(v5out.mqtt_password));
        util_scpy(v5out.mqtt_device_id,      tmp.mqtt_device_id,      sizeof(v5out.mqtt_device_id));
        v5out.as5600_zpos = tmp.as5600_zpos;
        v5out.as5600_mpos = tmp.as5600_mpos;
        v5out.as5600_mang = tmp.as5600_mang;
        v5out.as5600_offset_deg_100x = tmp.as5600_offset_deg_100x;
        v5out.bme280_altitude_ref_m = tmp.bme280_altitude_ref_m;
        v5out.bme280_sealevel_hpa   = tmp.bme280_sealevel_hpa;
        v5out.ltr390_gain_id        = tmp.ltr390_gain_id;
        v5out.ltr390_res_it_id      = tmp.ltr390_res_it_id;
        v5out.ltr390_reserved1      = tmp.ltr390_reserved1;
        v5out.ltr390_reserved2      = tmp.ltr390_reserved2;
        v5out.ltr390_window_factor  = tmp.ltr390_window_factor;
        v5out.hall_magnets_per_rev  = tmp.hall_magnets_per_rev;
        v5out.hall_reserved1        = tmp.hall_reserved1;
        v5out.hall_reserved2        = tmp.hall_reserved2;
        v5out.hall_reserved3        = tmp.hall_reserved3;
        v5out.hall_anemo_radius_m   = tmp.hall_anemo_radius_m;
        v5out.hall_anemo_calib_k    = tmp.hall_anemo_calib_k;
        util_scpy(v5out.sensor_rain_name,    tmp.sensor_rain_name,    sizeof(v5out.sensor_rain_name));
        v5out.rain_funnel_dia_mm    = tmp.rain_funnel_dia_mm;
        v5out.rain_tip_mass_g       = tmp.rain_tip_mass_g;
        v5out.rain_calib_k          = tmp.rain_calib_k;
        v5out.rain_dead_ms          = tmp.rain_dead_ms;
        v5out.rain_reserved1        = tmp.rain_reserved1;
        v5out.rain_reserved2        = tmp.rain_reserved2;
        config_sanitize(&v5out);
        *out = v5out;
        config_saveToEeprom(out);
        Serial.println(F("[CONFIG] Migrazione v3→v5 OK · salvata in EEPROM."));
        return true;
    }
    if (tmp.version == CONFIG_VERSION_LAST) {
        // Migrazione v4 → v5 (rinominato campo sensor_ds18_name → sensor_as3935_name; size identica char[40])
        // Strategia SAFE: defaults v5 → copia per-field tutti i campi v4 da tmp;
        // nuovo campo as3935 a default DEFAULT_SENSOR_AS3935 (rinuncia consapevole label DS18 custom semantica diversa).
        Config_t v5out;
        config_setDefaults(&v5out);
        v5out.magic = tmp.magic;
        v5out.version = CONFIG_VERSION_CURRENT;
        v5out.as5600_agc_min    = tmp.as5600_agc_min;
        v5out.as5600_agc_max    = tmp.as5600_agc_max;
        v5out.as5600_mag_min_lo = tmp.as5600_mag_min_lo;
        v5out.as5600_mag_min_hi = tmp.as5600_mag_min_hi;
        v5out.reserved_mqtt2    = tmp.reserved_mqtt2;
        util_scpy(v5out.device_name,         tmp.device_name,         sizeof(v5out.device_name));
        util_scpy(v5out.sensor_as5600_name,  tmp.sensor_as5600_name,  sizeof(v5out.sensor_as5600_name));
        util_scpy(v5out.sensor_as3935_name,  DEFAULT_SENSOR_AS3935,   sizeof(v5out.sensor_as3935_name));
        util_scpy(v5out.sensor_bme280_name,  tmp.sensor_bme280_name,  sizeof(v5out.sensor_bme280_name));
        util_scpy(v5out.sensor_hall_name,    tmp.sensor_hall_name,    sizeof(v5out.sensor_hall_name));
        util_scpy(v5out.sensor_ltr390_name,  tmp.sensor_ltr390_name,  sizeof(v5out.sensor_ltr390_name));
        util_scpy(v5out.sensor_sht31_name,   tmp.sensor_sht31_name,   sizeof(v5out.sensor_sht31_name));
        util_scpy(v5out.sensor_rain_name,    tmp.sensor_rain_name,    sizeof(v5out.sensor_rain_name));
        util_scpy(v5out.wifi_ssid,           tmp.wifi_ssid,           sizeof(v5out.wifi_ssid));
        util_scpy(v5out.wifi_password,       tmp.wifi_password,       sizeof(v5out.wifi_password));
        util_scpy(v5out.ap_ssid,             tmp.ap_ssid,             sizeof(v5out.ap_ssid));
        util_scpy(v5out.ap_password,         tmp.ap_password,         sizeof(v5out.ap_password));
        util_scpy(v5out.mqtt_server,         tmp.mqtt_server,         sizeof(v5out.mqtt_server));
        v5out.mqtt_port = tmp.mqtt_port;
        util_scpy(v5out.mqtt_user,           tmp.mqtt_user,           sizeof(v5out.mqtt_user));
        util_scpy(v5out.mqtt_password,       tmp.mqtt_password,       sizeof(v5out.mqtt_password));
        util_scpy(v5out.mqtt_device_id,      tmp.mqtt_device_id,      sizeof(v5out.mqtt_device_id));
        v5out.as5600_zpos = tmp.as5600_zpos;
        v5out.as5600_mpos = tmp.as5600_mpos;
        v5out.as5600_mang = tmp.as5600_mang;
        v5out.as5600_offset_deg_100x = tmp.as5600_offset_deg_100x;
        v5out.bme280_altitude_ref_m = tmp.bme280_altitude_ref_m;
        v5out.bme280_sealevel_hpa   = tmp.bme280_sealevel_hpa;
        v5out.ltr390_gain_id        = tmp.ltr390_gain_id;
        v5out.ltr390_res_it_id      = tmp.ltr390_res_it_id;
        v5out.ltr390_reserved1      = tmp.ltr390_reserved1;
        v5out.ltr390_reserved2      = tmp.ltr390_reserved2;
        v5out.ltr390_window_factor  = tmp.ltr390_window_factor;
        v5out.hall_magnets_per_rev  = tmp.hall_magnets_per_rev;
        v5out.hall_reserved1        = tmp.hall_reserved1;
        v5out.hall_reserved2        = tmp.hall_reserved2;
        v5out.hall_reserved3        = tmp.hall_reserved3;
        v5out.hall_anemo_radius_m   = tmp.hall_anemo_radius_m;
        v5out.hall_anemo_calib_k    = tmp.hall_anemo_calib_k;
        v5out.rain_funnel_dia_mm    = tmp.rain_funnel_dia_mm;
        v5out.rain_tip_mass_g       = tmp.rain_tip_mass_g;
        v5out.rain_calib_k          = tmp.rain_calib_k;
        v5out.rain_dead_ms          = tmp.rain_dead_ms;
        v5out.rain_reserved1        = tmp.rain_reserved1;
        v5out.rain_reserved2        = tmp.rain_reserved2;
        config_sanitize(&v5out);
        *out = v5out;
        config_saveToEeprom(out);
        Serial.println(F("[CONFIG] Migrazione v4→v5 OK · salvata in EEPROM."));
        return true;
    }
    if (tmp.version != CONFIG_VERSION_CURRENT) return false;
    uint32_t calc = util_crc32((const uint8_t*)&tmp, offsetof(Config_t, crc32));
    if (calc != tmp.crc32) return false;
    *out = tmp;
    return true;
}

// ============================================================================
// Istanze server e sensori (prima dei subsystems crash/supervisore/mqtt per ordine dichiarazioni)
// ============================================================================
static ESP8266WebServer  g_http(HTTP_PORT);
static WebSocketsServer  g_ws(WS_PORT);

static Adafruit_BME280   g_bme;
static bool              g_bme_ok    = false;
static bool     g_safe_mode       = false;   // [REGOLA OTA #5] vedi safemode_evaluateAtBoot()
static uint32_t g_safe_boot_count = 0;

// ============================================================================
// Istanze MQTT Home Assistant
// ============================================================================
static WiFiClient   g_wifiClient;
static PubSubClient g_mqtt(g_wifiClient);
static bool         g_mqtt_discovery_sent = false;
static void mqttOnMessage(char* topic, uint8_t* payload, unsigned int length);

// ============================================================================
// OTA · Costanti + Stato Runtime (HTTP Update + ArduinoOTA + MDNS)
//   - Safe-WDT: wdtFeed forzato nella progress ogni ~64 KiB
//   - Supervisor bypass: g_ota_in_progress=true inibisce escalation L1..L4
// ============================================================================
#define OTA_WDT_FEED_BYTES       65536UL   // feed watchdog ogni 64 KiB scritti
#define OTA_POST_SUCCESS_REBOOT_MS 3500UL  // ms tra update OK e reboot
#define OTA_ARDUINO_PORT          8266     // porta UDP per ArduinoOTA
#define OTA_BOOTLOADER_MAGIC_BYTE   0xE9U  // 1° byte di un firmware .bin con bootloader (firmware completo seriale)
#define OTA_BOOTLOADER_SKIP_BYTES 0x10000UL // 64 KiB: bootloader + padding in .bin prodotto da PlatformIO elf2bin.py

static bool     g_ota_in_progress   = false;
static bool     g_ota_supv_armed    = false; // backup supervisor.tripped stato
static supv_state_t g_ota_supv_state = SUPV_IDLE;
static uint32_t g_ota_last_feed_ms  = 0;
static size_t   g_ota_total         = 0;
static size_t   g_ota_total_user    = 0; // byte utili (tolto eventuale bootloader in testa)
static size_t   g_ota_skip_remaining = 0;
static bool     g_ota_detected_full_fw = false;
static bool     g_ota_begun         = false; // Update.begin() chiamato? (lazy al PRIMO chunk write dopo magic-detect)
static String   g_ota_md5_str;
static size_t   g_ota_written       = 0;
static int      g_ota_last_err      = 0;   // Update.getLastError()
static char     g_ota_hostname[64];        // MDNS-safe (<=63 char, [a-z0-9-])

static void ota_sanitizeHostname(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    dst[0] = '\0';
    if (!src) return;
    size_t j = 0;
    char prev = '-';
    for (size_t i = 0; src[i] != '\0' && j + 1 < dstSize; i++) {
        char c = src[i];
        if (c >= 'A' && c <= 'Z') c = (char)(c + ('a' - 'A'));
        bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (c == '-');
        if (!ok) c = '-';
        if (c == '-' && prev == '-') continue;
        dst[j++] = c;
        prev = c;
    }
    while (j > 0 && dst[j-1] == '-') { j--; }
    if (j == 0) { dst[j++] = 'e'; dst[j++] = 's'; dst[j++] = 'p'; }
    dst[j] = '\0';
}

// ============================================================================
// Helper inline AS5600 magnitude_min (uint16_t split su due uint8_t reserved)
// ============================================================================
static inline uint16_t cfg_getAs5600MagMin(const Config_t* c) {
    if (!c) return AS5600_MAG_MIN_DEF;
    return (uint16_t)(((uint16_t)c->as5600_mag_min_hi << 8) | (uint16_t)c->as5600_mag_min_lo);
}
static inline void cfg_setAs5600MagMin(Config_t* c, uint16_t v) {
    if (!c) return;
    c->as5600_mag_min_lo = (uint8_t)(v & 0x00FF);
    c->as5600_mag_min_hi = (uint8_t)((v >> 8) & 0x00FF);
}

// ============================================================================
// Snapshot dei dati sensori per publish MQTT 1Hz (aggiornati nel loop 100Hz)
static float        g_snap_heading_deg    = NAN;
static uint16_t     g_snap_as5600_raw     = 0;
static uint8_t      g_snap_as5600_status  = 0;     // registro STATUS
static uint8_t      g_snap_as5600_agc     = 0;     // registro AGC
static uint16_t     g_snap_as5600_mag     = 0;     // registro MAGNITUDE (12-bit)
static float        g_snap_as5600_quality = NAN;   // 2=OK, 1=WARN, 0=FAIL
static float        g_snap_wind_ms        = 0.0f;
static uint32_t     g_snap_hall_period_us = 0;
static uint32_t     g_snap_hall_count     = 0;
static float        g_snap_bme_t          = NAN;
static float        g_snap_bme_p          = NAN;
static float        g_snap_bme_h          = NAN;
static uint32_t     g_snap_status_bits    = 0;

// ============================================================================
// [NUOVE AGGIUNTE MQTT 2026-03] - Globali per publish M1/M4/M6/M7/M8/M9
// ============================================================================
// M4 · Build Date / Time Version (statica da compile-time)
const char g_fwBuildVersion[] = __DATE__ " @ " __TIME__;

// M7 · Contatore riconnessioni MQTT (incrementato nel reconnect callback)
uint32_t g_mqttReconnectCount = 0;

// M8 · Wind Gust Max 10 minuti: sliding window 60s × 10 buckets = 600s
#define GUST_BUCKETS_N  10
#define GUST_BUCKET_MS  60000UL
static float    s_gustBuckets[GUST_BUCKETS_N] = {0};
static uint8_t  s_gustCurBucket = 0;
static unsigned long s_gustLastBucketMs = 0;
static float    s_gustCurBucketMax = 0.0f;
float           g_windGustMax10Min = 0.0f;    // export: mqttPublishGrpB() usa extern

static void gustTickSample(float windMs, unsigned long now)
{
    if (!isfinite(windMs) || windMs < 0.0f) windMs = 0.0f;

    if (s_gustLastBucketMs == 0) {
        s_gustLastBucketMs = now;
        s_gustCurBucketMax = windMs;
        return;
    }
    if (windMs > s_gustCurBucketMax) s_gustCurBucketMax = windMs;

    if ((now - s_gustLastBucketMs) >= GUST_BUCKET_MS) {
        unsigned long elapsed = now - s_gustLastBucketMs;
        uint8_t advance = (uint8_t)(elapsed / GUST_BUCKET_MS);
        if (advance > GUST_BUCKETS_N) advance = GUST_BUCKETS_N;
        for (uint8_t k = 0; k < advance; k++) {
            s_gustBuckets[s_gustCurBucket] = (k == 0) ? s_gustCurBucketMax : 0.0f;
            s_gustCurBucket = (s_gustCurBucket + 1u) % GUST_BUCKETS_N;
        }
        s_gustLastBucketMs += (unsigned long)advance * GUST_BUCKET_MS;
        s_gustCurBucketMax = windMs;
        // calcolo max finestra complessiva
        float wMax = 0.0f;
        for (uint8_t i = 0; i < GUST_BUCKETS_N; i++) {
            if (s_gustBuckets[i] > wMax) wMax = s_gustBuckets[i];
        }
        if (s_gustCurBucketMax > wMax) wMax = s_gustCurBucketMax;
        g_windGustMax10Min = wMax;
    }
}

// ============================================================================
// [NUOVO] Heading Filter · Variabili runtime (3 stadi)
// ============================================================================
static HeadingFilterConfig_t g_hdrf_cfg;          // parametri caricati da EEPROM
static float  g_hdrf_last_valid_deg    = NAN;     // ultimo campione ACCETTATO (post-spike)
static unsigned long g_hdrf_last_valid_ms = 0;    // FIX 2: timestamp ultimo last_valid accettato (anti-freeze 2s)
static uint32_t g_hdrf_consec_rejects   = 0;      // FIX 2: conteggio spike REJECT consecutivi (anti-freeze)
static float  g_hdrf_circbuf[HDRFILT_WINDOW_MAX]; // buffer circolare N campioni (max 80)
static uint8_t g_hdrf_circbuf_head      = 0;      // indice prossimo slot da sovrascrivere
static uint8_t g_hdrf_circbuf_fill      = 0;      // quanti slot validi (0..window)
static float  g_hdrf_smoothed_deg       = NAN;    // uscita stadio 2 (media vettoriale)
static unsigned long g_hdrf_last_output_ms = 0;   // timestamp ultimo publish decimato
static uint32_t g_hdrf_spike_reject_cnt = 0;      // contatore diagnostico picchi scartati
static float  g_hdrf_last_raw_in_deg    = NAN;    // [dopo invert+offset] valore pre-Stage1 (per diagnosi UI)
static bool   g_hdrf_initialized         = false;
// Stato EMA VETTORIALE (2° stage smoothing) — tra Stage2 media vettoriale e Stage3 decimazione
float        g_hdrf_ema_s_sin = 0.0f;       // componente sin(θ) media pesata corrente
float        g_hdrf_ema_s_cos = 0.0f;       // componente cos(θ) media pesata corrente
bool         g_hdrf_ema_initialized = false;
static float  g_hdrf_ema_out_deg      = NAN;  // uscita EMA finale (per WS publish)

// ============================================================================
// [NUOVO] Sensor Filter Config · Variabili runtime (filtri vento + rain rate)
// ============================================================================
static SensorFilterConfig_t g_sfil_cfg;          // parametri caricati da EEPROM offset 3584
static bool  g_sfil_reset_ema_requested = false; // flag: reset stato EMA vento+rain al prossimo raw (applyRuntime)
// Extern stati locali vento/rain rate per applyRuntime (definiti in loop sezione sotto)
// Note: hall_smoothed_ms / s_rain_rate_ema sono var locali loop, reset interno usando g_sfil_reset_ema_requested

// ============================================================================
// CRASH LOGGER RTC USER MEMORY (words 48-127) - STUB per risparmio IRAM
// Funzioni originali rimosse: troppo codice in istruzioni IRAM Xtensa (32KB saturi).
// Possono essere riattivate in futuro se serve debug WDT/exception.
// ============================================================================
static void crash_initRtcHeartbeat(void) {}
static void crash_heartbeatPoll(void) {}
static void crash_capturePanicContext(const char*) {}
static void crash_spiffsStackWatermark(void) {}
static void ICACHE_FLASH_ATTR crash_printResetReport(void) {}

// ============================================================================
// SUPERVISORE + HARDWARE WATCHDOG (ARMATO 8 secondi)
// ============================================================================
static void supervisor_wdtFeed(void) {
    ESP.wdtFeed();
    // Aggiorna parola diagnostica contatore
    static uint32_t feedC = 0;
    g_diag_lastSensorWord = (g_diag_lastSensorWord & 0x0000FFFF) | ((feedC++ & 0xFFFF) << 16);
}

static bool supervisor_isStalled(void) {
    if (!WiFi.isConnected()) return false;
    if (!g_mqtt_was_connected) return false;
    uint32_t now = millis();
    bool connOk = g_mqtt.connected();
    bool pubRecent = (now - g_mqtt_lastGoodPubMs) < SUPV_PUB_STALL_MS;
    if (connOk && pubRecent) return false;
    if (!connOk) return true;
    return false;
}

static void supervisor_clearTrip(void) {
    g_supvTripped = false;
    g_supvState = SUPV_IDLE;
}

static void supervisor_init(void) {
    ESP.wdtEnable(SUPV_WDT_HW_MS);
    g_supvTripped = false;
    g_supvState = SUPV_IDLE;
    Serial.printf("[SUPV] HW Watchdog ARMATO %d ms · escalation L1→L4 (max %dms fino a HW Reset)\r\n",
        SUPV_WDT_HW_MS, SUPV_L4_RESET_MS);
}

static void supervisor_poll(void) {
    // ---- OTA IN CORSO: bypass completo escalation L1..L4 ----
    // Continua solo a nutrire il WDT HW per evitare reset durante scrittura flash.
    if (g_ota_in_progress) {
        ESP.wdtFeed();
        return;
    }
    supervisor_wdtFeed();
    uint32_t now = millis();

    if (!g_supvTripped) {
        if (!supervisor_isStalled()) { supervisor_wdtFeed(); return; }
        g_supvTripped = true;
        g_supvTripMs  = now;
        g_supvState   = SUPV_L1_TRIGGERED;
        Serial.println(F("[SUPV] STALL rilevato → L1 trigger"));
    }

    switch (g_supvState) {
        case SUPV_IDLE: break;
        case SUPV_L1_TRIGGERED: {
            if (WiFi.isConnected()) {
                g_supvL1Count++;
                // L1: bypass backoff (imposta backoff minimo)
                // Non c'è azione brusca; solo passaggio a stato successivo e countdown
            }
            g_supvLastActionMs = now;
            g_supvState = SUPV_L1_WAIT_COOLDOWN;
            Serial.println(F("[SUPV] L1 → attendi 3000ms per recovery automatica"));
            break;
        }
        case SUPV_L1_WAIT_COOLDOWN:
            supervisor_wdtFeed();
            if (!supervisor_isStalled()) {
                Serial.println(F("[SUPV] ✓ recovery in L1_COOLDOWN → CLEAR"));
                supervisor_clearTrip(); return;
            }
            if ((int32_t)(now - g_supvLastActionMs) >= SUPV_L1_COOLDOWN_MS) {
                g_supvState = SUPV_L2_TRIGGERED;
            }
            break;
        case SUPV_L2_TRIGGERED: {
            g_supvL2Count++;
            Serial.println(F("[SUPV] L2: disconnect MQTT client (forza riconnessione pulita)"));
            g_mqtt.disconnect();
            g_supvLastActionMs = now;
            g_supvState = SUPV_L2S_WAIT_COOLDOWN;
            break;
        }
        case SUPV_L2S_WAIT_COOLDOWN:
            supervisor_wdtFeed();
            if (!supervisor_isStalled()) {
                Serial.println(F("[SUPV] ✓ recovery in L2_COOLDOWN → CLEAR"));
                supervisor_clearTrip(); return;
            }
            if ((int32_t)(now - g_supvLastActionMs) >= SUPV_L2_COOLDOWN_MS) {
                g_supvState = SUPV_L3_TRIGGERED;
            }
            break;
        case SUPV_L3_TRIGGERED: {
            g_supvL3Count++;
            Serial.println(F("[SUPV] L3: WiFi.forceSleepBegin(1200ms) → OFF radio → riconnessione forzata"));
            WiFi.forceSleepBegin();
            delay(SUPV_L3_OFF_MS);
            WiFi.forceSleepWake();
            delay(SUPV_L3_POST_MS);
            // Richiedi riconnessione se ancora in STA mode
            if (WiFi.getMode() & WIFI_STA) {
                WiFi.reconnect();
            }
            g_supvLastActionMs = now;
            // Passa immediatamente a attesa L4
            g_supvState = SUPV_L4_DUE;
            break;
        }
        case SUPV_L4_DUE:
            supervisor_wdtFeed();
            if (!supervisor_isStalled()) {
                Serial.println(F("[SUPV] ✓ recovery in L4 attesa → CLEAR"));
                supervisor_clearTrip(); return;
            }
            if ((int32_t)(now - g_supvTripMs) >= SUPV_L4_RESET_MS) {
                g_supvResetCount++;
                Serial.println(F("[SUPV] L4: ESP.restart() · stall > 120 secondi"));
                crash_capturePanicContext("SUPV_L4_TIMEOUT");
                delay(20);
                ESP.restart();
            }
            break;
    }
}

// ============================================================================
// OTA · Supervisor Bypass + WDT Feed + Funzioni HTTP Upload
// ============================================================================
static void ota_bypassSupervisorBegin(void) {
    g_ota_supv_armed = g_supvTripped;
    g_ota_supv_state = g_supvState;
    g_supvTripped    = false;
    g_supvState      = SUPV_IDLE;
    g_ota_in_progress = true;
    g_ota_last_feed_ms = millis();
    Serial.println(F("[OTA] Supervisor bypassato · WDT manuale attivo"));
}

static void ota_bypassSupervisorEnd(void) {
    g_ota_in_progress = false;
    g_supvTripped     = g_ota_supv_armed;
    g_supvState       = g_ota_supv_state;
    Serial.println(F("[OTA] Supervisor stato ripristinato"));
}

static void ota_wdtFeedIfNeeded(bool force) {
    uint32_t now = millis();
    if (force || (int32_t)(now - g_ota_last_feed_ms) >= 2000) {
        ESP.wdtFeed();
        g_ota_last_feed_ms = now;
    }
}

static void ota_progress_cb(size_t progress, size_t total) {
    (void)total;
    static size_t lastFeedProg = 0;
    if ((progress - lastFeedProg) >= OTA_WDT_FEED_BYTES) {
        ota_wdtFeedIfNeeded(true);
        lastFeedProg = progress;
    }
}

// Forward declaration (definita dopo handleOtaUpload, ~linea 2913)
static size_t ota_getMaxSketchSizeBytes(void);

// ============================================================================
// OTA HTTP · HELPER PRINCIPALI
//   Strategia "BEGIN ON FIRST WRITE" per evitare il bug "Bad Size Given"
//   dovuto a doppia chiamata Update.begin():
//     · ota_http_begin()     → SALVA globals + MD5 MA NON chiama Update.begin()
//     · ota_http_writeChunk() → PRIMO chunk (len>0) chiama magic-detect su buf[0]:
//         - buf[0] == 0xE9 (bootloader in testa) → effectiveSize = total - 64KB, skip=64KB
//         - buf[0] != 0xE9 (sketch puro)          → effectiveSize = total, skip=0
//         → Update.begin(effectiveSize, U_FLASH) eseguito UNA SOLA VOLTA
//         → poi skip bytes + Update.write(resto chunk)
//   In questo modo non serve più annullare begin + ricrearlo (UpdateClass
//   ESP8266 core 3.x non resetta _size dopo .end(false) e fallisce con
//   "Bad Size Given" sul secondo .begin()).
// ============================================================================

static bool ota_http_begin(size_t total, const char* md5) {
    ota_bypassSupervisorBegin();
    g_ota_total   = total;
    g_ota_total_user = total;
    g_ota_written = 0;
    g_ota_skip_remaining = 0;
    g_ota_detected_full_fw = false;
    g_ota_begun = false;
    g_ota_last_err = 0;
    g_ota_md5_str = (md5 && *md5) ? String(md5) : String("");
    Update.onProgress(ota_progress_cb);
    Serial.printf("[OTA] ota_http_begin(): PREP begin-on-first-write · size dichiarato=%u MD5=%s\r\n",
        (unsigned)total, g_ota_md5_str.length() ? g_ota_md5_str.c_str() : "(non fornito)");
    return true;
}

static bool ota_http_writeChunk(const uint8_t* buf, size_t len) {
    if (!g_ota_in_progress || !buf || len == 0) return false;

    // ================================================================
    //  PRIMO chunk EVER: magic detect + Update.begin LAZY
    //  IMPORTANTE: La UpdateClass ESP8266 gestisce AUTONOMAMENTE
    //    - Il magic byte 0xE9 (firmware completo con bootloader in testa)
    //    - Skip dei primi 64KB (bootloader) → scrive direttamente sketch
    //    NON dobbiamo fare noi lo skip manuale, altrimenti Update.end()
    //    fallisce con "Magic byte is not 0xE9" perché non vede 0xE9 in testa.
    // ================================================================
    if (!g_ota_begun) {
        const bool isFullFirmware = (buf[0] == OTA_BOOTLOADER_MAGIC_BYTE);
        size_t effectiveSize;
        if (isFullFirmware) {
            // Safety: se totalSize è DICHIARATO e <= skip bytes → file tronco
            if (g_ota_total != 0 && g_ota_total <= OTA_BOOTLOADER_SKIP_BYTES) {
                Serial.printf("[OTA][ERR] firmware completo ma size=%u <= %u skip bytes (file tronco)\r\n",
                    (unsigned)g_ota_total, (unsigned)OTA_BOOTLOADER_SKIP_BYTES);
                g_ota_last_err = -999;
                ota_bypassSupervisorEnd();
                return false;
            }
            // effectiveSize = size TOTALE del file (inclusi 64KB bootloader).
            // La UpdateClass skippa AUTONOMAMENTE i primi 64KB quando vede magic 0xE9.
            if (g_ota_total != 0) {
                effectiveSize = g_ota_total;
            } else {
                // Transfer-Encoding chunked → size non nota.
                // Size sicura: sketch attuale + 256KB margine + 64KB bootloader.
                // Update.begin(0) non supportato → Bad Size 5.
                // Update.begin(max partition) → Not Enough Space code=4 per controllo errato Updater.
                size_t sketchNow = ESP.getSketchSize();
                size_t safeSize = sketchNow + (256UL * 1024UL) + OTA_BOOTLOADER_SKIP_BYTES;
                size_t maxPartition = ota_getMaxSketchSizeBytes() + OTA_BOOTLOADER_SKIP_BYTES;
                if (safeSize > maxPartition) safeSize = maxPartition;
                if (safeSize < (640UL * 1024UL)) safeSize = 640UL * 1024UL; // minimo 640KB
                effectiveSize = safeSize;
            }
            g_ota_total_user = effectiveSize;
            g_ota_detected_full_fw = true;
            Serial.printf("[OTA] PRIMO chunk: magic 0x%02X → FIRMWARE COMPLETO (skip %u B automatico da Updater) · total=%u B%s\r\n",
                (unsigned)buf[0],
                (unsigned)OTA_BOOTLOADER_SKIP_BYTES,
                (unsigned)effectiveSize,
                (g_ota_total == 0) ? " (sicura chunked)" : " (dichiarata)");
        } else {
            if (g_ota_total != 0) {
                effectiveSize = g_ota_total;
            } else {
                // Sketch puro chunked: size sicura
                size_t sketchNow = ESP.getSketchSize();
                size_t safeSize = sketchNow + (256UL * 1024UL);
                size_t maxPartition = ota_getMaxSketchSizeBytes();
                if (safeSize > maxPartition) safeSize = maxPartition;
                if (safeSize < (512UL * 1024UL)) safeSize = 512UL * 1024UL;
                effectiveSize = safeSize;
            }
            g_ota_total_user = effectiveSize;
            g_ota_detected_full_fw = false;
            Serial.printf("[OTA] PRIMO chunk: magic 0x%02X → SKETCH PURO · size=%u B%s\r\n",
                (unsigned)buf[0],
                (unsigned)effectiveSize,
                (g_ota_total == 0) ? " (sicura chunked)" : " (dichiarata)");
        }
        // effectiveSize < 4096 → errore sicuro
        if (effectiveSize < 4096UL) {
            Serial.printf("[OTA][ERR] effective size %u B < 4096 minimo\r\n", (unsigned)effectiveSize);
            g_ota_last_err = -999;
            ota_bypassSupervisorEnd();
            return false;
        }
        Update.setMD5(g_ota_md5_str.c_str());
        Update.onProgress(ota_progress_cb);
        bool okBegin = Update.begin(effectiveSize, U_FLASH);
        if (!okBegin) {
            g_ota_last_err = (int)Update.getError();
            Serial.printf("[OTA] Update.begin(%u, U_FLASH) FALLITA: err=%d (%s)\r\n",
                (unsigned)effectiveSize, g_ota_last_err, Update.getErrorString().c_str());
            ota_bypassSupervisorEnd();
            return false;
        }
        g_ota_begun = true;
        Serial.printf("[OTA] Update.begin(%u) OK · md5=%s\r\n",
            (unsigned)effectiveSize,
            g_ota_md5_str.length() ? g_ota_md5_str.c_str() : "(nessuno)");
    }

    // ================================================================
    //  SCRITTURA TUTTI I BYTE (senza skip! Updater skippa da solo 64KB)
    // ================================================================
    size_t written = Update.write(const_cast<uint8_t*>(buf), len);
    g_ota_written += written;
    ota_wdtFeedIfNeeded(false);
    if (written != len) {
        g_ota_last_err = (int)Update.getError();
        Serial.printf("[OTA] Update.write() fallita chunk: scritti %u / %u · err=%d (%s)\r\n",
            (unsigned)written, (unsigned)len, g_ota_last_err, Update.getErrorString().c_str());
        return false;
    }
    return true;
}

static bool ota_http_end(void) {
    ota_wdtFeedIfNeeded(true);
    // Safety: begin non è stato neppure chiamato → file vuoto / nessun chunk ricevuto
    if (!g_ota_begun) {
        Serial.printf("[OTA][ERR] fine upload ma Update.begin() mai chiamato (nessun chunk?)\r\n");
        g_ota_last_err = -997;
        ota_bypassSupervisorEnd();
        return false;
    }
    // Safety: scritto meno di quanto atteso (solo se size era dichiarata esplicitamente)
    if (g_ota_total != 0 && g_ota_written != g_ota_total_user) {
        Serial.printf("[OTA][WARN] scritti %u B / %u B attesi (tenta comunque end)\r\n",
            (unsigned)g_ota_written, (unsigned)g_ota_total_user);
    }
    bool ok = Update.end(true);
    g_ota_last_err = (int)Update.getError();
    g_ota_begun = false;
    if (!ok) {
        Serial.printf("[OTA] Update.end() FALLITA: err=%d (%s)\r\n",
            g_ota_last_err, Update.getErrorString().c_str());
    } else {
        // Calcola quanto è stato scritto VERAMENTE nello sketch (skip 64KB se full fw)
        size_t sketchWritten = g_ota_detected_full_fw
            ? ((g_ota_written > OTA_BOOTLOADER_SKIP_BYTES) ? (g_ota_written - OTA_BOOTLOADER_SKIP_BYTES) : 0)
            : g_ota_written;
        Serial.printf("[OTA] Update.end() OK · scritti %u/%u B (sketch=%u B) · skip automatico bootloader=%s\r\n",
            (unsigned)g_ota_written, (unsigned)g_ota_total_user,
            (unsigned)sketchWritten,
            g_ota_detected_full_fw ? "SI (firmware completo)" : "NO (sketch puro)");
    }
    ota_bypassSupervisorEnd();
    return ok;
}

static void ota_http_abort(void) {
    Serial.println(F("[OTA] Upload abortito / fallito prima della fine"));
    if (g_ota_begun) Update.end(false);
    g_ota_begun = false;
    g_ota_last_err = (int)Update.getError();
    ota_bypassSupervisorEnd();
}

static size_t ota_getMaxSketchSizeBytes(void) {
    extern uint8_t _FS_start;
    const uint32_t sketchFlashBase = 0x40200000UL;
    const uint32_t fsStart = (uint32_t)&_FS_start;
    const size_t sketchCurrent = ESP.getSketchSize();
    const size_t sketchFreeAfter = ESP.getFreeSketchSpace();
    const size_t sketchTotal = sketchCurrent + sketchFreeAfter;
    Serial.printf("[OTA] Sketch corrente=%u B  Free dopo=%u B  Tot (partition)=%u B\r\n",
        (unsigned)sketchCurrent, (unsigned)sketchFreeAfter, (unsigned)sketchTotal);
    if (fsStart > sketchFlashBase + sketchTotal) {
        const size_t fromFs = (size_t)((fsStart - sketchFlashBase) & 0xFFFFF000UL);
        Serial.printf("[OTA] Calcolo alternativo _FS_start-base=%u B, uso max()\r\n", (unsigned)fromFs);
        return (sketchTotal > fromFs) ? sketchTotal : fromFs;
    }
    const size_t maxAligned = (size_t)(sketchTotal & 0xFFFFF000UL);
    return maxAligned > 0x10000UL ? maxAligned : 0x10000UL;
}

// ---- Form minimale HTML PROGMEM GLOBALE (PROGMEM non valido in locale a funzione) ----
static const char OTA_MINIMAL_HTML[] PROGMEM =
R"rawliteral(<!doctype html><html lang="it"><head><meta charset="utf-8">
<title>OTA Firmware</title></head><body style="background:#111;color:#eee;font-family:system-ui;max-width:720px;margin:30px auto">
<h2 style="color:#fff">📤 Aggiornamento Firmware OTA</h2>
<form method="POST" action="/update" enctype="multipart/form-data">
  <div style="margin:16px 0">
    <label style="display:block;margin-bottom:6px;color:#90caf9">Seleziona file firmware (.bin):</label>
    <input type="file" name="firmware" accept=".bin,.bin.gz" required style="background:#1b1e25;border:1px solid #343a44;color:#eee;padding:10px;border-radius:6px;width:100%"/>
  </div>
  <div style="display:flex;gap:10px">
    <button type="submit" style="background:#1e3a8a;color:#fff;border:1px solid #2a4caf;padding:10px 18px;border-radius:6px;font-weight:600">💾 Flash Firmware</button>
    <a href="/" style="background:#1a1e24;color:#eee;border:1px solid #343a44;padding:10px 18px;border-radius:6px;text-decoration:none">⟵ Torna alla Dashboard</a>
  </div>
</form>
<p style="color:#7b828c;margin-top:26px;font-size:13px">
⚠ L'operazione impiega 10-30 secondi. Non spegnere durante la scrittura.<br>
Al termine l'ESP8266 si riavvierà automaticamente.</p>
<details style="margin-top:18px;padding:12px;background:#1a1e24;border:1px solid #2a2f38;border-radius:8px">
<summary style="color:#90caf9;cursor:pointer;font-weight:600">💡 Dove trovare il file firmware .bin?</summary>
<div style="margin-top:10px;color:#9aa3ad;font-size:13px;line-height:1.6">
Il file binario viene generato automaticamente dalla build PlatformIO nella cartella del progetto:<br><br>
<code style="background:#111;padding:4px 8px;border-radius:4px;color:#64d8a4">firmware_output/firmware_latest.bin</code><br><br>
Viene anche salvata una copia con timestamp per storico versioni (es. <code style="color:#64d8a4">firmware_<em>NOME</em>_20260923_183000.bin</code>).<br>
Caricare uno di questi file qui sopra per l'aggiornamento OTA.
</div>
</details>
</body></html>)rawliteral";

// ---- Form minimale HTML (se si naviga direttamente a /update) ----
static void handleOtaGet(void) {
    g_http.sendHeader("Cache-Control", "no-store");
    g_http.send_P(200, PSTR("text/html; charset=utf-8"), OTA_MINIMAL_HTML);
}

// ---- Handler HTTP: upload multipart chunk-by-chunk ----
static void rtc_bootloaderResetHard(void);   // forward declare
static void handleDebugRtcReset(void);       // forward declare
static void handleOtaUpload(void) {
    HTTPUpload& up = g_http.upload();
    switch (up.status) {
        case UPLOAD_FILE_START: {
            // OTA ROLLBACK ESCAPE: PRIMA di qualsiasi Update.begin() o MD5/size check
            // → cancella RTC mem words 0..31 (contatori exception bootloader).
            // La bank opposta è sempre "vergine" dal punto di vista dei failure counts.
            rtc_bootloaderResetHard();
            Serial.printf("[OTA] Upload START: name=%s totalSize=%u\r\n",
                up.filename.c_str(), (unsigned)up.totalSize);
            const char* md5 = g_http.hasHeader("X-MD5")
                ? g_http.header("X-MD5").c_str()
                : nullptr;
            size_t maxSketch = ota_getMaxSketchSizeBytes();
            size_t size;
            if (up.totalSize != 0) {
                size = up.totalSize;
                // Safety extra: se file completo (presunto) deve stare: max(0, totalSize - 64KB) <= maxSketch
                // (il check rigoroso lo facciamo poi nella write quando sappiamo se c'è bootloader)
                if (size > maxSketch + OTA_BOOTLOADER_SKIP_BYTES + 4096UL) {
                    Serial.printf("[OTA][ERR] file too large (include bootloader?): %u > max %u B (sketch=%u B)\r\n",
                        (unsigned)size, (unsigned)(maxSketch + OTA_BOOTLOADER_SKIP_BYTES), (unsigned)maxSketch);
                    g_http.send(413, "application/json",
                        "{\"ok\":false,\"error\":\"firmware too large for sketch partition (considera bootloader 64KB)\","
                        "\"file_size\":" + String((unsigned long)size) +
                        ",\"max_size\":" + String((unsigned long)(maxSketch + OTA_BOOTLOADER_SKIP_BYTES)) + "}");
                    return;
                }
                Serial.printf("[OTA] totalSize fornito: usa size=%u B · max sketch=%u B\r\n",
                    (unsigned)size, (unsigned)maxSketch);
            } else {
                Serial.println(F("[OTA] totalSize=0 (multipart sconosciuto): passa 0 → Updater calcola max da _FS_start/_startAddress"));
                size = 0;
            }
            if (!ota_http_begin(size, md5)) {
                g_http.send(500, "application/json",
                    "{\"ok\":false,\"error\":\"Update.begin() failed\",\"code\":" +
                    String(g_ota_last_err) + ",\"msg\":\"" + Update.getErrorString() + "\"}");
            }
            break;
        }
        case UPLOAD_FILE_WRITE: {
            if (!g_ota_in_progress) return;
            if (!ota_http_writeChunk(up.buf, up.currentSize)) {
                g_http.send(500, "application/json",
                    "{\"ok\":false,\"error\":\"write chunk failed\",\"written\":" +
                    String((unsigned long)g_ota_written) + ",\"code\":" +
                    String(g_ota_last_err) + ",\"msg\":\"" + Update.getErrorString() + "\"}");
            }
            break;
        }
        case UPLOAD_FILE_END: {
            if (!g_ota_in_progress) return;
            const size_t fileTotalWritten = g_ota_written;
            const size_t sketchOnlySize = g_ota_detected_full_fw
                ? ((fileTotalWritten > OTA_BOOTLOADER_SKIP_BYTES) ? (fileTotalWritten - OTA_BOOTLOADER_SKIP_BYTES) : 0)
                : fileTotalWritten;
            Serial.printf("[OTA] Upload END: received=%u bytes  written(total file)=%u bytes  sketch_only=%u bytes  declared_total=%u  bootloader_in_file=%s\r\n",
                (unsigned)up.totalSize,
                (unsigned)fileTotalWritten,
                (unsigned)sketchOnlySize,
                (unsigned)g_ota_total_user,
                g_ota_detected_full_fw ? "SI" : "NO");
            if (g_ota_total != 0 && fileTotalWritten != g_ota_total_user) {
                Serial.printf("[OTA][WARN] written mismatch: %u != expected total %u (tenta comunque end)\r\n",
                    (unsigned)fileTotalWritten, (unsigned)g_ota_total_user);
            }
            bool ok = ota_http_end();
            if (!ok) {
                String body = "{\"ok\":false,\"error\":\"Update.end() failed\",\"code\":";
                body += String(g_ota_last_err);
                body += ",\"msg\":\"";
                body += Update.getErrorString();
                body += "\"}";
                g_http.send(500, "application/json", body);
            } else {
                Serial.printf("[OTA][OK] Update completato · file=%u B sketch=%u B (bootloader auto=%s) · invio risposta + reboot\r\n",
                    (unsigned)fileTotalWritten,
                    (unsigned)sketchOnlySize,
                    g_ota_detected_full_fw ? "SI" : "NO");
                String body = "{\"ok\":true,\"size\":";
                body += String((unsigned long)fileTotalWritten);
                body += ",\"sketch_only_size\":";
                body += String((unsigned long)sketchOnlySize);
                body += ",\"bootloader_skipped\":";
                body += g_ota_detected_full_fw ? "true" : "false";
                body += ",\"reboot_ms\":";
                body += String(OTA_POST_SUCCESS_REBOOT_MS);
                body += "}";
                g_http.send(200, "application/json", body);
                ota_wdtFeedIfNeeded(true);
                yield();
                delay(350);
                yield();
                ota_wdtFeedIfNeeded(true);
                // DOUBLE-TAP anti-rollback: (1) pulisci TUTTA la RTC non volatile + eraseConfig
                rtc_bootloaderResetHard();
                delay(15);
                // (2) Imposta FINISH per dire a bootloader che l'OTA è valido
                system_upgrade_flag_set(UPGRADE_FLAG_FINISH);
                delay(15);
                // (3) Reboot NONOS SDK OTA-aware
                system_upgrade_reboot();
                while (true) { yield(); delay(1); }
            }
            break;
        }
        case UPLOAD_FILE_ABORTED: {
            ota_http_abort();
            break;
        }
    }
}

static void handleOtaPostFinish(void) {
    // Se l'upload OTA è andato a buon fine o ha già avuto un errore, abbiamo già inviato risposta
    // (non abbiamo accesso a g_http._chunked perché protected).
    // Quindi usiamo un flag: se g_ota_in_progress è rimasto true vuol dire che non è partito nulla.
    if (g_ota_in_progress) return;
    if (!g_http.hasArg("file_idx")) {
        String body = "{\"ok\":false,\"error\":\"nessun file nel POST multipart\"}";
        g_http.send(400, "application/json", body);
    }
}

// ============================================================================
// OTA ROLLBACK ESCAPE: resetta RTC memory zone bootloader (words 0..31) +
// eraseConfig(). Chiamato AUTOMATICAMENTE all'inizio di handleOtaUpload
// (PRIMA di Update.begin()) per cancellare qualsiasi contatore exception
// preesistente nella bank opposta. Evita rollback infiniti dopo 3 crash.
// Inoltre endpoint segreto POST /debug/rtc_reset per triggerarlo manualmente
// se ti trovi già in loop rollback e la LIVE è la bank vecchia.
// ============================================================================
static void ICACHE_FLASH_ATTR rtc_bootloaderResetHard(void) {
    const uint32_t ZERO = 0x00000000UL;
    for (int w = 0; w <= 63; w++) {
        system_rtc_mem_write(w, (void*)&ZERO, 4);
    }
    ESP.eraseConfig();
    delay(20);
    ESP.wdtFeed();
}
static void ICACHE_FLASH_ATTR handleDebugRtcReset(void) {
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    rtc_bootloaderResetHard();
    g_http.send(200, "application/json",
        "{\"ok\":true,\"msg\":\"RTC mem words 0..63 erased + ESP.eraseConfig() OK\",\"next\":\"puoi flashare OTA senza rollback\"}");
    delay(250);
    ESP.restart();
}

// ============================================================================
// WIFI ROBUSTO: 11 tentativi + hard reset + fallback poll ogni 30s
// ============================================================================
static bool netTrySTAOnce(const char* ssid, const char* pass, uint32_t timeout) {
    if (WiFi.status() == WL_CONNECTED) { WiFi.disconnect(true); delay(100); }
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, pass);
    uint32_t start = millis();
    while ((int32_t)(millis() - start) < (int32_t)timeout) {
        supervisor_wdtFeed();
        wl_status_t s = WiFi.status();
        if (s == WL_CONNECTED) return true;
        if (s == WL_CONNECT_FAILED) return false;
        delay(200);
    }
    return (WiFi.status() == WL_CONNECTED);
}

static bool netStartSTAWithRetries(const char* ssid, const char* pass) {
    Serial.printf("[NET] Connessione STA a '%s' · %d tentativi · timeout/tentativo %d ms\r\n",
        ssid, WIFI_STA_RETRY_COUNT + 1, WIFI_STA_PER_TRY_TIMEOUT_MS);
    for (int t = 0; t <= WIFI_STA_RETRY_COUNT; t++) {
        Serial.printf("[NET]  tentativo %d/%d …", t + 1, WIFI_STA_RETRY_COUNT + 1);
        bool ok = netTrySTAOnce(ssid, pass, WIFI_STA_PER_TRY_TIMEOUT_MS);
        if (ok) {
            Serial.printf(" OK  IP=%s  RSSI=%d dBm\r\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
            g_net_fallback_mode = false;
            return true;
        }
        Serial.print(F(" FALLITO"));
        if (t < WIFI_STA_RETRY_COUNT) {
            Serial.printf(" · sleep radio %d ms + delay %d ms\r\n",
                WIFI_STA_HARD_RESET_OFF_MS, WIFI_STA_INTER_RETRY_DELAY_MS);
            WiFi.forceSleepBegin();
            delay(WIFI_STA_HARD_RESET_OFF_MS);
            WiFi.forceSleepWake();
            delay(WIFI_STA_INTER_RETRY_DELAY_MS);
        } else {
            Serial.println();
        }
    }
    return false;
}

static void netFallbackPollRetry(void) {
    if (WiFi.status() == WL_CONNECTED) {
        g_net_fallback_mode = false;
        return;
    }
    uint32_t now = millis();
    if ((int32_t)(now - g_net_fallbackLastPollMs) < WIFI_FALLBACK_POLL_PERIOD_MS) return;
    g_net_fallbackLastPollMs = now;
    if (!g_net_fallback_mode) {
        Serial.printf("[NET][FALLBACK] STA non connessa → nuovo tentativo (poll ogni %d ms)\r\n",
            WIFI_FALLBACK_POLL_PERIOD_MS);
        g_net_fallback_mode = true;
    }
    // 1 singolo tentativo soft (senza radio power-cycle per non disturbare AP)
    WiFi.setAutoReconnect(true);
    if (WiFi.getMode() & WIFI_STA) WiFi.reconnect();
}

static void netStartSTAorAP(void) {
    bool staOk = netStartSTAWithRetries(g_config.wifi_ssid,
        (strlen(g_config.wifi_password) > 0) ? g_config.wifi_password : nullptr);
    if (!staOk) {
        Serial.print(F("[NET] STA fallita tutti i tentativi → avvio Access Point '"));
        Serial.print(g_config.ap_ssid); Serial.println('\'');
        WiFi.mode(WIFI_AP);
        bool apOk;
        if (strlen(g_config.ap_password) >= 8) {
            apOk = WiFi.softAP(g_config.ap_ssid, g_config.ap_password);
        } else {
            apOk = WiFi.softAP(g_config.ap_ssid);
        }
        if (apOk) {
            Serial.printf("[NET] AP OK  IP=%s  GW=%s\r\n",
                WiFi.softAPIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
        } else {
            Serial.println(F("[NET] ERRORE softAP start"));
        }
    }
}

// ============================================================================
// MQTT ROBUSTO: TCP keepalive, 4 sanity-check zombie, jitter backoff
// ============================================================================
static void mqtt_setTcpKeepAlive(void) {
}

static bool mqtt_socketSanityChecks(void) {
    return g_mqtt.connected();
}

// --- Forward declarations MQTT helpers (definiti più avanti, ~L3106 e L3241) ---
static void mqttGetBaseTopic(char* buf, size_t bufLen);
static void mqttGetAvailTopic(char* buf, size_t bufLen);
static void mqttPublishAllDiscovery(void);

static void ICACHE_FLASH_ATTR mqtt_ensureConnected(void) {
    if (WiFi.status() != WL_CONNECTED) return;
    const bool currentlyConnected = g_mqtt.connected();

    if (currentlyConnected) {
        g_mqtt_was_connected = true;
        if (!mqtt_socketSanityChecks()) {
            Serial.println(F("[MQTT] zombie rilevato (socket sanity) → disconnect forzata"));
            g_mqtt.disconnect();
            g_mqtt_pub_consec_fail = 0;
            g_mqtt_consec_failures++;
            delay(20);
            return;
        }
        if (g_mqtt_consec_failures > 0) g_mqtt_consec_failures = 0;
        return;
    }
    // Transizione CONN → DISCONN = backoff 0
    static uint32_t s_lastTryMs = 0;
    static uint32_t s_backoffMs = MQTT_CONNECT_BACKOFF_MIN_MS;
    uint32_t now = millis();
    if (g_mqtt_was_connected) {
        s_lastTryMs = now - 0x7FFFFFFF;     // forza tentativo subito
        s_backoffMs = MQTT_CONNECT_BACKOFF_MIN_MS;
        g_mqtt_was_connected = false;
    } else {
        if ((int32_t)(now - s_lastTryMs) < (int32_t)s_backoffMs) return;
    }
    s_lastTryMs = now;
    uint32_t jitter = ((uint32_t)os_random() % 100u);
    s_backoffMs = (s_backoffMs + 300 + jitter < (uint32_t)MQTT_CONNECT_BACKOFF_MAX_MS)
                  ? (s_backoffMs + 300 + jitter) : (uint32_t)MQTT_CONNECT_BACKOFF_MAX_MS;

    Serial.printf("[MQTT] Connect %s:%u …", g_config.mqtt_server, g_config.mqtt_port);
    if (!g_mqtt.setBufferSize(MQTT_CLIENT_BUFFER_SIZE)) {
        Serial.print(F(" buf-fail"));
    }
    g_mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
    g_mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
    IPAddress ip;
    char willTopic[160];
    snprintf_P(willTopic, sizeof(willTopic), PSTR("homeassistant/sensor/%s/availability"),
        g_config.mqtt_device_id);
    bool ok = false;
    if (ip.fromString(g_config.mqtt_server)) {
        g_mqtt.setServer(ip, g_config.mqtt_port);
        ok = g_mqtt.connect(g_config.mqtt_device_id,
            strlen(g_config.mqtt_user) > 0 ? g_config.mqtt_user : nullptr,
            strlen(g_config.mqtt_password) > 0 ? g_config.mqtt_password : nullptr,
            willTopic, 1, true, "offline", true);
    } else {
        g_mqtt.setServer(g_config.mqtt_server, g_config.mqtt_port);
        ok = g_mqtt.connect(g_config.mqtt_device_id,
            strlen(g_config.mqtt_user) > 0 ? g_config.mqtt_user : nullptr,
            strlen(g_config.mqtt_password) > 0 ? g_config.mqtt_password : nullptr,
            willTopic, 1, true, "offline", true);
    }
    if (!ok) {
        g_mqtt_consec_failures++;
        Serial.printf(" FALLITO rc=%u consec=%u backoff=%lu ms\r\n",
            (unsigned)g_mqtt.state(), (unsigned)g_mqtt_consec_failures,
            (unsigned long)s_backoffMs);
        return;
    }
    mqtt_setTcpKeepAlive();
    g_mqtt_lastGoodConnMs = now;
    g_mqtt_pub_consec_fail = 0;
    g_mqttReconnectCount++;  // M7: incrementa contatore riconnessioni (totale, boot incluso = total_increasing ok)
    Serial.printf(" OK  keepAlive=%us sockT=%us buf=%uB  (connessioni totali=%lu)\r\n",
        MQTT_KEEPALIVE_S, MQTT_SOCKET_TIMEOUT_S, MQTT_CLIENT_BUFFER_SIZE,
        (unsigned long)g_mqttReconnectCount);
    // --- LWT complementare: pubblica online + Home Assistant Discovery
    {
        char avail[128]; mqttGetAvailTopic(avail, sizeof(avail));
        g_mqtt.publish(avail, "online", true);
    }
    // --- Callback + subscribe topic comandi (es. reset pioggia evento da HA)
    g_mqtt.setCallback(mqttOnMessage);
    {
        char cmdTopic[160];
        snprintf_P(cmdTopic, sizeof(cmdTopic), PSTR("homeassistant/sensor/%s/cmd/rain_reset"), g_config.mqtt_device_id);
        g_mqtt.subscribe(cmdTopic);
        Serial.printf_P(PSTR("[MQTT] Subscribed CMD: %s\n"), cmdTopic);
    }
    if (!g_mqtt_discovery_sent) {
        mqttPublishAllDiscovery();
        g_mqtt_discovery_sent = true;
        Serial.println(F("[MQTT] HA Discovery pubblicati (~24 sensori dopo cleanup MQTT 2026-03)"));
    }
}

// ============================================================================
// Hall A3144 - stato condiviso con ISR
// ============================================================================
volatile uint32_t g_hall_last_us    = 0;
volatile uint32_t g_hall_period_us  = 0;
volatile bool     g_hall_new_period = false;
volatile uint32_t g_hall_count      = 0;

static void IRAM_ATTR hall_isr()
{
    uint32_t t = micros();
    if (g_hall_last_us != 0 && t > g_hall_last_us) {
        uint32_t delta = t - g_hall_last_us;
        if (delta >= HALL_MIN_PERIOD_US) {
            g_hall_period_us = delta;
            g_hall_new_period = true;
        }
    }
    g_hall_last_us = t;
    g_hall_count++;
}

// ============================================================================
// Rain Gauge Bascula - stato condiviso con ISR
//   Dead-time da EEPROM (g_config.rain_dead_ms) è cachato da loop() e usato
//   per confronto, ma ISR usa un valore minimo hardcoded per sicurezza critica.
// ============================================================================
volatile uint32_t g_rain_last_ms    = 0;
volatile uint32_t g_rain_tick_count = 0;
volatile bool     g_rain_new_tick   = false;

static void IRAM_ATTR rain_isr()
{
    uint32_t t = millis();
    uint32_t dt = g_rain_isr_dead_ms;
    if (dt < RAIN_DEAD_MS_DEF) dt = RAIN_DEAD_MS_DEF;
    if (g_rain_last_ms == 0 || (t - g_rain_last_ms) >= dt) {
        g_rain_last_ms  = t;
        g_rain_tick_count++;
        g_rain_new_tick = true;
    }
}

// ============================================================================
// Helper C per calcoli derivati lato firmware
// ============================================================================
static float cToF(float c) { return c * 9.0f / 5.0f + 32.0f; }
static float cToK(float c) { return c + 273.15f; }

static const char ROSE16[16][4] PROGMEM = {
    "N","NNE","NE","ENE","E","ESE","SE","SSE",
    "S","SSW","SW","WSW","W","WNW","NW","NNW"
};

static const char* rose16Of(float deg)
{
    static char buf[4];
    if (!isfinite(deg)) { buf[0] = 0; return buf; }
    const float step = 360.0f / 16.0f;
    const float h = step / 2.0f;
    float a = fmodf(deg, 360.0f);
    if (a < 0) a += 360.0f;
    a = fmodf(a + h, 360.0f);
    uint8_t i = (uint8_t)floorf(a / step);
    if (i >= 16) i = 15;
    strcpy_P(buf, ROSE16[i]);
    return buf;
}

static uint8_t beaufortOf(float ms)
{
    static const float PROGMEM limits[] = {
        0.2f, 1.5f, 3.3f, 5.4f, 7.9f, 10.7f, 13.8f, 17.1f,
        20.7f, 24.4f, 28.4f, 32.6f
    };
    if (!isfinite(ms) || ms < 0) return 0;
    for (uint8_t i = 0; i < 12; i++) {
        if (ms < pgm_read_float(&limits[i])) return i;
    }
    return 12;
}

static float dewPointC(float tc, float rh)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    const float a = 17.27f, b = 237.7f;
    if (rh < 0.01f) rh = 0.01f;
    if (rh > 100.0f) rh = 100.0f;
    float alpha = a * tc / (b + tc) + logf(rh / 100.0f);
    return (b * alpha) / (a - alpha);
}

static float absHumidityGm3(float tc, float rh)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    float T = cToK(tc);
    float es = 6.112f * expf(17.67f * tc / (tc + 243.5f));
    float ea = es * (rh / 100.0f);
    float rho_v = 0.21667f * (ea * 100.0f) / T;
    return rho_v * 1000.0f;
}

static float heatIndexC(float tc, float rh)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    float T = cToF(tc);
    float R = rh;
    if (R < 0) R = 0;
    if (R > 100) R = 100;
    float HI = 0.5f * (T + 61.0f + ((T - 68.0f) * 1.2f) + (R * 0.094f));
    if (HI >= 80.0f) {
        HI = -42.379f + 2.04901523f * T + 10.14333127f * R
            - 0.22475541f * T * R
            - 6.83783e-3f * T * T
            - 5.481717e-2f * R * R
            + 1.22874e-3f * T * T * R
            + 8.5282e-4f * T * R * R
            - 1.99e-6f * T * T * R * R;
    }
    return (HI - 32.0f) * 5.0f / 9.0f;
}

static float windChillC(float tc, float vKmh)
{
    if (!isfinite(tc) || !isfinite(vKmh)) return NAN;
    if (tc > 10.0f || vKmh < 4.8f) return NAN;
    float v16 = powf(vKmh, 0.16f);
    return 13.12f + 0.6215f * tc - 11.37f * v16 + 0.3965f * tc * v16;
}

static float apparentTempC(float tc, float vKmh, float rh)
{
    if (!isfinite(tc)) return NAN;
    float wc = windChillC(tc, vKmh);
    if (isfinite(wc)) return wc;
    float hi = heatIndexC(tc, rh);
    if (isfinite(hi) && tc >= 20.0f && isfinite(rh) && rh >= 40.0f) return hi;
    return tc;
}

static float humidexOf(float tc, float rh)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    float es = 6.112f * expf(17.67f * tc / (tc + 243.5f));
    float e  = es * (rh / 100.0f);
    return tc + 5.0f / 9.0f * (e - 10.0f);
}

static float frostPointC(float tc, float rh)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    const float a = 21.8745584f, b = 265.5f;
    if (rh < 0.01f) rh = 0.01f;
    if (rh > 100.0f) rh = 100.0f;
    float tf = (tc < 0.0f) ? tc : 0.0f;
    float alpha = a * tf / (b + tf) + logf(rh / 100.0f);
    return (b * alpha) / (a - alpha);
}

static float wetBulbStullC(float tc, float rh, float p_hpa)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    float rhClamped = rh;
    if (rhClamped < 0.0f)  rhClamped = 0.0f;
    if (rhClamped > 100.0f) rhClamped = 100.0f;
    if (!isfinite(p_hpa) || p_hpa < 100.0f || p_hpa > 2000.0f) p_hpa = 1013.25f;
    float RHp = rhClamped * (p_hpa / 1013.25f);
    if (RHp < 0.0f) RHp = 0.0f;
    if (RHp > 100.0f) RHp = 100.0f;
    float T  = tc;
    float Rs = sqrtf(RHp + 8.313659f);
    float at1 = atanf(0.151977f * Rs);
    float at2 = atanf(T + RHp);
    float at3 = atanf(RHp - 1.676331f);
    float RH32 = powf(RHp, 1.5f);
    float at4 = atanf(0.023101f * RHp) * RH32 * 0.00391838f;
    float WB = T * at1 + at2 - at3 + at4 - 4.686035f;
    if (WB > T) WB = T;
    return WB;
}

static float vaporPressureDeficitKPa(float tc, float rh)
{
    if (!isfinite(tc) || !isfinite(rh)) return NAN;
    if (rh < 0.01f) rh = 0.01f;
    if (rh > 100.0f) rh = 100.0f;
    float es_hpa = 6.112f * expf(17.67f * tc / (tc + 243.5f));
    float ea_hpa = es_hpa * (rh / 100.0f);
    return (es_hpa - ea_hpa) / 10.0f;
}

static float pressureAltitudeM(float hPa)
{
    if (!isfinite(hPa) || hPa <= 0) return NAN;
    float p0 = g_config.bme280_sealevel_hpa;
    if (p0 < 500.0f || p0 > 1500.0f) p0 = DEFAULT_BME_SEALEVEL_HPA;
    return 44330.0f * (1.0f - powf(hPa / p0, 1.0f / 5.255f));
}

// ============================================================================
// Helper Rain Gauge Bascula
//   mm_per_tick = ((massa_g / 1g/cm³) / (π·(diametro_mm/20)²)) · 10 · K
// ============================================================================
static float rainCalcMmPerTick(float dia_mm, float mass_g, float K)
{
    if (dia_mm < 1.0f) dia_mm = RAIN_FUNNEL_DIA_MM_DEF;
    if (mass_g < 0.01f) mass_g = RAIN_TIP_MASS_G_DEF;
    if (K < 0.01f) K = 1.0f;
    float r_cm = dia_mm / 20.0f;
    float v_cm3 = mass_g / 1.0f;   // ρ_acqua = 1 g/cm³
    return (v_cm3 / (PI * r_cm * r_cm)) * 10.0f * K;
}

static const char* rainClassWmoStr(float mm_h)
{
    if (!isfinite(mm_h) || mm_h < 0.05f) return "Assente";
    if (mm_h < 2.0f)  return "Leggera";
    if (mm_h <= 8.0f) return "Moderata";
    return "Forte";
}

// ============================================================================
// RainPersist_t v3 [RINOMINATO RAI3 · backward compat v2 RAI2 / v1 RAIN]:
//
// PROBLEMA v2 (risolto in v3):
//   v2 salvava solo offset. Dopo HW reboot actual=0 → post-boot align doveva
//   portare offset <= actual (0). Ma in questo modo:
//     · Prima reboot: actual=10, tot_off=5 → totale 5 tick (persistito 5)
//     · Dopo reboot actual=0, tot_off salvato=5 → align porta tot_off a 0
//     · Totale = 0 - 0 = 0 ❌ VALORE CUMULATIVO PERSO!
//
// SOLUZIONE v3: salviamo il VALORE CUMULATIVO FINALE (cum_saved = actual - tot_off)
//   e, in post-boot, calcoliamo il nuovo total_offset = actual_boot - cum_saved.
//   · actual_boot = 0 → tot_off = (0 - cum_saved) = -cum_saved · NON POSITIVO (saturiamo 0)
//     → MA noi VEDIAMO actual crescere col tempo: DOPO che actual >= cum_saved,
//       il totale torna cum_saved (corretto!).
//     → Per evitare attesa, in post-boot forziamo RAM:
//         if actual_boot < cum_saved → sovrascrivi g_rain_tick_count = cum_saved
//         (ISR continuerà ad incrementare correttamente)
//   Così la persistenza è perfetta anche con offset actual=0.
// ============================================================================
typedef struct {
    uint32_t magic;
    uint32_t total_offset_ticks;   // v2: offset TOTALE (v3 mantenuto per forward/backward safety)
    uint32_t today_offset_ticks;   // v2: offset OGGI  (v3 invariato per reset oggi mezzanotte)
    uint32_t today_day_epoch;      // codice giorno (v3 invariato)
    uint32_t cum_total_saved;      // v3 NUOVO: valore cumulativo TOTALE (actual - tot_off) PRIMA del reboot
    uint32_t crc32;
} RainPersist_t;

#define RAIN_PERSIST_MAGIC_V3  0x52414933UL   // 'RAI3'
#define RAIN_PERSIST_MAGIC_V2  0x52414932UL   // 'RAI2' (v2 backward compat)
#define RAIN_PERSIST_MAGIC     0x5241494EUL   // 'RAIN' (v1 backward compat)
#define RAIN_WB_FORCE_NOW      true

// ============================================================================
// Rain Gauge: Variabili globali statiche
//   Dichiarate QUI perché usate dalle funzioni rain_loadPersistedCounters /
//   rain_writeBack subito sotto, oltre che in loop() e setup().
// ============================================================================
static uint32_t      g_rain_total_offset_ticks   = 0;   // reset TOTALE manuale (MQTT cmd)
static uint32_t      g_rain_today_offset_ticks   = 0;   // reset OGNI MEZZANOTTE (auto)
static uint32_t      g_rain_today_day_epoch      = 0;   // ultimo giorno YYYYMMDD del reset oggi
static bool          g_rain_persist_dirty        = false; // forza salvataggio EEPROM subito
static uint32_t      s_rain_last_wb_ms           = 0;

static uint32_t      g_snap_rain_tick_count       = 0;
static float         g_snap_rain_total_mm         = NAN;
static float         g_snap_rain_today_mm         = NAN;
static float         g_snap_rain_rate_mmh         = NAN;
static uint32_t      g_snap_rain_last_age_s       = 0;
static bool          g_snap_rain_ok               = false;
static float         g_snap_rain_mm_per_tick      = 0.0f;

// Helper: calcola giorno corrente da Unix epoch (ESP8266: time(NULL)=epoch UTC o CET se timezone).
// Formato ritorno: 20261008 per 2026-10-08; 0 se tempo non disponibile.
static uint32_t rainGetTodayEpochDay(void) {
    time_t tNow = time(nullptr);
    if (tNow < (time_t)946684800UL) return 0;  // tempo non sincronizzato (< 2000-01-01)
    struct tm lt;
    memset(&lt, 0, sizeof(lt));
    localtime_r(&tNow, &lt);
    return (uint32_t)((lt.tm_year + 1900U) * 10000U) +
           (uint32_t)((lt.tm_mon + 1U) * 100U) +
           (uint32_t)(lt.tm_mday);
}

// Helper: salva RAIN PERSIST immediatamente (usa dopo reset TOTALE o dopo ogni tick).
// v3: cum_total_saved = actual_ticks - total_offset_ticks = valore cumulativo TOTALE
//     (preservato tra i reboot; actual viene riallineato in loading se minore)
static bool rainWritePersistImmediate(const char* reason) {
    RainPersist_t rp; memset(&rp, 0, sizeof(rp));
    rp.magic = RAIN_PERSIST_MAGIC_V3;
    // --- Calcolo atomico cum_total_saved senza interruzioni ---
    uint32_t act;
    noInterrupts();
    act = g_rain_tick_count;
    interrupts();
    if (act >= g_rain_total_offset_ticks) {
        rp.cum_total_saved = act - g_rain_total_offset_ticks;
    } else {
        rp.cum_total_saved = 0u;   // safety floor se offset disallineato (non dovrebbe succedere)
    }
    rp.total_offset_ticks = g_rain_total_offset_ticks;
    rp.today_offset_ticks = g_rain_today_offset_ticks;
    rp.today_day_epoch    = g_rain_today_day_epoch;
    rp.crc32 = 0;
    rp.crc32 = util_crc32((const uint8_t*)&rp, offsetof(RainPersist_t, crc32));
    EEPROM.begin(EEPROM_SIZE_BYTES);
    EEPROM.put(RAIN_PERSIST_EEPROM_OFFSET, rp);
    bool ok = EEPROM.commit();
    EEPROM.end();
    if (ok) {
        s_rain_last_wb_ms = millis();
        g_rain_persist_dirty = false;
        if (reason && *reason) {
            Serial.printf_P(PSTR("[RAIN] EEPROM Save v3 (%s): tot_off=%lu today_off=%lu day=%lu cum_total_saved=%lu (actual=%lu → valore=%lu)\n"),
                reason,
                (unsigned long)g_rain_total_offset_ticks,
                (unsigned long)g_rain_today_offset_ticks,
                (unsigned long)g_rain_today_day_epoch,
                (unsigned long)rp.cum_total_saved,
                (unsigned long)act,
                (unsigned long)rp.cum_total_saved);
        }
    } else {
        Serial.println(F("[RAIN] EEPROM Save FAIL: commit() error"));
    }
    return ok;
}

static void rain_loadPersistedCounters(void)
{
    RainPersist_t rp; memset(&rp, 0, sizeof(rp));
    EEPROM.begin(EEPROM_SIZE_BYTES);
    EEPROM.get(RAIN_PERSIST_EEPROM_OFFSET, rp);
    EEPROM.end();
    uint32_t calc_crc = util_crc32((const uint8_t*)&rp, offsetof(RainPersist_t, crc32));
    // --- v3 RAI3: formato con cum_total_saved (persistenza perfetta) ---
    if (rp.magic == RAIN_PERSIST_MAGIC_V3) {
        if (calc_crc != rp.crc32) {
            Serial.println(F("[RAIN] CRC RainPersist v3 fallito → azzero tutto. EEPROM corrotta?"));
            g_rain_total_offset_ticks = 0;
            g_rain_today_offset_ticks = 0;
            g_rain_today_day_epoch    = 0;
            return;
        }
        g_rain_today_offset_ticks = rp.today_offset_ticks;
        g_rain_today_day_epoch    = rp.today_day_epoch;
        // --- RIPRISTINO PERSISTENZA v3: ---
        //   Salvato cum_total_saved = X (es. 5 = 8.14mm).
        //   In boot actual RAM = 0.
        //   Vogliamo che total = actual - tot_off = X subito (senza attesa tick).
        //   Soluzione: (1) forza in RAM g_rain_tick_count = cum_total_saved
        //              (2) total_offset = 0 → total = X - 0 = X (OK)
        //   Se cum_total_saved = 0 (reset totale), non tocco niente.
        uint32_t cum = rp.cum_total_saved;
        noInterrupts();
        if (g_rain_tick_count < cum) {
            g_rain_tick_count = cum;   // ripristina contatore RAM al cumulativo
        }
        interrupts();
        // --- HOTFIX v3 load CRITICO:
        //   NON ricalcolare tot_off! Usa direttamente rp.total_offset_ticks.
        //   Il ricalcolo act_now - cum rompe quando cum=0 (reset totale o
        //   migrazione v2) portando tot_off a 0 invece del valore salvato N.
        g_rain_total_offset_ticks = rp.total_offset_ticks;
        uint32_t act_now;
        noInterrupts();
        act_now = g_rain_tick_count;
        interrupts();
        // Safety: garantisci che act_now >= tot_off + cum (coerenza matematica)
        uint32_t exp_min = g_rain_total_offset_ticks + cum;
        if (act_now < exp_min) {
            noInterrupts();
            g_rain_tick_count = exp_min;
            act_now = exp_min;
            interrupts();
        }
        Serial.printf_P(PSTR("[RAIN] Persistiti v3 caricati: cum_total_saved=%lu act_now=%lu → tot_off=%lu (da struct, NON ricalcolato) today_off=%lu day=%lu (totale=%lu tick ripristinato)\n"),
            (unsigned long)cum,
            (unsigned long)act_now,
            (unsigned long)g_rain_total_offset_ticks,
            (unsigned long)g_rain_today_offset_ticks,
            (unsigned long)g_rain_today_day_epoch,
            (unsigned long)cum);
        return;
    }
    // --- v2 RAI2: solo offset (senza cumulativo). Migriamo a v3 SUBITO dopo caricamento. ---
    if (rp.magic == RAIN_PERSIST_MAGIC_V2) {
        if (calc_crc != rp.crc32) {
            Serial.println(F("[RAIN] CRC RainPersist v2 fallito → azzero offset."));
            g_rain_total_offset_ticks = 0;
            g_rain_today_offset_ticks = 0;
            g_rain_today_day_epoch    = 0;
            rainWritePersistImmediate("migrate v2→v3 (CRC fail)");
            return;
        }
        g_rain_total_offset_ticks = rp.total_offset_ticks;
        g_rain_today_offset_ticks = rp.today_offset_ticks;
        g_rain_today_day_epoch    = rp.today_day_epoch;
        // --- HOTFIX migrazione v2→v3 CRITICO:
        //   Forza actual RAM >= tot_off_v2 PRIMA del salvataggio.
        //   Altrimenti rainWritePersistImmediate calcola
        //   cum_saved = max(0, 0 - tot_off) = 0 floor (perde cumulativo).
        //   Con actual = tot_off_v2 → cum_saved = actual - tot_off = 0 correttamente,
        //   e tot_off viene mantenuto e salvato in struct v3.
        uint32_t act_pre;
        noInterrupts();
        act_pre = g_rain_tick_count;
        if (g_rain_tick_count < rp.total_offset_ticks) {
            g_rain_tick_count = rp.total_offset_ticks;
        }
        interrupts();
        Serial.printf_P(PSTR("[RAIN] Trovato RainPersist v2 (tot_off=%lu today_off=%lu day=%lu). Migrazione v3: align actual %lu→%lu, cum_saved=actual-tot_off.\n"),
            (unsigned long)rp.total_offset_ticks,
            (unsigned long)rp.today_offset_ticks,
            (unsigned long)rp.today_day_epoch,
            (unsigned long)act_pre,
            (unsigned long)g_rain_tick_count);
        rainWritePersistImmediate("migr v2→v3");   // salva in v3 cum_total_saved calcolato
        return;
    }
    // --- v1 RAIN legacy (senza 2 offset): migra a v3. ---
    if (rp.magic == RAIN_PERSIST_MAGIC) {
        Serial.println(F("[RAIN] Trovato RainPersist v1 (legacy). Migrazione v3 → offset + cumulativo = 0."));
        g_rain_total_offset_ticks = 0;
        g_rain_today_offset_ticks = 0;
        g_rain_today_day_epoch    = 0;
        rainWritePersistImmediate("migr v1→v3");
        return;
    }
    Serial.println(F("[RAIN] Nessun contatore persistito (EEPROM vergine o formato non riconosciuto)."));
    g_rain_total_offset_ticks = 0;
    g_rain_today_offset_ticks = 0;
    g_rain_today_day_epoch    = 0;
    rainWritePersistImmediate("init EEPROM v3 virgin");
}

// Rain write-back: salvataggio EEPROM AD OGNI AGGIORNAMENTO (forza dirty → write immediato).
// Se non dirty: usa time-trigger backup ogni 1h (safety).
static void rain_writeBackIfNeeded(void)
{
    uint32_t now = millis();
    bool force_immediate = g_rain_persist_dirty;
    bool time_trigger = (s_rain_last_wb_ms == 0) ||
                        (now - s_rain_last_wb_ms) >= RAIN_WB_INTERVAL_MS;
    if (!force_immediate && !time_trigger) return;
    rainWritePersistImmediate(force_immediate ? "dirty" : "hourly");
}

// ============================================================================
// Helper calcoli + stringhe LTR390
// ============================================================================
static float ltr390_gainFactor(uint8_t gainId)
{
    switch (gainId & 0x07) {
        case 0: return 1.0f;
        case 1: return 3.0f;
        case 2: return 6.0f;
        case 3: return 9.0f;
        case 4: return 18.0f;
        default: return NAN;
    }
}

static float ltr390_integMs(uint8_t resId)
{
    switch (resId & 0x07) {
        case 0: return 400.0f;
        case 1: return 200.0f;
        case 2: return 100.0f;
        case 3: return 50.0f;
        case 4: return 25.0f;
        case 5: return 25.0f; // ridondante datasheet
        case 6: return 50.0f;
        case 7: return 100.0f;
        default: return NAN;
    }
}

// Formula Lux LTR390 — UFFICIALE (ESPHome ltr390 + Adafruit)
//   Il LTR390 ha UN SOLO output ALS (20-bit) già prefiltrato a risposta
//   fotopica (~occhio umano). NON serve rapporto CH0/CH1 (quello è per
//   sensori LTR-303/329/501 che hanno 2 fotodiodi separati).
//
//   lux = (0.6 * als_raw) / (gain * (integMs / 100))
//   WF (Window Factor) opzionale se installato dietro vetro:
//     lux = lux * wfac (wfac >= 1.0; 1.0 default nessun vetro)
static float ltr390_calcLux(uint32_t als_raw, float gain, float integMs)
{
    if (!isfinite(gain) || !isfinite(integMs) || gain <= 0.0f || integMs <= 0.0f) return NAN;
    float a = (float)als_raw;
    float wf = gain * (integMs / 100.0f);
    if (wf <= 0.0f) return 0.0f;
    float lux = (0.6f * a) / wf;
    if (!isfinite(lux)) return 0.0f;
    if (lux < 0.0f) lux = 0.0f;
    return lux;
}

// Formula UV Index: normalizza UV_raw al riferimento 18x gain / 100ms.
// UVI_rif 18x/100ms = UV_raw / 2300  →  UVI = (UV_raw * (18/gain) * (100/integ)) / 2300
static float ltr390_calcUVI(uint32_t uvRaw, float gain, float integMs)
{
    if (!isfinite(gain) || !isfinite(integMs) || gain <= 0.0f || integMs <= 0.0f) return NAN;
    float raw = (float)uvRaw;
    float norm = raw * (18.0f / gain) * (100.0f / integMs);
    float uvi = norm / LTR390_UVI_SENSITIVITY;
    if (!isfinite(uvi)) return 0.0f;
    if (uvi < 0.0f) uvi = 0.0f;
    return uvi;
}

static const char* ltr390_uvLevelStr(float uvi)
{
    if (!isfinite(uvi)) return "";
    if (uvi < 3.0f)  return "Low";
    if (uvi < 6.0f)  return "Moderate";
    if (uvi < 8.0f)  return "High";
    if (uvi < 11.0f) return "Very High";
    return "Extreme";
}

static const char* ltr390_lightClassStr(float lux)
{
    if (!isfinite(lux)) return "";
    if (lux < 10.0f)     return "Dark";
    if (lux < 200.0f)    return "Dim";
    if (lux < 1000.0f)   return "Normal";
    if (lux < 10000.0f)  return "Bright";
    return "Very Bright";
}

static const char* ltr390_gainStr(uint8_t gainId)
{
    static char buf[8];
    float f = ltr390_gainFactor(gainId);
    if (!isfinite(f)) return "";
    snprintf(buf, sizeof(buf), "%.0fx", (double)f);
    return buf;
}

static const char* ltr390_resStr(uint8_t resId)
{
    static const char PROGMEM tbl[8][16] = {
        "20bit/400ms", "19bit/200ms", "18bit/100ms", "17bit/50ms",
        "16bit/25ms",  "16bit/25ms",  "17bit/50ms",  "18bit/100ms"
    };
    static char buf[18];
    strcpy_P(buf, tbl[resId & 0x07]);
    return buf;
}

// ============================================================================
// Costanti Beaufort (13 classi) e conversioni pressione
// ============================================================================
static const char ROSE8[8][4] PROGMEM = {
    "N","NE","E","SE","S","SW","W","NW"
};

struct BeaufortInfo { float limitMs; const char name[22]; const char desc[36]; };
static const struct BeaufortInfo PROGMEM BEAUFORT_DB[13] = {
    { 0.2f,  "Calm",                 "Aria immobile" },
    { 1.5f,  "Bava di vento",        "Fumo verticale" },
    { 3.3f,  "Brezza leggera",       "Direzione vento dal fumo" },
    { 5.4f,  "Brezza tesa",          "Foglie movimento" },
    { 7.9f,  "Vento moderato",       "Polvere, carte sollevate" },
    { 10.7f, "Vento teso",           "Alberi ondeggiano" },
    { 13.8f, "Vento fresco",         "Rami mossi" },
    { 17.1f, "Vento forte",          "Alberi interi oscillano" },
    { 20.7f, "Burrasca",             "Rami spezzati" },
    { 24.4f, "Burrasca forte",       "Danni lievi a tetti" },
    { 28.4f, "Tempesta",             "Danni considerevoli" },
    { 32.6f, "Tempesta violenta",    "Danni estesi" },
    { 1e9f,  "Uragano",              "Distruzione" }
};

static const char* beaufortName(uint8_t bf)
{
    static char buf[24];
    if (bf > 12) bf = 12;
    strcpy_P(buf, BEAUFORT_DB[bf].name);
    return buf;
}
static const char* beaufortDesc(uint8_t bf)
{
    static char buf[40];
    if (bf > 12) bf = 12;
    strcpy_P(buf, BEAUFORT_DB[bf].desc);
    return buf;
}

static const char* rose8Of(float deg)
{
    static char buf[4];
    if (!isfinite(deg)) { buf[0] = 0; return buf; }
    const float step = 360.0f / 8.0f;
    const float h = step / 2.0f;
    float a = fmodf(deg, 360.0f);
    if (a < 0) a += 360.0f;
    a = fmodf(a + h, 360.0f);
    uint8_t i = (uint8_t)floorf(a / step);
    if (i >= 8) i = 7;
    strcpy_P(buf, ROSE8[i]);
    return buf;
}

static const char* heading3(float deg)
{
    static char buf[16];
    if (!isfinite(deg)) { buf[0] = 0; return buf; }
    int h = (int)roundf(fmodf(deg, 360.0f));
    if (h < 0) h += 360;
    if (h >= 360) h = 0;
    snprintf(buf, sizeof(buf), "%03d°", h);
    return buf;
}

#if 0 // --- funzioni helper mantenute per uso futuro, evitano warning unused ---
static const char* degToDMS(float deg)
{
    static char buf[32];
    if (!isfinite(deg)) { buf[0] = 0; return buf; }
    float a = fmodf(deg, 360.0f);
    if (a < 0) a += 360.0f;
    int d = (int)floorf(a);
    float mFloat = (a - d) * 60.0f;
    int m = (int)floorf(mFloat);
    float sFloat = (mFloat - m) * 60.0f;
    snprintf(buf, sizeof(buf), "%d° %02d' %04.1f\"", d, m, sFloat);
    return buf;
}
#endif

static const char* sensorStatusList(uint32_t bits)
{
    static char buf[112];
    snprintf(buf, sizeof(buf), "%s | %s | %s | %s | %s | %s%s",
        (bits & 0x001) ? "AS5600"  : "-",
        (bits & 0x020) ? "SHT31"   : "-",
        (bits & 0x004) ? "BME280"  : "-",
        (bits & 0x008) ? "HALL"    : "-",
        (bits & 0x010) ? "LTR390"  : "-",
        (bits & 0x100) ? "RAIN"    : "-",
        (bits & 0x040) ? " · HTR"  : "");
    return buf;
}

static const char* sensorStatusHexBin(uint32_t bits)
{
    static char buf[40];
    uint16_t w = (uint16_t)(bits & 0xFFFF);
    snprintf(buf, sizeof(buf), "0x%03X  b%d%d%d%d%d%d%d%d%d",
        w,
        (w>>8)&1,
        (w>>7)&1,(w>>6)&1,(w>>5)&1,(w>>4)&1,
        (w>>3)&1,(w>>2)&1,(w>>1)&1,(w>>0)&1);
    return buf;
}

#if 0 // --- helper mantenuti per uso futuro ---
static const char* beaufortFull(uint8_t bf)
{
    static char buf[80];
    if (bf > 12) bf = 12;
    char n[24], d[40];
    strcpy_P(n, BEAUFORT_DB[bf].name);
    strcpy_P(d, BEAUFORT_DB[bf].desc);
    snprintf(buf, sizeof(buf), "%s \xC2\xB7 %s", n, d);
    return buf;
}

static uint8_t quadrantMath(float deg)
{
    if (!isfinite(deg)) return 0;
    float a = fmodf(deg, 360.0f);
    if (a < 0) a += 360.0f;
    if (a < 0.01f || fabsf(a-90.0f) < 0.01f || fabsf(a-180.0f) < 0.01f || fabsf(a-270.0f) < 0.01f) return 0;
    if (a < 90.0f) return 1;
    if (a < 180.0f) return 2;
    if (a < 270.0f) return 3;
    return 4;
}
#endif

// Stato LTR390 + snapshot MQTT
static bool         g_ltr390_ok           = false;
// ---- State machine ALS <-> UVS (switch modale, millis() non bloccante) ----
typedef enum {
  LTR390_ST_IDLE_START_ALS = 0,
  LTR390_ST_WAIT_ALS_DRDY,
  LTR390_ST_WARMUP_UVS_FROM_ALS,  // Hybrid STANDBY 8ms non bloccante prima di UVS
  LTR390_ST_IDLE_START_UVS,
  LTR390_ST_WAIT_UVS_DRDY,
  LTR390_ST_WARMUP_ALS_FROM_UVS,  // Hybrid STANDBY 8ms non bloccante prima di ALS
  LTR390_ST_WD_RECOVERY_10MS,     // Watchdog recovery: delay10ms non bloccante
  LTR390_ST_WD_RECOVERY_2MS       // Watchdog recovery: delay2ms non bloccante
} ltr390_state_t;
static const char* ltr390_state_name(ltr390_state_t s){
  switch(s){
    case LTR390_ST_IDLE_START_ALS:     return "IDLE_START_ALS";
    case LTR390_ST_WAIT_ALS_DRDY:      return "WAIT_ALS_DRDY";
    case LTR390_ST_WARMUP_UVS_FROM_ALS:return "WARMUP_UVS";
    case LTR390_ST_IDLE_START_UVS:     return "IDLE_START_UVS";
    case LTR390_ST_WAIT_UVS_DRDY:      return "WAIT_UVS_DRDY";
    case LTR390_ST_WARMUP_ALS_FROM_UVS:return "WARMUP_ALS";
    case LTR390_ST_WD_RECOVERY_10MS:   return "WD_REC_10MS";
    case LTR390_ST_WD_RECOVERY_2MS:    return "WD_REC_2MS";
    default: return "?";
  }
}
static ltr390_state_t g_ltr390_state     = LTR390_ST_IDLE_START_ALS;
static unsigned long g_ltr390_state_ts   = 0;
static uint32_t      g_ltr390_last_als   = 0;
static uint32_t      g_ltr390_last_uv    = 0;
static bool          g_ltr390_als_new    = false;
static bool          g_ltr390_uv_new     = false;
static uint32_t      g_ltr390_als_cnt    = 0;
static uint32_t      g_ltr390_uv_cnt     = 0;
// Statistiche diagnostiche write-verify
static uint32_t      g_ltr390_wv_try     = 0;  // tentativi scrittura MAIN_CTRL attivo
static uint32_t      g_ltr390_wv_fail    = 0;  // write-verify fail MAIN_CTRL
static uint32_t      g_ltr390_wv_rdy     = 0;  // write-verify OK
static uint8_t       g_ltr390_last_wv_want = 0xFF;
static uint8_t       g_ltr390_last_wv_got  = 0xFF;
static uint32_t      g_ltr390_start_als_count = 0;
static uint32_t      g_ltr390_start_uvs_count = 0;
// Statistiche strumentazione scientific debugging (sessione ltr-fsm-stuck-wv-try-0)
static uint32_t      g_ltr390_poll_calls     = 0;   // chiamate totali poll() da boot
// Fallback hardware: se continuous mode non funziona dopo timeout, usa hybrid
// (STANDBY → delay → start modalità successiva) per il ciclo successivo.
static uint8_t       g_ltr390_hybrid_next     = 0;   // 0=continuous, 1=prossimo switch usa STANDBY intermedi
static uint32_t      g_ltr390_hybrid_count    = 0;   // conteggio attivazioni hybrid per DIAG
static uint32_t     g_snap_ltr_als_raw    = 0;  // era g_snap_ltr_ch0
static uint32_t     g_snap_ltr_uv_raw     = 0;
static float        g_snap_ltr_lux        = NAN;
static float        g_snap_ltr_uvi        = NAN;
static uint8_t      g_snap_ltr_gain       = 0;
static uint8_t      g_snap_ltr_res        = 0;

// Watchdog LTR390: forza publish MQTT a zero se i valori raw rimangono
// == 0 per 15 minuti consecutivi. Risolve il problema dei valori "bloccati"
// a valori non-nulli pubblicati precedentemente quando il sensore scende
// lentamente a zero di notte. 0 = watchdog non attivo (valore != 0 in questo momento).
#define LTR390_ZERO_WD_MS     (15UL * 60UL * 1000UL)
static unsigned long g_ltr390_als_zero_ms = 0;
static unsigned long g_ltr390_uv_zero_ms  = 0;

// ============================================================================
// Stato runtime SHT31-D + snapshot MQTT / WS
// ============================================================================
static bool         g_sht31_ok           = false;
typedef enum {
  SHT31_HTR_OFF = 0,
  SHT31_HTR_WARMUP,          // appena acceso, attesa min on-time
  SHT31_HTR_ON,              // acceso, aspetto soglia RH_OFF
  SHT31_HTR_COOLDOWN         // appena spento, min off-time
} sht31_heater_state_t;
static sht31_heater_state_t g_sht31_heater_state = SHT31_HTR_OFF;
static unsigned long g_sht31_heater_ts    = 0;
static bool          g_sht31_heater_forced_on = false;
static uint32_t      g_sht31_heater_cycle_count = 0;
static uint32_t      g_sht31_crc_error_count   = 0;
static uint32_t      g_sht31_i2c_error_count   = 0;

static float        g_snap_sht31_t      = NAN;
static float        g_snap_sht31_rh     = NAN;
static float        g_snap_sht31_dp     = NAN;   // Dew Point
static float        g_snap_sht31_fp     = NAN;   // Frost Point
static float        g_snap_sht31_wb     = NAN;   // Wet Bulb (Stull 2011)
static float        g_snap_sht31_vpd    = NAN;   // Vapor Pressure Deficit [kPa]
static float        g_snap_sht31_ah     = NAN;   // Umidità assoluta g/m³
static float        g_snap_sht31_hi     = NAN;   // Heat Index SHT31
static float        g_snap_sht31_hx     = NAN;   // Humidex SHT31
static bool         g_snap_sht31_htr_on = false;

// ============================================================================
// Rain Gauge Persistenza + Snapshots
//   (typedef e tutte le variabili statiche spostate PRIMA di rain_loadPersistedCounters,
//    vedi blocco helper rain sopra ~L2391; qui lasciato solo segnaposto per evitare
//    errori "defined but not used" dovuti a dichiarazione postuma)
// ============================================================================

// ============================================================================
// I2C bus scanner diagnostico
// ============================================================================
static void i2c_diagPins(const char* label)
{
    pinMode(D2, INPUT_PULLUP);
    pinMode(D1, INPUT_PULLUP);
    delay(2);
    uint8_t sda = digitalRead(D2);
    uint8_t scl = digitalRead(D1);
    char bl[128];
    snprintf(bl, sizeof(bl),
        "[I2C] DIAG PINS %s: SDA(D2/GPIO4)=%u %s · SCL(D1/GPIO5)=%u %s",
        label ? label : "",
        (unsigned)sda, (sda==HIGH)?"PULLUP_OK":"SDA_LOW_SHORT",
        (unsigned)scl, (scl==HIGH)?"PULLUP_OK":"SCL_LOW_SHORT");
    Serial.println(bl);
}

// Sequenza standard NXP UM10204 §3.1.16 "Bus clear" + SMBus recovery:
// Se un sensore mantiene SDA basso (lockup a metà transazione, ACK non rilasciato),
// inviamo 9 impulsi di SCL via bit-bang open-drain (senza libreria Wire) per
// forzare il device a finire il byte interno e rilasciare SDA, poi START+STOP.
static bool i2c_busRecover(void)
{
    const uint8_t SDA_PIN = D2;
    const uint8_t SCL_PIN = D1;

    // (1) Entrambe linee rilasciate (open drain + pull-up esterni/interni)
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(20);

    // Se SDA è già alto → non c'è niente da recuperare, successo vuoto
    if (digitalRead(SDA_PIN) == HIGH) {
        Serial.println(F("[I2C] BUS RECOVERY: SDA già alto · skip 9-clock"));
        return true;
    }

    Serial.println(F("[I2C] BUS RECOVERY: SDA=LOW · inizio 9x SCL pulses open-drain"));

    // (2) 9 clock impulsi (un byte intero + ACK) · open-drain: LOW=DriveLow, HIGH=Release
    for (uint8_t i = 0; i < 9; i++) {
        // SCL LOW (drive)
        pinMode(SCL_PIN, OUTPUT);
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(10);   // tLOW = min 4.7µs (Standard mode 100kHz)
        // SCL HIGH (rilascia → pull-up)
        pinMode(SCL_PIN, INPUT_PULLUP);
        delayMicroseconds(10);   // tHIGH = min 4.0µs
        // Se SDA è tornato alto a metà → possiamo fermarci prima
        if (digitalRead(SDA_PIN) == HIGH) {
            Serial.printf("[I2C] BUS RECOVERY: SDA released after pulse #%u/9\r\n", (unsigned)(i+1));
            break;
        }
    }

    // (3) Controllo se SDA si è liberato
    uint8_t sda_after = digitalRead(SDA_PIN);
    Serial.printf("[I2C] BUS RECOVERY: post-9clk SDA=%u\r\n", (unsigned)sda_after);

    if (sda_after == LOW) {
        Serial.println(F("[I2C] BUS RECOVERY: FAIL · SDA ancora LOW dopo 9 pulses · VERO CORTO FISICO?"));
        return false;
    }

    // (4) Genera condizione START + STOP per assicurare tutti i device siano in stato idle
    // START: SDA HIGH→LOW mentre SCL HIGH
    pinMode(SDA_PIN, INPUT_PULLUP);  delayMicroseconds(5);
    pinMode(SCL_PIN, INPUT_PULLUP);  delayMicroseconds(5);
    pinMode(SDA_PIN, OUTPUT); digitalWrite(SDA_PIN, LOW); delayMicroseconds(5);
    pinMode(SCL_PIN, OUTPUT); digitalWrite(SCL_PIN, LOW); delayMicroseconds(5);
    // STOP: SDA LOW→HIGH mentre SCL HIGH
    pinMode(SCL_PIN, INPUT_PULLUP);  delayMicroseconds(5);
    pinMode(SDA_PIN, INPUT_PULLUP);  delayMicroseconds(5);
    Serial.println(F("[I2C] BUS RECOVERY: START+STOP generato · bus in idle"));

    // (5) Sicurezza: ripristina modalità input + pullup
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delay(1);
    return true;
}

static void i2c_scanBus(void)
{
    // ----- EXTENDED SCAN 0x00..0x77 (cattura AS3935 0x02/0x03 che sono fuori da 0x08!) -----
    Serial.println(F("--- I2C EXT SCAN 0x00..0x77 ---"));
    uint8_t ext_count = 0;
    for (uint8_t a = 0x00; a <= 0x77; a++) {
        Wire.beginTransmission(a);
        uint8_t err = Wire.endTransmission(true);
        if (err == 0) {
            const char *lbl = "?";
            switch(a){
                case 0x00: lbl = "I2C-General-Call(0x00) · NON è un device"; break;
                case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06: case 0x07: lbl = "reserved-low"; break;
                case 0x36: lbl = "AS5600"; break;
                case 0x44: case 0x45: lbl = "SHT31-D"; break;
                case 0x3C: case 0x3D: lbl = "SSD1306"; break;
                case 0x48: case 0x49: case 0x4A: case 0x4B: lbl = "ADS1115"; break;
                case 0x53: lbl = "LTR390"; break;
                case 0x23: case 0x5C: lbl = "BH1750"; break;
                case 0x70: case 0x71: case 0x72: case 0x73: lbl = "TCA9548A"; break;
                case 0x76: case 0x77: lbl = "BME280/BMP280"; break;
            }
            char buf[64];
            snprintf(buf, sizeof(buf), "  0x%02X  ACK  %s", a, lbl);
            Serial.println(buf);
            ext_count++;
        }
    }
    char bufExt[64];
    snprintf(bufExt, sizeof(bufExt), "--- I2C EXT SCAN END, %u device(s) ---", ext_count);
    Serial.println(bufExt);

    // ----- STANDARD SCAN 0x08..0x77 (storico per retrocompatibilita') -----
    Serial.println(F("--- I2C SCAN START 0x08..0x77 ---"));
    uint8_t count = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        Wire.beginTransmission(a);
        uint8_t err = Wire.endTransmission(true);
        if (err == 0) {
            const char *lbl = "?";
            switch(a){
                case 0x36: lbl = "AS5600"; break;
                case 0x44: case 0x45: lbl = "SHT31-D"; break;
                case 0x3C: case 0x3D: lbl = "SSD1306"; break;
                case 0x48: case 0x49: case 0x4A: case 0x4B: lbl = "ADS1115"; break;
                case 0x53: lbl = "LTR390"; break;
                case 0x23: case 0x5C: lbl = "BH1750"; break;
                case 0x70: case 0x71: case 0x72: case 0x73: lbl = "TCA9548A"; break;
                case 0x76: case 0x77: lbl = "BME280/BMP280"; break;
            }
            char buf[40];
            snprintf(buf, sizeof(buf), "  0x%02X  ACK  %s", a, lbl);
            Serial.println(buf);
            count++;
        }
    }
    char buf[48];
    snprintf(buf, sizeof(buf), "--- I2C SCAN END, %u device(s) ---", count);
    Serial.println(buf);
}

// ============================================================================
// Funzioni helper AS5600
// ============================================================================
static uint8_t as5600_writeReg16(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)((value >> 8) & 0xFF));
    Wire.write((uint8_t)(value & 0xFF));
    return Wire.endTransmission();
}

static uint8_t as5600_readReg16(uint8_t reg, uint16_t *out)
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) return err;
    if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return 0xFE;
    uint16_t hi = (uint16_t)Wire.read();
    uint16_t lo = (uint16_t)Wire.read();
    *out = (uint16_t)(((hi & 0x00FF) << 8) | (lo & 0x00FF));
    return 0;
}

static uint8_t as5600_readReg8(uint8_t reg, uint8_t *out)
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) return err;
    if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) != 1) return 0xFE;
    *out = (uint8_t)Wire.read();
    return 0;
}

// ============================================================================
// [NUOVO] Heading Filter · Aritmetica circolare + Persistenza EEPROM
// ============================================================================

// Differenza angolare shortest-path: risultato in [-180, +180)
static float circularDeltaDeg(float a_deg, float b_deg)
{
    float d = a_deg - b_deg;
    while (d >=  180.0f) d -= 360.0f;
    while (d <  -180.0f) d += 360.0f;
    return d;
}

// Media VETTORIALE di N angoli (correttezza matematica per wraparound 0/360).
// Restituisce NaN se nessun campione valido.
static float circularVectorMeanDeg(const float* angles_deg, uint8_t count)
{
    if (!angles_deg || count == 0) return NAN;
    double sum_s = 0.0;
    double sum_c = 0.0;
    uint8_t valid = 0;
    for (uint8_t i = 0; i < count; i++) {
        float a = angles_deg[i];
        if (!isfinite(a)) continue;
        // Unwrap in [-180,+180) per seno/coseno (seno e coseno hanno periodo 360,
        // quindi qualsiasi rappresentazione va bene ma riduciamo range per
        // minimizzare errore di precisione float in sin/cos).
        while (a >=  180.0f) a -= 360.0f;
        while (a <  -180.0f) a += 360.0f;
        double rad = (double)a * (PI / 180.0);
        sum_s += sin(rad);
        sum_c += cos(rad);
        valid++;
    }
    if (valid == 0) return NAN;
    double mean_s = sum_s / (double)valid;
    double mean_c = sum_c / (double)valid;
    double mean_rad = atan2(mean_s, mean_c);
    float deg = (float)(mean_rad * (180.0 / PI));
    // Normalizza in [0, 360)
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

static void hdrf_setDefaults(HeadingFilterConfig_t* c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->magic = HDRFILT_MAGIC;
    c->version = HDRFILT_VERSION_CURR;
    c->filter_enable           = (uint8_t)HDRFILT_FILTER_EN_DEF;
    c->spike_threshold_deg_x100 = (int16_t)HDRFILT_SPIKE_DEG_DEF_X100;
    c->movavg_window           = (uint8_t)HDRFILT_WINDOW_DEF;
    c->output_period_ms        = (uint16_t)HDRFILT_OUTPUT_MS_DEF;
    c->ema_alpha_x100          = (uint8_t)HDRFILT_EMA_ALPHA_DEF_X100;
}

static void hdrf_sanitize(HeadingFilterConfig_t* c)
{
    if (!c) return;
    if (c->magic != HDRFILT_MAGIC || c->version > HDRFILT_VERSION_CURR || c->version < 1) {
        hdrf_setDefaults(c);
        return;
    }
    // Backward compat: v1 → v3: aggiunge ema_alpha_x100 + filter_enable=1
    if (c->version == 1) {
        c->version = HDRFILT_VERSION_CURR;
        c->ema_alpha_x100 = (uint8_t)HDRFILT_EMA_ALPHA_DEF_X100;
        c->filter_enable  = (uint8_t)HDRFILT_FILTER_EN_DEF;
    }
    // Backward compat: v2 → v3: aggiunge filter_enable=1 (preservando parametri esistenti)
    if (c->version == 2) {
        c->version = HDRFILT_VERSION_CURR;
        c->filter_enable  = (uint8_t)HDRFILT_FILTER_EN_DEF;
    }
    // Master enable (boolean clamp)
    if (c->filter_enable > 1U) c->filter_enable = 1U;
    // Spike threshold: [0..36000] x100 (0=disabilita, 36000=disabilita)
    if (c->spike_threshold_deg_x100 < (int16_t)HDRFILT_SPIKE_DEG_MIN_X100)
        c->spike_threshold_deg_x100 = (int16_t)HDRFILT_SPIKE_DEG_MIN_X100;
    if (c->spike_threshold_deg_x100 > (int16_t)HDRFILT_SPIKE_DEG_MAX_X100)
        c->spike_threshold_deg_x100 = (int16_t)HDRFILT_SPIKE_DEG_MAX_X100;
    // Window: [1..240]
    if (c->movavg_window < HDRFILT_WINDOW_MIN) c->movavg_window = HDRFILT_WINDOW_MIN;
    if (c->movavg_window > HDRFILT_WINDOW_MAX) c->movavg_window = HDRFILT_WINDOW_MAX;
    // Output period: [50..60000] ms
    if (c->output_period_ms < HDRFILT_OUTPUT_MS_MIN) c->output_period_ms = (uint16_t)HDRFILT_OUTPUT_MS_MIN;
    if (c->output_period_ms > HDRFILT_OUTPUT_MS_MAX) c->output_period_ms = (uint16_t)HDRFILT_OUTPUT_MS_MAX;
    // EMA alpha: [0..100] x100 (0=disattiva, 100=nessun ritardo: out = in)
    if (c->ema_alpha_x100 < HDRFILT_EMA_ALPHA_MIN_X100) c->ema_alpha_x100 = HDRFILT_EMA_ALPHA_MIN_X100;
    if (c->ema_alpha_x100 > HDRFILT_EMA_ALPHA_MAX_X100) c->ema_alpha_x100 = HDRFILT_EMA_ALPHA_MAX_X100;
}

static bool hdrf_loadFromEEPROM(void)
{
    HeadingFilterConfig_t c; memset(&c, 0, sizeof(c));
    EEPROM.begin(EEPROM_SIZE_BYTES);
    EEPROM.get(HDRFILT_EEPROM_OFFSET, c);
    EEPROM.end();
    bool ok = false;
    if (c.magic == HDRFILT_MAGIC && c.version >= 1 && c.version <= HDRFILT_VERSION_CURR) {
        if (c.version == HDRFILT_VERSION_CURR) {
            uint32_t calc = util_crc32((const uint8_t*)&c, offsetof(HeadingFilterConfig_t, crc32));
            if (calc == c.crc32) ok = true;
        } else {
            // v1 (in EEPROM): different layout (sizeof <= our new struct since we added to the END),
            // trust the data, we will convert reserved_1 → ema_alpha_x100 and bump version inside sanitize.
            ok = true;
        }
    }
    if (!ok) {
        Serial.println(F("[HDGF] EEPROM non valida/assente → applico defaults."));
        hdrf_setDefaults(&c);
    }
    hdrf_sanitize(&c);
    // Backup nella struttura runtime e applica runtime
    g_hdrf_cfg = c;
    Serial.printf_P(PSTR("[HDGF] Load: spike=%.2f°  win=%u  out=%ums  ema=%.2f\r\n"),
        (double)((float)c.spike_threshold_deg_x100 / 100.0f),
        (unsigned)c.movavg_window,
        (unsigned)c.output_period_ms,
        (double)((float)c.ema_alpha_x100 / 100.0f));
    return ok;
}

static bool hdrf_saveToEEPROM(const HeadingFilterConfig_t* c_in)
{
    if (!c_in) return false;
    HeadingFilterConfig_t c = *c_in;
    hdrf_sanitize(&c);
    c.magic = HDRFILT_MAGIC;
    c.version = HDRFILT_VERSION_CURR;
    c.crc32 = util_crc32((const uint8_t*)&c, offsetof(HeadingFilterConfig_t, crc32));
    EEPROM.begin(EEPROM_SIZE_BYTES);
    EEPROM.put(HDRFILT_EEPROM_OFFSET, c);
    bool ok = EEPROM.commit();
    EEPROM.end();
    if (ok) {
        g_hdrf_cfg = c;
        Serial.printf_P(PSTR("[HDGF] Saved EEPROM OK: spike=%.2f°  win=%u  out=%ums  ema=%.2f\r\n"),
            (double)((float)c.spike_threshold_deg_x100 / 100.0f),
            (unsigned)c.movavg_window,
            (unsigned)c.output_period_ms,
            (double)((float)c.ema_alpha_x100 / 100.0f));
    } else {
        Serial.println(F("[HDGF] ERR: EEPROM.commit() fallita."));
    }
    return ok;
}

// Applica parametri a runtime: se la finestra o l'α sono cambiati, resetta buffer/EMA state.
static void hdrf_applyRuntime(const HeadingFilterConfig_t* c_new)
{
    if (!c_new) return;
    uint8_t old_win   = g_hdrf_cfg.movavg_window;
    uint8_t old_ax100 = g_hdrf_cfg.ema_alpha_x100;
    g_hdrf_cfg = *c_new;
    if (old_win != c_new->movavg_window || old_ax100 != c_new->ema_alpha_x100) {
        // Reset buffer circolare + stato EMA vettoriale interno
        memset(g_hdrf_circbuf, 0, sizeof(g_hdrf_circbuf));
        g_hdrf_circbuf_head = 0;
        g_hdrf_circbuf_fill = 0;
        g_hdrf_smoothed_deg = g_hdrf_last_valid_deg;
        extern float g_hdrf_ema_s_sin, g_hdrf_ema_s_cos;
        extern bool  g_hdrf_ema_initialized;
        if (isfinite(g_hdrf_last_valid_deg)) {
            float r = g_hdrf_last_valid_deg * (float)(PI / 180.0);
            g_hdrf_ema_s_sin = sinf(r);
            g_hdrf_ema_s_cos = cosf(r);
        } else {
            g_hdrf_ema_s_sin = 0.0f; g_hdrf_ema_s_cos = 0.0f;
        }
        g_hdrf_ema_initialized = false;
        Serial.printf_P(PSTR("[HDGF] win(%u→%u) or ema(%.2f→%.2f): buffer+EMA reset.\r\n"),
            (unsigned)old_win, (unsigned)c_new->movavg_window,
            (double)((float)old_ax100 / 100.0f),
            (double)((float)c_new->ema_alpha_x100 / 100.0f));
    } else {
        Serial.printf_P(PSTR("[HDGF] Runtime apply: spike=%.2f° win=%u out=%ums ema=%.2f\r\n"),
            (double)((float)c_new->spike_threshold_deg_x100 / 100.0f),
            (unsigned)c_new->movavg_window,
            (unsigned)c_new->output_period_ms,
            (double)((float)c_new->ema_alpha_x100 / 100.0f));
    }
}

// ============================================================================
// SensorFilterConfig_t · helpers (setDefaults / sanitize / load / save / applyRuntime)
//   Pattern identico a HeadingFilterConfig_t · EEPROM offset 3584 magic SFIL
// ============================================================================
static void sfil_setDefaults(SensorFilterConfig_t* c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->magic = SFIL_MAGIC;
    c->version = SFIL_VERSION_CURR;
    c->hall_ema_alpha_x1000   = (uint16_t)SFIL_HALL_EMA_DEF_X1000;
    c->hall_hold_decay_x10000 = (uint16_t)SFIL_HALL_DECAY_DEF_X10000;
    c->rain_rate_ema_x1000    = (uint16_t)SFIL_RRATE_DEF_X1000;
}

static void sfil_sanitize(SensorFilterConfig_t* c)
{
    if (!c) return;
    if (c->magic != SFIL_MAGIC || c->version > SFIL_VERSION_CURR || c->version < 1) {
        sfil_setDefaults(c);
        return;
    }
    // Hall EMA alpha x1000 (0.010..1.000)
    if (c->hall_ema_alpha_x1000 < SFIL_HALL_EMA_MIN_X1000) c->hall_ema_alpha_x1000 = SFIL_HALL_EMA_MIN_X1000;
    if (c->hall_ema_alpha_x1000 > SFIL_HALL_EMA_MAX_X1000) c->hall_ema_alpha_x1000 = SFIL_HALL_EMA_MAX_X1000;
    // Hall hold decay x10000 (0.9000..0.9999)
    if (c->hall_hold_decay_x10000 < SFIL_HALL_DECAY_MIN_X10000) c->hall_hold_decay_x10000 = SFIL_HALL_DECAY_MIN_X10000;
    if (c->hall_hold_decay_x10000 > SFIL_HALL_DECAY_MAX_X10000) c->hall_hold_decay_x10000 = SFIL_HALL_DECAY_MAX_X10000;
    // Rain rate EMA x1000
    if (c->rain_rate_ema_x1000 < SFIL_RRATE_MIN_X1000) c->rain_rate_ema_x1000 = SFIL_RRATE_MIN_X1000;
    if (c->rain_rate_ema_x1000 > SFIL_RRATE_MAX_X1000) c->rain_rate_ema_x1000 = SFIL_RRATE_MAX_X1000;
}

static bool sfil_saveToEEPROM(const SensorFilterConfig_t* c_in)
{
    if (!c_in) return false;
    SensorFilterConfig_t c = *c_in;
    sfil_sanitize(&c);
    c.magic = SFIL_MAGIC;
    c.version = SFIL_VERSION_CURR;
    c.crc32 = util_crc32((const uint8_t*)&c, offsetof(SensorFilterConfig_t, crc32));
    EEPROM.begin(EEPROM_SIZE_BYTES);
    EEPROM.put(SFIL_EEPROM_OFFSET, c);
    bool ok = EEPROM.commit();
    EEPROM.end();
    if (ok) {
        g_sfil_cfg = c;
        Serial.printf_P(PSTR("[SFIL] Saved EEPROM OK: hall_ema=%.3f  hall_decay=%.4f  rain_ema=%.3f\r\n"),
            (double)((float)c.hall_ema_alpha_x1000 / 1000.0f),
            (double)((float)c.hall_hold_decay_x10000 / 10000.0f),
            (double)((float)c.rain_rate_ema_x1000 / 1000.0f));
    } else {
        Serial.println(F("[SFIL] ERR: EEPROM.commit() fallita."));
    }
    return ok;
}

// Applica parametri a runtime: se α EMA vento/rain o decay cambiano →
// flag reset stati EMA vento+rain (seed da zero al primo raw)
static void sfil_applyRuntime(const SensorFilterConfig_t* c_new)
{
    if (!c_new) return;
    uint16_t old_hema = g_sfil_cfg.hall_ema_alpha_x1000;
    uint16_t old_hdec = g_sfil_cfg.hall_hold_decay_x10000;
    uint16_t old_rema = g_sfil_cfg.rain_rate_ema_x1000;
    g_sfil_cfg = *c_new;
    if (old_hema != c_new->hall_ema_alpha_x1000 || old_hdec != c_new->hall_hold_decay_x10000 || old_rema != c_new->rain_rate_ema_x1000) {
        // Flag: resetta stati EMA vento e rainrate al prossimo loop (vedi loop rispettive sezioni)
        g_sfil_reset_ema_requested = true;
        Serial.printf_P(PSTR("[SFIL] filter changed (hall_ema %.3f→%.3f  decay %.4f→%.4f  rain_ema %.3f→%.3f) → reset EMA stati.\r\n"),
            (double)((float)old_hema / 1000.0f),  (double)((float)c_new->hall_ema_alpha_x1000 / 1000.0f),
            (double)((float)old_hdec / 10000.0f), (double)((float)c_new->hall_hold_decay_x10000 / 10000.0f),
            (double)((float)old_rema / 1000.0f),  (double)((float)c_new->rain_rate_ema_x1000 / 1000.0f));
    }
}

static bool sfil_loadFromEEPROM(void)
{
    SensorFilterConfig_t c; memset(&c, 0, sizeof(c));
    EEPROM.begin(EEPROM_SIZE_BYTES);
    EEPROM.get(SFIL_EEPROM_OFFSET, c);
    EEPROM.end();
    bool ok = false;
    if (c.magic == SFIL_MAGIC && c.version >= 1 && c.version <= SFIL_VERSION_CURR) {
        uint32_t calc = util_crc32((const uint8_t*)&c, offsetof(SensorFilterConfig_t, crc32));
        if (calc == c.crc32) ok = true;
    }
    if (!ok) {
        Serial.println(F("[SFIL] EEPROM non valida/assente → applico defaults e salvo."));
        sfil_setDefaults(&c);
        sfil_saveToEEPROM(&c);
    } else {
        sfil_sanitize(&c);
        g_sfil_cfg = c;
        Serial.printf_P(PSTR("[SFIL] Load OK: hall_ema=%.3f  hall_decay=%.4f  rain_ema=%.3f\r\n"),
            (double)((float)c.hall_ema_alpha_x1000 / 1000.0f),
            (double)((float)c.hall_hold_decay_x10000 / 10000.0f),
            (double)((float)c.rain_rate_ema_x1000 / 1000.0f));
    }
    // reset ema stati su boot: inizializza stato EMA interno al primo campione (vedi loop
    return ok;
}

// Forward declarations (funzioni definite più in basso, usate in config_applyRuntimeFromWeb)
static uint8_t ltr390_writeReg8(uint8_t reg, uint8_t val);
static void    sht31_init(void);
static void    bme280_init(void);

static void as5600_applyConfig(void)
{
    uint16_t confVal = 0;
    confVal |= ((uint16_t)(AS5600_CONF_WD   & 0x01)) << 13;
    confVal |= ((uint16_t)(AS5600_CONF_FTH  & 0x07)) << 10;
    confVal |= ((uint16_t)(AS5600_CONF_SF   & 0x07)) <<  7;
    confVal |= ((uint16_t)(AS5600_CONF_PWMF & 0x03)) <<  5;
    confVal |= ((uint16_t)(AS5600_CONF_OUTS & 0x03)) <<  3;
    confVal |= ((uint16_t)(AS5600_CONF_HYST & 0x03)) <<  1;
    confVal |= ((uint16_t)(AS5600_CONF_PM   & 0x03)) <<  0;
    as5600_writeReg16(AS5600_REG_CONF, confVal);
    uint16_t zpos = (g_config.as5600_zpos != 0 || g_config.as5600_mpos != 0 || g_config.as5600_mang != 0)
                  ? (uint16_t)(g_config.as5600_zpos & 0x0FFF)
                  : (uint16_t)(AS5600_ZPOS & 0x0FFF);
    uint16_t mpos = (g_config.as5600_zpos != 0 || g_config.as5600_mpos != 0 || g_config.as5600_mang != 0)
                  ? (uint16_t)(g_config.as5600_mpos & 0x0FFF)
                  : (uint16_t)(AS5600_MPOS & 0x0FFF);
    uint16_t mang = (g_config.as5600_zpos != 0 || g_config.as5600_mpos != 0 || g_config.as5600_mang != 0)
                  ? (uint16_t)(g_config.as5600_mang & 0x0FFF)
                  : (uint16_t)(AS5600_MANG & 0x0FFF);
    as5600_writeReg16(AS5600_REG_ZPOS, zpos);
    as5600_writeReg16(AS5600_REG_MPOS, mpos);
    as5600_writeReg16(AS5600_REG_MANG, mang);
    Serial.printf_P(PSTR("[AS5600] Config applicata: ZPOS=%u MPOS=%u MANG=%u · offset_deg_x100=%d (%.2f°)\n"),
                     (unsigned)zpos, (unsigned)mpos, (unsigned)mang,
                     (int)g_config.as5600_offset_deg_100x, (float)g_config.as5600_offset_deg_100x / 100.0f);
}

// ============================================================================
// Applicazione RUNTIME della nuova config salvata via HTTP (dopo POST /api/config)
//   Evita il reboot per parametri non-WiFi e rendi effettive le modifiche SUBITO.
//   Safety: leggi g_config (già aggiornato prima di chiamare questa funzione).
// ============================================================================
static void config_applyRuntimeFromWeb(void)
{
    Serial.println(F("[CFG-RUNTIME] Applicazione parametri da EEPROM senza reboot ..."));
    if (g_safe_mode) { Serial.println(F("[CFG-RUNTIME] SAFE MODE: salvato in EEPROM, nessun accesso I2C")); return; }

    // --- 1. AS5600: ZPOS/MPOS/MANG/OFFSET (registri HW + offset letto direttamente nel loop) ---
    as5600_applyConfig();

    // --- 2. BME280: Altitudine e pressione livello mare (usati nel calcolo pressione relativa) ---
    //   bme280_init() riapplica la lettura di g_config.bme280_altitude_ref_m e
    //   g_config.bme280_sealevel_hpa. Esegue un soft reset interno del BME280
    //   per sicurezza (impiega ~5ms).
    bme280_init();

    // --- 3. SHT31: reinizializza il sensore (consistenza) ---
    sht31_init();

    // --- 4. LTR390: gain_id + res_it_id  →  applica senza SW-Reset invasivo ---
    {
        uint8_t gid = g_config.ltr390_gain_id;
        uint8_t rid = g_config.ltr390_res_it_id;
        if (gid > 4) gid = LTR390_GAIN_ID_DEFAULT;
        if (rid > 7) rid = LTR390_RES_IT_ID_DEFAULT;
        if (g_ltr390_gain_id != gid || g_ltr390_res_it_id != rid) {
            g_ltr390_gain_id = gid;
            g_ltr390_res_it_id = rid;
            Serial.printf_P(PSTR("[LTR390] Nuova config runtime: gain_id=%u res_it_id=%u\n"),
                             (unsigned)gid, (unsigned)rid);
            // Standby → scrivi i 2 registri → torna in modalità ALS attiva
            (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_MODE_STANDBY);
            delay(20);
            uint8_t rate = (uint8_t)(((LTR390_RES_IT_ID & 0x07) << 4) | (LTR390_MEAS_RATE & 0x07));
            (void)ltr390_writeReg8(LTR390_REG_MEAS_RATE, rate);
            (void)ltr390_writeReg8(LTR390_REG_GAIN,     (uint8_t)(LTR390_GAIN_ID & 0x0F));
            delay(10);
            // Riprendi l'ultimo stato attivo (ALS) + reset state machine
            (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, g_ltr390_mode_als_active);
            delay(10);
            g_ltr390_state    = LTR390_ST_WAIT_ALS_DRDY;
            g_ltr390_state_ts = millis();
            g_ltr390_last_als = 0;
            g_ltr390_last_uv  = 0;
            g_ltr390_als_new  = false;
            g_ltr390_uv_new   = false;
            Serial.println(F("[LTR390] Config runtime applicata (no SW-RST)"));
        }
    }

    // --- 5. Pluviometro: rain_dead_ms → aggiorna cache usata in ISR ---
    {
        uint16_t rd = g_config.rain_dead_ms;
        if (rd < RAIN_DEAD_MS_DEF) rd = RAIN_DEAD_MS_DEF;
        noInterrupts();
        g_rain_isr_dead_ms = (uint32_t)rd;
        interrupts();
        Serial.printf_P(PSTR("[RAIN] ISR dead-time aggiornato runtime: %u ms\n"), (unsigned)rd);
    }

    // --- 6. Anemometro Hall: parametri (magnets/radius/calib_k) usati nel loop direttamente ---
    //   g_config.hall_magnets_per_rev / hall_anemo_radius_m / hall_anemo_calib_k
    //   sono letti *direttamente* da g_config nel loop main (NESSUNA cache in
    //   globale separato), quindi il solo assegnamento g_config = next nella
    //   POST già li rende effettivi al prossimo giro.
    Serial.printf_P(PSTR("[HALL] Parametri runtime: magnets=%u radius_m=%.3f calib_k=%.2f\n"),
                     (unsigned)g_config.hall_magnets_per_rev,
                     g_config.hall_anemo_radius_m,
                     g_config.hall_anemo_calib_k);

    // --- 7. Calcolo derivato pioggia (mm/tip) → usato nel loop diretto, già aggiornato ---
    Serial.printf_P(PSTR("[RAIN] Calcoli runtime: diam_mm=%.1f mass_g=%.2f calib_k=%.2f\n"),
                     g_config.rain_funnel_dia_mm,
                     g_config.rain_tip_mass_g,
                     g_config.rain_calib_k);

    // --- 9. [NUOVO] Heading Filter: applica parametri runtime (window change → buffer reset) ---
    {
        HeadingFilterConfig_t hc = g_hdrf_cfg;
        hdrf_applyRuntime(&hc);
    }

    // --- 8. FLAG: forza prossima publish MQTT (A/B/C) con force=true ---
    //   Necessario se Δ < soglia on-change (es. offset heading +1.80°, ma soglia=2°):
    //   senza force=true non verrebbe pubblicato per 2+ secondi o per sempre.
    g_mqtt_force_repub_all = true;
    Serial.println(F("[MQTT] Force republish flag attivo → prossimo scheduler: publish completo"));
    Serial.println(F("[CFG-RUNTIME] Applicazione terminata."));
}

// ============================================================================
// Helper LTR390 I2C (Wire raw, senza libreria wrapper)
// ============================================================================
static uint8_t ltr390_writeReg8(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(LTR390_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission();
}

static uint8_t ltr390_readReg8(uint8_t reg, uint8_t *out)
{
    Wire.beginTransmission(LTR390_ADDR);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) return err;
    if (Wire.requestFrom((uint8_t)LTR390_ADDR, (uint8_t)1) != 1) return 0xFE;
    *out = (uint8_t)Wire.read();
    return 0;
}

static uint32_t ltr390_readReg20(uint8_t regLsb)
{
    Wire.beginTransmission(LTR390_ADDR);
    Wire.write(regLsb);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) return 0;
    if (Wire.requestFrom((uint8_t)LTR390_ADDR, (uint8_t)3) != 3) return 0;
    uint32_t lsb = (uint32_t)Wire.read();
    uint32_t msb = (uint32_t)Wire.read();
    uint32_t xtb = (uint32_t)Wire.read();
    return ((xtb & 0x0F) << 16) | (msb << 8) | lsb;
}

// Forward declarations
static bool ltr390_setMode(uint8_t wantMode, bool strictReadback = true);
static void ltr390_autoDetectActiveMode(void);
static void ltr390_strongRecovery(void);

static void ltr390_applyConfig(void)
{
    // Standby per sicurezza
    (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_MODE_STANDBY);
    delay(20);
    // Risoluzione/Integrazione + Measurement Rate
    uint8_t rate = (uint8_t)(((LTR390_RES_IT_ID & 0x07) << 4) | (LTR390_MEAS_RATE & 0x07));
    (void)ltr390_writeReg8(LTR390_REG_MEAS_RATE, rate);
    // Gain
    (void)ltr390_writeReg8(LTR390_REG_GAIN, (uint8_t)(LTR390_GAIN_ID & 0x0F));
    delay(10);
    // --- Write-Verify registri configurazione ---
    uint8_t rr = 0xFF, rg = 0xFF, rm = 0xFF;
    bool rate_ok = (ltr390_readReg8(LTR390_REG_MEAS_RATE, &rr) == 0) && (rr == rate);
    bool gain_ok = (ltr390_readReg8(LTR390_REG_GAIN,      &rg) == 0) && (rg == (uint8_t)(LTR390_GAIN_ID & 0x0F));
    bool main_ok = (ltr390_readReg8(LTR390_REG_MAIN_CTRL, &rm) == 0);
    char b[224];
    snprintf(b, sizeof(b),
      "[LTR390 INIT] MEAS_RATE want=0x%02X got=0x%02X(%c) | "
      "GAIN want=0x%02X got=0x%02X(%c) | "
      "MAIN_STBY got=0x%02X(%c) [actives ALS=0x06/UVS=0x0E]",
      rate, rr, rate_ok?'*':'!',
      (unsigned)(LTR390_GAIN_ID & 0x0F), rg, gain_ok?'*':'!',
      rm, main_ok?'*':'!');
    Serial.println(b);
    // ---- [FSM INIT] Continuous mode, NO standby cycling (pattern verificato su HW) ----
    // Prima: MAIN_CTRL=0x06 (RIS2=1+enable+ALS) scritto una volta produceva valori.
    // Adesso: stessa filosofia con switch modale ALS↔UVS cambiando solo bit3,
    //         SENZA passare per standby tra ALS e UVS.
    //         g_ltr390_state = WAIT_ALS_DRDY + MAIN_CTRL = ALS_ACTIVE 0x06.
    // ---- [PRIMING 3x] Alcuni clone/chip hanno bisogno di write multipli prima
    //      che il latch bit2 si attivi davvero. Facciamo 3 write con delay crescenti.
    {
      const uint8_t mask = 0x0E;
      uint8_t got = 0xFF;
      (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, g_ltr390_mode_als_active);
      delay(10);
      (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, g_ltr390_mode_als_active);
      delay(15);
      (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, g_ltr390_mode_als_active);
      delay(20);
      bool primed = (ltr390_readReg8(LTR390_REG_MAIN_CTRL, &got) == 0) && ((got & mask) == (g_ltr390_mode_als_active & mask));
      // Aggiorna stats wv per tenere traccia (3 write totali):
      g_ltr390_wv_try += 3;
      g_ltr390_wv_rdy += (primed ? 3 : 2);
      g_ltr390_wv_fail += (primed ? 0 : 1);
      g_ltr390_last_wv_want = g_ltr390_mode_als_active;
      g_ltr390_last_wv_got = got;
      Serial.print(F("[LTR390 INIT] PRIMING 3x ALS want=0x06 got=0x"));
      Serial.print(got, HEX);
      Serial.println(primed ? F("* MATCH") : F(" MISMATCH (continuo comunque)"));
    }
    g_ltr390_state    = LTR390_ST_WAIT_ALS_DRDY;
    g_ltr390_state_ts = millis();
    g_ltr390_last_als = 0;
    g_ltr390_last_uv  = 0;
    g_ltr390_als_new  = false;
    g_ltr390_uv_new   = false;
    // Fallback hybrid: inizialmente 0
    g_ltr390_hybrid_next = 0;
    g_ltr390_hybrid_count = 0;
    // Azzeramento statistiche wv (sovrascritte da PRIMING 3x contano)
    g_ltr390_start_als_count = 1;
    g_ltr390_start_uvs_count = 0;
}

static bool ltr390_init(void)
{
    uint8_t part = 0;
    if (ltr390_readReg8(LTR390_REG_PART_ID, &part) != 0) {
        Serial.println(F("LTR390: I2C read fallito"));
        return false;
    }
    if (part != 0xB2) {
        Serial.print(F("LTR390: PART_ID non atteso 0x"));
        Serial.print(part, HEX);
        Serial.println(F(" (atteso 0xB2)"));
        return false;
    }
    // [H19 FIX] SW Reset (Main Control Register bit4)
    // --- Fonte: LiteON LTR-390UV Datasheet sez 6.1 MAIN_CTRL + Adafruit library reset() ---
    // Prima di qualsiasi configurazione, assicurarsi che TUTTI i registri siano
    // portati ai valori di default (elimina latch residui delle iterazioni
    // precedenti, soprattutto dopo un WDT o un reset software ESP8266 senza
    // power-cycle). Scriviamo 0x10 (bit4 SW_Reset=1, gli altri 0 per sicurezza).
    // Il reset si auto-cancella dopo ~5ms (datasheet sez 10.2 Electrical specs).
    ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_SW_RESET);
    delay(20);
    // Dopo SW Reset, torniamo in standby esplicito (per sicurezza) e attendiamo
    // che l'I2C torni disponibile.
    ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_MODE_STANDBY);
    delay(10);
    {
      // ISTRUMENTAZIONE: stampa risultato reset
      uint8_t mc = 0xFF;
      (void)ltr390_readReg8(LTR390_REG_MAIN_CTRL, &mc);
      Serial.print(F("[LTR390 INIT] post-SW-Reset MAIN_CTRL=0x")); Serial.print(mc, HEX);
      Serial.print(F(" (expect STBY 0x00, RES bits non-zero ammessi)"));
      Serial.println();
    }
    ltr390_applyConfig();
    // --- AUTO-DETECT modalità MAIN_CTRL attiva funzionante ---
    // Prova 0x02,0x06,0x0A,0x0E e salva la prima che produce ALS_DRDY in 2s.
    // Sovrascrive g_ltr390_mode_als_active e g_ltr390_mode_uvs_active.
    ltr390_autoDetectActiveMode();
    // Dopo auto-detect: forza als_mode non strict (per chip con read-back fallato)
    (void)ltr390_setMode(g_ltr390_mode_als_active, /*strictReadback=*/false);
    g_ltr390_state = LTR390_ST_WAIT_ALS_DRDY;
    g_ltr390_state_ts = millis();
    // [FIX F4] Clear DRDY flags sporchi post-power-up / reset.
    // Datasheet LTR390 sez MAIN_STATUS (0x07): DRDY bit si cancella SOLAMENTE
    // con lettura ALS_DATA (0x0D-0x0F) o UVS_DATA (0x10-0x12), non con standby!
    // Senza questa clear la state machine rimane latchata ad aspettare il
    // "WRONG DRDY" (es. MAIN_STATUS=0x08 UVS_DRDY vecchio mentre wait ALS).
    (void)ltr390_readReg20(LTR390_REG_ALS_DATA_LSB);
    (void)ltr390_readReg20(LTR390_REG_UVS_DATA_LSB);
    delay(2);
    uint8_t msPostClear = 0xFF;
    (void)ltr390_readReg8(LTR390_REG_MAIN_STATUS, &msPostClear);
    g_ltr390_boot_ts = millis();
    g_ltr390_recovery_count = 0;
    Serial.print(F("LTR390: OK @ 0x53 (PART_ID=0xB2) switch modale ALS↔UVS  MAIN_STATUS post-clear=0x"));
    Serial.println(msPostClear, HEX);
    Serial.print(F("[LTR390 INIT] FINAL als_mode=0x")); Serial.print(g_ltr390_mode_als_active, HEX);
    Serial.print(F(" uvs_mode=0x")); Serial.print(g_ltr390_mode_uvs_active, HEX);
    Serial.print(F(" boot_ts=")); Serial.println((unsigned long)g_ltr390_boot_ts);
    return true;
}

static bool ltr390_setMode(uint8_t wantMode, bool strictReadback)
{
    g_ltr390_wv_try++;
    g_ltr390_last_wv_want = wantMode;
    const uint8_t mask = 0x0E;
    uint8_t wErr = ltr390_writeReg8(LTR390_REG_MAIN_CTRL, wantMode);
    if (wErr != 0) {
        g_ltr390_wv_fail++;
        g_ltr390_last_wv_got = 0xFE;
        return false;
    }
    delay(5);
    uint8_t got = 0xFF;
    if (ltr390_readReg8(LTR390_REG_MAIN_CTRL, &got) != 0) {
        // Fallback non-strict: write ha avuto ACK ma read fallisce → OK
        if (!strictReadback) {
            g_ltr390_wv_rdy++;
            g_ltr390_last_wv_got = 0xFD;
            return true;
        }
        g_ltr390_wv_fail++;
        g_ltr390_last_wv_got = 0xFD;
        return false;
    }
    g_ltr390_last_wv_got = got;
    if ((got & mask) == (wantMode & mask)) {
        g_ltr390_wv_rdy++;
        return true;
    }
    // Read-back non matcha: se non strict, consideriamo successo
    if (!strictReadback) {
        g_ltr390_wv_rdy++;
        return true;
    }
    g_ltr390_wv_fail++;
    return false;
}

// ============================================================================
// AUTO-DETECT ACTIVE MODE: prova TUTTE le 4 combinazioni MAIN_CTRL attivo,
// salva la prima che produce ALS_DRDY in 2s. Nessuna assunzione su chip/clone.
// ============================================================================
static void ltr390_autoDetectActiveMode(void)
{
    // 4 possibili pattern MAIN_CTRL attivo (bit1 sempre 1; bit2 e bit3 variabili)
    // bit1=1(enable), bit2=reserved latch, bit3=UVS_Mode. Per ALS testiamo bit3=0.
    const uint8_t als_candidates[4] = {0x02, 0x06, 0x0A, 0x0E};  // bit3=0
    uint8_t best_als = 0xFF;
    Serial.println(F("[LTR390 AUTO-DETECT] Mode scan: ALS candidates 0x02,0x06,0x0A,0x0E"));
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t want = als_candidates[i];
        // Clear DRDY
        (void)ltr390_readReg20(LTR390_REG_ALS_DATA_LSB);
        (void)ltr390_readReg20(LTR390_REG_UVS_DATA_LSB);
        // Write: NON strict readback (chi ha readback buggato può comunque produrre DRDY)
        (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, want);
        delay(5);
        unsigned long ts = millis();
        bool drdy = false;
        uint8_t st = 0;
        // Poll 2s attesa DRDY
        while (millis() - ts < 2000) {
            if (ltr390_readReg8(LTR390_REG_MAIN_STATUS, &st) == 0 && (st & LTR390_ST_ALS_DRDY)) {
                uint32_t als = ltr390_readReg20(LTR390_REG_ALS_DATA_LSB);
                Serial.print(F("[LTR390 AUTO-DETECT] cand=0x"));
                Serial.print(want, HEX);
                Serial.print(F(" DRDY! MAIN_STATUS=0x"));
                Serial.print(st, HEX);
                Serial.print(F(" als_raw=")); Serial.println(als);
                // 0 è consentito in dark; se DRDY matcha comunque OK.
                drdy = true;
                best_als = want;
                break;
            }
            delay(10);
        }
        (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_MODE_STANDBY);
        delay(2);
        if (drdy) {
            g_ltr390_mode_als_active = best_als;
            // UVS corrispondente = stessi bit (con eventuale bit2) + bit3=1 UVS_Mode
            g_ltr390_mode_uvs_active = (uint8_t)(best_als | 0x08);
            Serial.print(F("[LTR390 AUTO-DETECT] SELECTED als_mode=0x"));
            Serial.print(g_ltr390_mode_als_active, HEX);
            Serial.print(F(" uvs_mode=0x"));
            Serial.println(g_ltr390_mode_uvs_active, HEX);
            return;
        }
        Serial.print(F("[LTR390 AUTO-DETECT] cand=0x"));
        Serial.print(want, HEX);
        Serial.println(F(" → no DRDY in 2s"));
    }
    // Nessuna modalità: mantieni default 0x06/0x0E e logga warning
    Serial.print(F("[LTR390 AUTO-DETECT] NESSUNO DRDY! fallback default als=0x"));
    Serial.print(g_ltr390_mode_als_active, HEX);
    Serial.print(F(" uvs=0x"));
    Serial.println(g_ltr390_mode_uvs_active, HEX);
}

// ============================================================================
// AUTO-RECOVERY STRONG: se dopo 15s als_cnt == 0 → SW Reset + Wire restart
// + PRIMING 10x write alternati als_mode e uvs_mode per scardinare latch.
// ============================================================================
static void ltr390_strongRecovery(void)
{
    Serial.print(F("[LTR390 STRONG-RECOVERY] #")); Serial.print(++g_ltr390_recovery_count);
    Serial.print(F(" als_cnt=0 after ")); Serial.print((unsigned long)(millis()-g_ltr390_boot_ts));
    Serial.println(F("ms → SW Reset + Wire restart + PRIMING 10x"));
    // 1. Wire soft-restart (ESP8266 Wire 3.30102 NON ha .end(); usiamo workaround)
    pinMode(D2, INPUT_PULLUP);
    pinMode(D1, INPUT_PULLUP);
    delay(5);
    Wire.begin(D2, D1);  // ESP8266 standard I2C pins: SDA=D2(GPIO4), SCL=D1(GPIO5)
    Wire.setClock(I2C_CLOCK_HZ);
    delay(10);
    // 2. SW Reset chip
    (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_SW_RESET);
    delay(25);
    (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_MODE_STANDBY);
    delay(10);
    // 3. Clear DRDY throw-away reads
    (void)ltr390_readReg20(LTR390_REG_ALS_DATA_LSB);
    (void)ltr390_readReg20(LTR390_REG_UVS_DATA_LSB);
    delay(2);
    // 4. PRIMING 10x alternati als/uvs write
    for (uint8_t i = 0; i < 10; i++) {
        uint8_t w = (i % 2 == 0) ? g_ltr390_mode_als_active : g_ltr390_mode_uvs_active;
        (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, w);
        delay(30);
    }
    // 5. Ultimo write = als_mode non strict
    (void)ltr390_setMode(g_ltr390_mode_als_active, /*strictReadback=*/false);
    // 6. Reset state machine e statistiche
    g_ltr390_state = LTR390_ST_WAIT_ALS_DRDY;
    g_ltr390_state_ts = millis();
    g_ltr390_als_cnt = 0;
    g_ltr390_uv_cnt  = 0;
    g_ltr390_hybrid_next = 1;   // Forza hybrid per il primo switch
    g_ltr390_boot_ts = millis();  // Reset window 15s per prossimo recovery
    // 7. [CRITICAL FIX SHT31] Wire.begin() duplicata sopra ha RESETTATO il bus I2C.
    //    Gli altri sensori sul bus (SHT31, BME280, AS5600) potrebbero essere in lockup
    //    perché la sequenza di clock è stata interrotta. Re-inizializziamo TUTTI.
    Serial.println(F("[LTR390 STRONG-RECOVERY] Post-Wire-begin → REINIT bus I2C: BME280 + SHT31"));
    (void)i2c_busRecover();
    delay(3);
    bme280_init();
    delay(2);
    sht31_init();
}

// State machine non bloccante LTR390.
//   Avanza ciclicamente: START_ALS → WAIT_ALS_DRDY → START_UVS → WAIT_UVS_DRDY → …
//   Da chiamare almeno ogni 10ms nel loop principale. Aggiorna g_ltr390_last_als /
//   g_ltr390_last_uv e imposta flag als_new / uv_new quando arrivano valori freschi.
static void ICACHE_FLASH_ATTR ltr390_poll(void)
{
    // ISTRUMENTAZIONE: conta chiamate poll TOTALI (anche prima di early return)
    g_ltr390_poll_calls++;
    if (!g_ltr390_ok) return;
    unsigned long now = millis();
    uint8_t st = 0;
    // --- Diagnostica throttled LTR390 ogni 5s ---
    static unsigned long lastDiagMs = 0;
    if ((now - lastDiagMs) >= 5000) {
        lastDiagMs = now;
        uint8_t mc=0xFF, ms=0xFF, gn=0xFF, mr=0xFF;
        bool mc_ok = (ltr390_readReg8(LTR390_REG_MAIN_CTRL,  &mc) == 0);
        bool ms_ok = (ltr390_readReg8(LTR390_REG_MAIN_STATUS, &ms) == 0);
        bool gn_ok = (ltr390_readReg8(LTR390_REG_GAIN,        &gn) == 0);
        bool mr_ok = (ltr390_readReg8(LTR390_REG_MEAS_RATE,   &mr) == 0);
        char b[288];
        snprintf(b, sizeof(b),
          "[LTR390 DIAG] ok=%d st=%u(%s) age=%lums | poll=%lu hb=%lu | als/uv_cnt=%lu/%lu  last_als/uv=%lu/%lu | "
          "MAIN_CTRL=0x%02X(%c) MAIN_STATUS=0x%02X(%c) | GAIN=0x%02X(%c) MEAS_RATE=0x%02X(%c) | "
          "WV want=0x%02X got=0x%02X  try=%lu ok=%lu fail=%lu | start_als/uvs=%lu/%lu",
          (int)g_ltr390_ok,
          (unsigned)g_ltr390_state, ltr390_state_name(g_ltr390_state),
          (unsigned long)(now - g_ltr390_state_ts),
          (unsigned long)g_ltr390_poll_calls, (unsigned long)g_ltr390_hybrid_count,
          (unsigned long)g_ltr390_als_cnt, (unsigned long)g_ltr390_uv_cnt,
          (unsigned long)g_ltr390_last_als, (unsigned long)g_ltr390_last_uv,
          mc, mc_ok?'*':'!', ms, ms_ok?'*':'!',
          gn, gn_ok?'*':'!', mr, mr_ok?'*':'!',
          (unsigned)g_ltr390_last_wv_want, (unsigned)g_ltr390_last_wv_got,
          (unsigned long)g_ltr390_wv_try, (unsigned long)g_ltr390_wv_rdy, (unsigned long)g_ltr390_wv_fail,
          (unsigned long)g_ltr390_start_als_count, (unsigned long)g_ltr390_start_uvs_count);
        Serial.println(b);
    }
    // --- AUTO-RECOVERY STRONG dopo 15s se als_cnt=0 e meno di 2 recovery già fatti ---
    if (g_ltr390_als_cnt == 0 && g_ltr390_recovery_count < 2 && (now - g_ltr390_boot_ts) > 15000) {
        ltr390_strongRecovery();
    }
    unsigned long timeout_ms;
    switch (LTR390_RES_IT_ID) {
        case 0: timeout_ms = 400 + 50; break;  // 400ms
        case 1: timeout_ms = 200 + 40; break;  // 200ms (default indoor)
        case 2: timeout_ms = 100 + 25; break;  // 100ms
        case 3: timeout_ms =  50 + 15; break;
        case 4: timeout_ms =  25 + 10; break;
        case 5: timeout_ms =  25 + 10; break;
        case 6: timeout_ms =  50 + 15; break;
        case 7: timeout_ms = 100 + 25; break;
        default: timeout_ms = 250; break;
    }
    // [FIX 20B/C] Continuous mode NO standby cycling tra ALS e UVS:
    // Startup wait rimosso; switch diretto WAIT_ALS_DRDY ↔ WAIT_UVS_DRDY cambiando
    // solo il bit3 UVS_Mode del MAIN_CTRL (0x06 ↔ 0x0E).
    // Hybrid next: se 1, passa per WARMUP_X (STANDBY delay 8ms millis() NON bloccante)
    // prima di avviare la nuova modalità.

    switch (g_ltr390_state) {
        case LTR390_ST_IDLE_START_ALS:
            // Fallback: avanza SEMPRE (no gating setMode). Stats wv in DIAG.
            (void)ltr390_setMode(g_ltr390_mode_als_active, true);
            g_ltr390_start_als_count++;
            g_ltr390_state_ts = now;
            g_ltr390_state    = LTR390_ST_WAIT_ALS_DRDY;
            break;

        case LTR390_ST_WAIT_ALS_DRDY: {
            bool drdy = (ltr390_readReg8(LTR390_REG_MAIN_STATUS, &st) == 0 && (st & LTR390_ST_ALS_DRDY));
            unsigned long state_age = now - g_ltr390_state_ts;
            if ((!drdy) && (now - g_ltr390_forced_read_als_ts >= LTR390_FORCED_READ_MS)) {
                uint32_t v = ltr390_readReg20(LTR390_REG_ALS_DATA_LSB);
                bool integ_pass = (state_age >= timeout_ms);
                bool val_ok     = (v != g_ltr390_last_als) || (v != 0) || integ_pass;
                if (val_ok) {
                    g_ltr390_last_als = v;
                    g_ltr390_als_new  = true;
                    g_ltr390_als_cnt++;
                    drdy = true;
                }
                g_ltr390_forced_read_als_ts = now;
            }
            if (drdy) {
                uint32_t v = ltr390_readReg20(LTR390_REG_ALS_DATA_LSB);
                bool integ_pass = (state_age >= timeout_ms);
                if (v != 0 || (st & LTR390_ST_ALS_DRDY) || integ_pass) {
                    g_ltr390_last_als = v;
                    g_ltr390_als_new  = true;
                    g_ltr390_als_cnt++;
                }
                if (g_ltr390_hybrid_next) {
                    g_ltr390_hybrid_next = 0;
                    (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_MODE_STANDBY);
                    g_ltr390_state_ts = now;
                    g_ltr390_state    = LTR390_ST_WARMUP_UVS_FROM_ALS;
                } else {
                    (void)ltr390_setMode(g_ltr390_mode_uvs_active, false);
                    g_ltr390_start_uvs_count++;
                    g_ltr390_state_ts = now;
                    g_ltr390_state    = LTR390_ST_WAIT_UVS_DRDY;
                }
            } else if (state_age > (timeout_ms + 3000)) {
                Serial.print(F("[LTR390] ALS DRDY TIMEOUT after "));
                Serial.print(state_age);
                Serial.print(F("ms MAIN_STATUS=0x")); Serial.print(st, HEX);
                Serial.println(F(" → recovery + HYBRID NEXT"));
                g_ltr390_hybrid_next = 1;
                g_ltr390_hybrid_count++;
                (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_MODE_STANDBY);
                g_ltr390_state_ts = now;
                g_ltr390_state    = LTR390_ST_WD_RECOVERY_10MS;
            }
            break;
        }

        case LTR390_ST_WARMUP_UVS_FROM_ALS:
            // 8ms attesa STANDBY prima di UVS
            if ((now - g_ltr390_state_ts) >= 8) {
                (void)ltr390_setMode(g_ltr390_mode_uvs_active, false);
                g_ltr390_start_uvs_count++;
                g_ltr390_state_ts = now;
                g_ltr390_state    = LTR390_ST_WAIT_UVS_DRDY;
            }
            break;

        case LTR390_ST_WD_RECOVERY_10MS:
            // 10ms dopo STANDBY nel watchdog recovery
            if ((now - g_ltr390_state_ts) >= 10) {
                (void)ltr390_readReg20(LTR390_REG_ALS_DATA_LSB);
                (void)ltr390_readReg20(LTR390_REG_UVS_DATA_LSB);
                g_ltr390_state_ts = now;
                g_ltr390_state    = LTR390_ST_WD_RECOVERY_2MS; // 2ms gap
            }
            break;

        case LTR390_ST_WD_RECOVERY_2MS:
            if ((now - g_ltr390_state_ts) >= 2) {
                (void)ltr390_setMode(g_ltr390_mode_als_active, false);
                g_ltr390_state_ts = now;
                g_ltr390_state    = LTR390_ST_WAIT_ALS_DRDY;
            }
            break;

        case LTR390_ST_IDLE_START_UVS:
            (void)ltr390_setMode(g_ltr390_mode_uvs_active, true);
            g_ltr390_start_uvs_count++;
            g_ltr390_state_ts = now;
            g_ltr390_state    = LTR390_ST_WAIT_UVS_DRDY;
            break;

        case LTR390_ST_WAIT_UVS_DRDY: {
            bool drdy = (ltr390_readReg8(LTR390_REG_MAIN_STATUS, &st) == 0 && (st & LTR390_ST_UVS_DRDY));
            unsigned long state_age = now - g_ltr390_state_ts;
            if ((!drdy) && (now - g_ltr390_forced_read_uvs_ts >= LTR390_FORCED_READ_MS)) {
                uint32_t v = ltr390_readReg20(LTR390_REG_UVS_DATA_LSB);
                bool integ_pass = (state_age >= timeout_ms);
                bool val_ok     = (v != g_ltr390_last_uv) || (v != 0) || integ_pass;
                if (val_ok) {
                    g_ltr390_last_uv = v;
                    g_ltr390_uv_new  = true;
                    g_ltr390_uv_cnt++;
                    drdy = true;
                }
                g_ltr390_forced_read_uvs_ts = now;
            }
            if (drdy) {
                uint32_t v = ltr390_readReg20(LTR390_REG_UVS_DATA_LSB);
                bool integ_pass = (state_age >= timeout_ms);
                if (v != 0 || (st & LTR390_ST_UVS_DRDY) || integ_pass) {
                    g_ltr390_last_uv = v;
                    g_ltr390_uv_new  = true;
                    g_ltr390_uv_cnt++;
                }
                if (g_ltr390_hybrid_next) {
                    g_ltr390_hybrid_next = 0;
                    (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_MODE_STANDBY);
                    g_ltr390_state_ts = now;
                    g_ltr390_state    = LTR390_ST_WARMUP_ALS_FROM_UVS;
                } else {
                    (void)ltr390_setMode(g_ltr390_mode_als_active, false);
                    g_ltr390_start_als_count++;
                    g_ltr390_state_ts = now;
                    g_ltr390_state    = LTR390_ST_WAIT_ALS_DRDY;
                }
            } else if (state_age > (timeout_ms + 3000)) {
                Serial.print(F("[LTR390] UVS DRDY TIMEOUT after "));
                Serial.print(state_age);
                Serial.print(F("ms MAIN_STATUS=0x")); Serial.print(st, HEX);
                Serial.println(F(" → recovery + HYBRID NEXT"));
                g_ltr390_hybrid_next = 1;
                g_ltr390_hybrid_count++;
                (void)ltr390_writeReg8(LTR390_REG_MAIN_CTRL, LTR390_MODE_STANDBY);
                g_ltr390_state_ts = now;
                g_ltr390_state    = LTR390_ST_WD_RECOVERY_10MS;
            }
            break;
        }

        case LTR390_ST_WARMUP_ALS_FROM_UVS:
            if ((now - g_ltr390_state_ts) >= 8) {
                (void)ltr390_setMode(g_ltr390_mode_als_active, false);
                g_ltr390_start_als_count++;
                g_ltr390_state_ts = now;
                g_ltr390_state    = LTR390_ST_WAIT_ALS_DRDY;
            }
            break;
    }
}

// Restituisce gli ultimi valori raw disponibili e (se passati non-NULL) i gain/res id
// letti direttamente dai registri di configurazione. Flag new_als / new_uv = true se
// il valore corrispondente è stato aggiornato dall'ultima chiamata a read().
static bool ltr390_read(uint32_t *als, uint32_t *uv,
                        uint8_t *gainId, uint8_t *resId,
                        bool *new_als, bool *new_uv)
{
    if (als)    *als    = g_ltr390_last_als;
    if (uv)     *uv     = g_ltr390_last_uv;
    if (new_als){ *new_als = g_ltr390_als_new; g_ltr390_als_new = false; }
    if (new_uv) { *new_uv  = g_ltr390_uv_new;  g_ltr390_uv_new  = false; }
    uint8_t g = 0, r = 0;
    if (gainId) {
        if (ltr390_readReg8(LTR390_REG_GAIN, &g) != 0) return false;
        *gainId = (uint8_t)(g & 0x0F);
    }
    if (resId) {
        if (ltr390_readReg8(LTR390_REG_MEAS_RATE, &r) != 0) return false;
        *resId = (uint8_t)((r >> 4) & 0x07);
    }
    return true;
}

// ============================================================================
// Helper CRC8 SHT31-D · Sensirion polinomio 0x31, init 0xFF (datasheet 4.11)
// ============================================================================
static uint8_t sht31_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
        }
    }
    return crc;
}

// ============================================================================
// Driver I2C raw SHT31-D (no wrapper)
// ============================================================================
static uint8_t sht31_writeCmd16(uint16_t cmd)
{
    Wire.beginTransmission(SHT31_ADDR);
    Wire.write((uint8_t)((cmd >> 8) & 0xFF));
    Wire.write((uint8_t)(cmd & 0xFF));
    uint8_t err = Wire.endTransmission(true);
    if (err != 0) g_sht31_i2c_error_count++;
    return err;
}

static uint8_t sht31_softReset(void)
{
    uint8_t e = sht31_writeCmd16(SHT31_CMD_SOFT_RESET);
    delay(2);
    return e;
}

static uint8_t sht31_clearStatus(void)
{
    return sht31_writeCmd16(SHT31_CMD_CLEAR_STATUS);
}

// Legge status register 16-bit + CRC8. Ritorna 0 se ok, altrimenti errore Wire
static uint8_t sht31_readStatus(uint16_t *status16)
{
    if (sht31_writeCmd16(SHT31_CMD_READ_STATUS) != 0) return 0xFE;
    if (Wire.requestFrom((uint8_t)SHT31_ADDR, (uint8_t)3) != 3) {
        g_sht31_i2c_error_count++;
        return 0xFD;
    }
    uint8_t buf[3];
    buf[0] = (uint8_t)Wire.read();
    buf[1] = (uint8_t)Wire.read();
    buf[2] = (uint8_t)Wire.read();
    uint8_t crcGot = sht31_crc8(buf, 2);
    if (crcGot != buf[2]) {
        g_sht31_crc_error_count++;
        return 0xFC;
    }
    if (status16) *status16 = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    return 0;
}

static uint8_t sht31_setHeater(bool on)
{
    return sht31_writeCmd16(on ? SHT31_CMD_HEATER_ENABLE : SHT31_CMD_HEATER_DISABLE);
}

// Single-Shot High Repeatability, No Clock Stretch (0x2400).
// Attesa conv ~15ms, 6 byte restituiti: [Tmsb,Tlsb,Tcrc, RHmsb,RHlsb,RHcrc]
static uint8_t sht31_singleShot(float *tC, float *rh)
{
    if (sht31_writeCmd16(SHT31_CMD_SS_HIGH_NOCSB) != 0) return 0xFE;
    unsigned long t0 = micros();
    unsigned long maxWaitUs = (SHT31_CONV_MAX_MS * 1000UL) + 500UL;
    bool ready = false;
    while ((micros() - t0) < maxWaitUs) {
        if (Wire.requestFrom((uint8_t)SHT31_ADDR, (uint8_t)6) == 6) {
            ready = true;
            break;
        }
        delayMicroseconds(200);
    }
    if (!ready) {
        g_sht31_i2c_error_count++;
        return 0xFD;
    }
    uint8_t buf[6];
    for (uint8_t i=0; i<6; i++) buf[i] = (uint8_t)Wire.read();

    uint8_t crcT  = sht31_crc8(&buf[0], 2);
    uint8_t crcRH = sht31_crc8(&buf[3], 2);
    bool crcOk = (crcT == buf[2]) && (crcRH == buf[5]);
    if (!crcOk) {
        g_sht31_crc_error_count++;
        return 0xFC;
    }

    uint16_t rawT  = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    uint16_t rawRH = (uint16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[4]);
    if (tC) *tC = -45.0f + 175.0f * ((float)rawT / 65535.0f);
    if (rh) *rh = 100.0f * ((float)rawRH / 65535.0f);
    return 0;
}

// Heater FSM con anti-ciclismo: min on-time, min off-time, isteresi 3% (98% on / 95% off)
static void sht31_heater_fsm_step(float rh_current)
{
    unsigned long now = millis();
    unsigned long stateAge = now - g_sht31_heater_ts;
    switch (g_sht31_heater_state) {
    case SHT31_HTR_OFF:
        if (isfinite(rh_current) && rh_current >= SHT31_HEATER_RH_ON) {
            if (stateAge >= SHT31_HEATER_MIN_OFF_MS || g_sht31_heater_ts == 0) {
                sht31_setHeater(true);
                g_sht31_heater_state = SHT31_HTR_WARMUP;
                g_sht31_heater_ts = now;
                g_sht31_heater_cycle_count++;
                Serial.print(F("[SHT31] HEATER ON (RH=")); Serial.print(rh_current, 1);
                Serial.print(F("% ≥ ")); Serial.print(SHT31_HEATER_RH_ON, 0);
                Serial.println(F("%)"));
            }
        }
        break;
    case SHT31_HTR_WARMUP:
        if (stateAge >= SHT31_HEATER_MIN_ON_MS) {
            g_sht31_heater_state = SHT31_HTR_ON;
        }
        break;
    case SHT31_HTR_ON:
        if (isfinite(rh_current) && rh_current <= SHT31_HEATER_RH_OFF) {
            sht31_setHeater(false);
            g_sht31_heater_state = SHT31_HTR_COOLDOWN;
            g_sht31_heater_ts = now;
            Serial.print(F("[SHT31] HEATER OFF (RH=")); Serial.print(rh_current, 1);
            Serial.print(F("% ≤ ")); Serial.print(SHT31_HEATER_RH_OFF, 0);
            Serial.println(F("%)"));
        } else if (stateAge >= 600000UL) {
            sht31_setHeater(false);
            g_sht31_heater_state = SHT31_HTR_COOLDOWN;
            g_sht31_heater_ts = now;
            Serial.println(F("[SHT31] HEATER timeout max 10min safety OFF"));
        }
        break;
    case SHT31_HTR_COOLDOWN:
        if (stateAge >= SHT31_HEATER_MIN_OFF_MS) {
            g_sht31_heater_state = SHT31_HTR_OFF;
            g_sht31_heater_ts = now;
        }
        break;
    default:
        g_sht31_heater_state = SHT31_HTR_OFF;
        g_sht31_heater_ts = now;
        break;
    }
    g_snap_sht31_htr_on = (g_sht31_heater_state == SHT31_HTR_WARMUP ||
                           g_sht31_heater_state == SHT31_HTR_ON);
}

static const char* sht31_wireErrStr(uint8_t err)
{
    switch (err) {
        case 0: return "OK";
        case 1: return "DATA_TOO_LONG";
        case 2: return "NACK_ADDR";
        case 3: return "NACK_DATA";
        case 4: return "OTHER_ERR";
        case 0xFE: return "CMD_WRITE_FAIL";
        default:  return "UNKNOWN";
    }
}

static void sht31_init(void)
{
    g_sht31_ok = false;
    g_sht31_heater_state = SHT31_HTR_OFF;
    g_sht31_heater_ts = millis();
    g_sht31_crc_error_count = 0;
    g_sht31_i2c_error_count = 0;

    Serial.print(F("[SHT31] Init I2C 0x"));
    Serial.print(SHT31_ADDR, HEX);
    Serial.println(F(" · driver raw + CRC8 0x31 · 3 tentativi con bus-recovery intermedio"));

    const uint8_t MAX_TRY = 3;
    float tTest = NAN, rhTest = NAN;
    uint8_t lastErr = 0xFF;
    uint16_t sr = 0;

    for (uint8_t attempt = 1; attempt <= MAX_TRY; attempt++)
    {
        Serial.print(F("[SHT31]  tentativo #")); Serial.print(attempt);
        Serial.print(F("/")); Serial.print(MAX_TRY); Serial.print(F(" : "));

        if (attempt > 1) {
            (void)i2c_busRecover();
            delay(5);
            (void)sht31_clearStatus();
            delay(2);
        }

        uint8_t e_rst = sht31_writeCmd16(SHT31_CMD_SOFT_RESET);
        if (e_rst != 0) {
            Serial.print(F("SOFT_RESET FAIL wire_err=")); Serial.print(e_rst);
            Serial.print(F("(")); Serial.print(sht31_wireErrStr(e_rst)); Serial.print(F(")"));
            lastErr = e_rst;
            Serial.println();
            continue;
        }
        delay(2);
        (void)sht31_clearStatus();
        delay(1);

        Serial.print(F("RST→OK · "));

        uint8_t r = sht31_singleShot(&tTest, &rhTest);
        if (r != 0) {
            Serial.print(F("SINGLE_SHOT FAIL err=0x")); Serial.print(r, HEX);
            Serial.print(F("(")); Serial.print(sht31_wireErrStr(r)); Serial.print(F(")"));
            lastErr = r;
            Serial.println();
            continue;
        }

        if (!isfinite(tTest) || !isfinite(rhTest) || tTest < -50.0f || tTest > 150.0f ||
            rhTest < -5.0f || rhTest > 105.0f) {
            Serial.print(F("VALORI OUT-OF-RANGE T=")); Serial.print((double)tTest);
            Serial.print(F(" RH=")); Serial.print((double)rhTest);
            lastErr = 0xFD;
            Serial.println();
            continue;
        }

        (void)sht31_readStatus(&sr);
        Serial.print(F("T=")); Serial.print((double)tTest, 2);
        Serial.print(F("°C RH=")); Serial.print((double)rhTest, 2);
        Serial.print(F("% SR=0x")); Serial.print(sr, HEX);
        Serial.print(F(" → "));
        g_sht31_ok = true;
        break;
    }

    if (g_sht31_ok) {
        char bl[160];
        snprintf(bl, sizeof(bl),
            "[SHT31] OK · T=%.2f °C · RH=%.2f %% · StatusReg=0x%04X · CRCerr=%lu · I2Cerr=%lu",
            (double)tTest, (double)rhTest, (unsigned)sr,
            (unsigned long)g_sht31_crc_error_count,
            (unsigned long)g_sht31_i2c_error_count);
        Serial.println(bl);
    } else {
        Serial.print(F("[SHT31] FALLITO dopo ")); Serial.print(MAX_TRY);
        Serial.print(F(" tentativi. Ultimo errore=0x")); Serial.print(lastErr, HEX);
        Serial.print(F("(")); Serial.print(sht31_wireErrStr(lastErr));
        Serial.println(F(") · Dispositivo non raggiungibile o cablaggio I2C errato"));
        Serial.print(F("[SHT31] DIAG: Indirizzo I2C usato=0x")); Serial.print(SHT31_ADDR, HEX);
        Serial.print(F(" SDA=GPIO")); Serial.print(I2C_SDA_PIN);
        Serial.print(F(" SCL=GPIO")); Serial.print(I2C_SCL_PIN);
        Serial.print(F(" Clock=")); Serial.print(100000UL); Serial.println(F(" Hz"));
    }
}

// ============================================================================
// AS3935 Lightning Detector — RIMOSSO (2026-09-23)
//   Problemi hardware breakout CJMCU-3935: sensore non raggiungibile su I2C.
//   Stub vuoti mantenuti per non rompere riferimenti in setup()/loop()/altrove.
//   Per ripristinare: riattivare defines vars + blocco originale funzioni.
// ============================================================================
// --- Stub variabili globali AS3935 referenziate fuori da questo modulo ---
static bool          g_as3935_ok            = false;

static void as3935_isr(void) {}
static int  as3935_writeReg8(uint8_t, uint8_t) { return -1; }
static int  as3935_readReg8(uint8_t, uint8_t*) { return -1; }
static int  as3935_cmdReset(void)              { return -1; }
static int  as3935_cmdCalibrateRco(void)       { return -1; }
static bool as3935_probeAddr(uint8_t addr)     { (void)addr; return false; }

static void as3935_bruteForce8(void)          {}
static void as3935_dumpRegs(const char*)       {}
static void as3935_init(void)                  { Serial.println(F("[AS3935] Sensore rimosso dal progetto · stub init.")); }
static void as3935_poll(void)                  {}

// ============================================================================
// BME280 - inizializzazione e lettura
// ============================================================================
static void bme280_init(void)
{
    uint8_t addr = BME280_ADDR_0;
    if (!g_bme.begin(addr, &Wire)) {
        addr = BME280_ADDR_1;
        if (!g_bme.begin(addr, &Wire)) {
            g_bme_ok = false;
            Serial.println(F("BME280: non trovato agli indirizzi 0x76/0x77"));
            return;
        }
    }
    g_bme_ok = true;
    g_bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,
        Adafruit_BME280::SAMPLING_X2,
        Adafruit_BME280::SAMPLING_X16,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_X16,
        Adafruit_BME280::STANDBY_MS_125
    );
    Serial.print(F("BME280: OK @ 0x"));
    if (addr < 0x10) Serial.print('0');
    Serial.println(addr, HEX);
}

static bool bme280_read(float *tempC, float *hPa, float *hum)
{
    if (!g_bme_ok) return false;
    *tempC = g_bme.readTemperature();
    *hPa   = g_bme.readPressure() / 100.0f;
    *hum   = g_bme.readHumidity();
    return true;
}

// ============================================================================
// Handlers HTTP
// ============================================================================
static void handleRoot(void)
{
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    g_http.send_P(200, "text/html; charset=utf-8", INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

static void jsonAppendStr(String& s, const char* key, const char* val, bool comma=true) {
    s += '"'; s += key; s += '"'; s += ':';
    s += '"';
    for (const char* p = val; p && *p; ++p) {
        char c = *p;
        if (c == '"' || c == '\\') { s += '\\'; s += c; }
        else if (c >= 0x20) s += c;
    }
    s += '"';
    if (comma) s += ',';
}
static void jsonAppendU32(String& s, const char* key, uint32_t v, bool comma=true) {
    s += '"'; s += key; s += '"'; s += ':'; s += String(v); if (comma) s += ',';
}
static void jsonAppendI32(String& s, const char* key, int32_t v, bool comma=true) {
    s += '"'; s += key; s += '"'; s += ':'; s += String(v); if (comma) s += ',';
}
static void jsonAppendBool(String& s, const char* key, bool v, bool comma=true) {
    s += '"'; s += key; s += '"'; s += ':'; s += (v ? "true" : "false"); if (comma) s += ',';
}
static void jsonAppendFlt(String& s, const char* key, float v, int digits=3, bool comma=true) {
    s += '"'; s += key; s += '"'; s += ':'; s += String(v, digits); if (comma) s += ',';
}

static void handleApiLabels(void) {
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    String s = "{";
    jsonAppendStr(s, "device_name",         g_config.device_name);
    jsonAppendStr(s, "sensor_as5600_name",  g_config.sensor_as5600_name);
    jsonAppendStr(s, "sensor_sht31_name",   g_config.sensor_sht31_name);
    jsonAppendStr(s, "sensor_bme280_name",  g_config.sensor_bme280_name);
    jsonAppendStr(s, "sensor_hall_name",    g_config.sensor_hall_name);
    jsonAppendStr(s, "sensor_ltr390_name",  g_config.sensor_ltr390_name);
    jsonAppendStr(s, "sensor_rain_name",    g_config.sensor_rain_name, false);
    s += "}";
    g_http.send(200, "application/json", s);
}

static void ICACHE_FLASH_ATTR handleApiConfigGet(void) {
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    String s = "{";
    jsonAppendStr(s, "device_name",         g_config.device_name);
    jsonAppendStr(s, "sensor_as5600_name",  g_config.sensor_as5600_name);
    jsonAppendStr(s, "sensor_sht31_name",   g_config.sensor_sht31_name);
    jsonAppendStr(s, "sensor_bme280_name",  g_config.sensor_bme280_name);
    jsonAppendStr(s, "sensor_hall_name",    g_config.sensor_hall_name);
    jsonAppendStr(s, "sensor_ltr390_name",  g_config.sensor_ltr390_name);
    jsonAppendStr(s, "wifi_ssid",           g_config.wifi_ssid);
    jsonAppendStr(s, "wifi_password",       g_config.wifi_password);
    jsonAppendStr(s, "ap_ssid",             g_config.ap_ssid);
    jsonAppendStr(s, "ap_password",         g_config.ap_password);
    jsonAppendStr(s, "mqtt_server",         g_config.mqtt_server);
    jsonAppendU32(s, "mqtt_port",           g_config.mqtt_port);
    jsonAppendStr(s, "mqtt_user",           g_config.mqtt_user);
    jsonAppendStr(s, "mqtt_password",       g_config.mqtt_password);
    jsonAppendStr(s, "mqtt_device_id",      g_config.mqtt_device_id);
    jsonAppendU32(s, "as5600_zpos",         g_config.as5600_zpos);
    jsonAppendU32(s, "as5600_mpos",         g_config.as5600_mpos);
    jsonAppendU32(s, "as5600_mang",         g_config.as5600_mang);
    jsonAppendI32(s, "as5600_offset_deg_100x", g_config.as5600_offset_deg_100x);
    jsonAppendU32(s, "as5600_agc_min",      g_config.as5600_agc_min);
    jsonAppendU32(s, "as5600_agc_max",      g_config.as5600_agc_max);
    jsonAppendU32(s, "as5600_mag_min",      cfg_getAs5600MagMin(&g_config));
    jsonAppendU32(s, "hdgf_filter_enable",      (g_hdrf_cfg.filter_enable ? 1UL : 0UL));
    jsonAppendFlt(s, "hdgf_spike_threshold_deg", (float)g_hdrf_cfg.spike_threshold_deg_x100 / 100.0f, 2);
    jsonAppendU32(s, "hdgf_movavg_window",      g_hdrf_cfg.movavg_window);
    jsonAppendU32(s, "hdgf_output_period_ms",   g_hdrf_cfg.output_period_ms);
    jsonAppendFlt(s, "hdgf_ema_alpha",          (float)g_hdrf_cfg.ema_alpha_x100 / 100.0f, 2);
    jsonAppendFlt(s, "bme280_altitude_ref_m",g_config.bme280_altitude_ref_m, 2);
    jsonAppendFlt(s, "bme280_sealevel_hpa",  g_config.bme280_sealevel_hpa, 2);
    jsonAppendU32(s, "ltr390_gain_id",      g_config.ltr390_gain_id);
    jsonAppendU32(s, "ltr390_res_it_id",    g_config.ltr390_res_it_id);
    jsonAppendFlt(s, "ltr390_window_factor",g_config.ltr390_window_factor, 2);
    jsonAppendU32(s, "hall_magnets_per_rev",g_config.hall_magnets_per_rev);
    jsonAppendFlt(s, "hall_anemo_radius_m", g_config.hall_anemo_radius_m, 3);
    jsonAppendFlt(s, "hall_anemo_calib_k",  g_config.hall_anemo_calib_k, 2);
    jsonAppendStr(s, "sensor_rain_name",    g_config.sensor_rain_name);
    jsonAppendFlt(s, "rain_funnel_dia_mm",  g_config.rain_funnel_dia_mm, 1);
    jsonAppendFlt(s, "rain_tip_mass_g",     g_config.rain_tip_mass_g, 2);
    jsonAppendFlt(s, "rain_calib_k",        g_config.rain_calib_k, 2);
    jsonAppendU32(s, "rain_dead_ms",        g_config.rain_dead_ms);
    jsonAppendFlt(s, "sfil_hall_ema",       (float)g_sfil_cfg.hall_ema_alpha_x1000 / 1000.0f, 2);
    jsonAppendFlt(s, "sfil_hall_decay",     (float)g_sfil_cfg.hall_hold_decay_x10000 / 10000.0f, 4);
    jsonAppendFlt(s, "sfil_rain_rate_ema",  (float)g_sfil_cfg.rain_rate_ema_x1000 / 1000.0f, 2);
    // === VALORI RUNTIME SENSORI (snapshot globali g_snap_*) ===
    jsonAppendU32(s, "uptime_s",            (uint32_t)(millis() / 1000UL));
    jsonAppendU32(s, "heap_free_kb",        (uint32_t)(ESP.getFreeHeap() / 1024U));
    jsonAppendU32(s, "sensor_status_bits",  g_snap_status_bits);
    {
      char hx[14]; snprintf(hx, sizeof(hx), "0x%08lX", (unsigned long)g_snap_status_bits);
      jsonAppendStr(s, "sensor_status_hex", hx);
    }
    // AS5600 Bussola
    if (isfinite(g_snap_heading_deg)) jsonAppendFlt(s, "as5600_heading_deg", g_snap_heading_deg, 2);
    else                              jsonAppendFlt(s, "as5600_heading_deg", -999.0f, 1);
    // Hall Anemometro
    jsonAppendFlt(s, "wind_speed_ms",       g_snap_wind_ms, 3);
    // wind_speed_kmh RIMOSSO: duplicazione (km/h = ms * 3.6, calcolabile in Home Assistant
    //   nativamente da device_class wind_speed, oppure nel frontend / Utility Meter)
    // SHT31 Temp/Umid
    jsonAppendBool(s,"sht31_ok",            g_sht31_ok && isfinite(g_snap_sht31_t));
    if (isfinite(g_snap_sht31_t))  jsonAppendFlt(s, "sht31_temp_c",   g_snap_sht31_t,  2);
    if (isfinite(g_snap_sht31_rh)) jsonAppendFlt(s, "sht31_humidity_rh", g_snap_sht31_rh, 1);
    if (isfinite(g_snap_sht31_dp)) jsonAppendFlt(s, "sht31_dew_point_c", g_snap_sht31_dp, 1);
    if (isfinite(g_snap_sht31_wb)) jsonAppendFlt(s, "sht31_wet_bulb_c",  g_snap_sht31_wb, 1);
    if (isfinite(g_snap_sht31_vpd))jsonAppendFlt(s, "sht31_vpd_kpa",    g_snap_sht31_vpd, 2);
    // BME280 Pressione
    if (isfinite(g_snap_bme_t)) jsonAppendFlt(s, "bme280_temp_c",      g_snap_bme_t, 2);
    if (isfinite(g_snap_bme_p)) jsonAppendFlt(s, "bme280_pressure_hpa",g_snap_bme_p, 1);
    if (isfinite(g_snap_bme_h)) jsonAppendFlt(s, "bme280_humidity_rh", g_snap_bme_h, 1);
    // LTR390 Luce/UV
    jsonAppendBool(s,"ltr390_ok",           g_ltr390_ok && isfinite(g_snap_ltr_lux) && g_snap_ltr_lux >= -1e-6f);
    if (isfinite(g_snap_ltr_lux)) jsonAppendFlt(s, "ltr390_lux",        g_snap_ltr_lux, 1);
    if (isfinite(g_snap_ltr_uvi)) jsonAppendFlt(s, "ltr390_uv_index",   g_snap_ltr_uvi, 2);
    // Rain Gauge
    if (isfinite(g_snap_rain_total_mm)) jsonAppendFlt(s, "rain_total_mm",    g_snap_rain_total_mm, 2);
    if (isfinite(g_snap_rain_rate_mmh)) jsonAppendFlt(s, "rain_rate_mmh",    g_snap_rain_rate_mmh, 2);
    // === FINE VALORI RUNTIME ===
    jsonAppendStr(s, "fw_build",            FW_BUILD_STR, false);
    s += "}";
    g_http.send(200, "application/json", s);
}

static void httpGetStrField(const String& json, const char* key, char* out, size_t outMax) {
    out[0] = 0;
    String q = String("\"") + key + "\"";
    int idx = json.indexOf(q);
    if (idx < 0) return;
    idx = json.indexOf(':', idx + q.length());
    if (idx < 0) return;
    idx++;
    while (idx < (int)json.length() && (json[idx]==' '||json[idx]=='\t'||json[idx]=='\n'||json[idx]=='\r')) idx++;
    if (idx >= (int)json.length()) return;
    if (json[idx] == '"') {
        idx++;
        size_t wi = 0;
        while (idx < (int)json.length() && wi + 1 < outMax && json[idx] != '"') {
            if (json[idx] == '\\' && idx + 1 < (int)json.length()) {
                char n = json[idx+1];
                if (n == '"' || n == '\\') { out[wi++] = n; idx += 2; continue; }
            }
            out[wi++] = json[idx++];
        }
        out[wi] = 0;
    }
}
static float httpGetFltField(const String& json, const char* key, float def) {
    String q = String("\"") + key + "\"";
    int idx = json.indexOf(q);
    if (idx < 0) return def;
    idx = json.indexOf(':', idx + q.length());
    if (idx < 0) return def;
    idx++;
    while (idx < (int)json.length() && (json[idx]==' '||json[idx]=='\t'||json[idx]=='\n'||json[idx]=='\r')) idx++;
    int end = idx;
    // [BUG FIX] Parser adesso riconosce:
    //   - cifre 0-9
    //   - '.'  separatore decimale
    //   - '-'  segno negativo (SOLO in posizione iniziale, lo skippiamo per end-start)
    //   - 'e'/'E'+'+'  (notazione scientifica, safety: se presente non tronchiamo)
    // Fermiamo il parse a ',' '}' '\n' '\r' '\t' ' ' (NUOVO: anche tab/newline sono fine-campo)
    bool allowMinus = true;
    while (end < (int)json.length()) {
        char c = json[end];
        if (c == ',' || c == '}' || c == ' ' || c == '\n' || c == '\r' || c == '\t') break;
        if (c == '-' && !allowMinus) break;
        if (c == '-') { allowMinus = false; end++; continue; }
        if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' ) {
            allowMinus = false;
            end++;
        } else {
            break;
        }
    }
    String s = json.substring(idx, end);
    char buf[64]; s.toCharArray(buf, sizeof(buf)); buf[sizeof(buf)-1]=0;
    if (buf[0] == 0) return def;
    return strtof(buf, nullptr);
}
static int32_t httpGetI32Field(const String& json, const char* key, int32_t def) {
    // Implementazione DIRETTA con strtol (evita passaggio float e roundoff per int32).
    // Riconosce: [-+]?[0-9]+ + delimitatori [,} \t\n\r ]
    String q = String("\"") + key + "\"";
    int idx = json.indexOf(q);
    if (idx < 0) return def;
    idx = json.indexOf(':', idx + q.length());
    if (idx < 0) return def;
    idx++;
    // skip whitespace
    while (idx < (int)json.length() && (json[idx]==' '||json[idx]=='\t'||json[idx]=='\n'||json[idx]=='\r')) idx++;
    int end = idx;
    // Allow leading +/- only once at start
    bool allowSign = true;
    while (end < (int)json.length()) {
        char c = json[end];
        if (c==','||c=='}'||c==' '||c=='\n'||c=='\r'||c=='\t') break;
        if ((c=='-'||c=='+') && allowSign) { allowSign=false; end++; continue; }
        if (c>='0' && c<='9') { allowSign=false; end++; continue; }
        break;
    }
    String s = json.substring(idx, end);
    char buf[32]; s.toCharArray(buf, sizeof(buf)); buf[sizeof(buf)-1]=0;
    if (buf[0] == 0) return def;
    char* endp = nullptr;
    long v = strtol(buf, &endp, 10);
    if (endp == buf) return def;  // nessuna cifra trovata
    return (int32_t)v;
}
static uint32_t httpGetU32Field(const String& json, const char* key, uint32_t def) {
    // Implementazione DIRETTA con strtoul (evita passaggio float).
    // Accetta solo numeri positivi >= 0; negativi o malformati → def.
    String q = String("\"") + key + "\"";
    int idx = json.indexOf(q);
    if (idx < 0) return def;
    idx = json.indexOf(':', idx + q.length());
    if (idx < 0) return def;
    idx++;
    while (idx < (int)json.length() && (json[idx]==' '||json[idx]=='\t'||json[idx]=='\n'||json[idx]=='\r')) idx++;
    // Se troviamo '-' all'inizio subito dopo whitespace: è negativo → return def
    if (idx < (int)json.length() && json[idx] == '-') return def;
    int end = idx;
    bool allowSign = true;
    while (end < (int)json.length()) {
        char c = json[end];
        if (c==','||c=='}'||c==' '||c=='\n'||c=='\r'||c=='\t') break;
        if ((c=='+') && allowSign) { allowSign=false; end++; continue; }
        if (c>='0' && c<='9') { allowSign=false; end++; continue; }
        break;
    }
    String s = json.substring(idx, end);
    char buf[32]; s.toCharArray(buf, sizeof(buf)); buf[sizeof(buf)-1]=0;
    if (buf[0] == 0) return def;
    char* endp = nullptr;
    unsigned long v = strtoul(buf, &endp, 10);
    if (endp == buf) return def;
    return (uint32_t)v;
}

static void ICACHE_FLASH_ATTR handleApiConfigPost(void) {
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    if (!g_http.hasArg("plain")) {
        g_http.send(400, "application/json", "{\"ok\":false,\"error\":\"missing body\"}");
        return;
    }
    const String& body = g_http.arg("plain");
    // [TRACE DEBUG OFFSET] Stampa BODY JSON completo in Seriale (dimensione massima 4KB safety)
    {
      uint32_t bodyLen = body.length();
      Serial.printf_P(PSTR("[CFG-POST] ----- POST /api/config BODY ----- len=%u\n"), (unsigned)bodyLen);
      if (bodyLen == 0) {
        Serial.println(F("[CFG-POST] BODY VUOTO! (body.arg('plain') non ha dati)"));
      } else {
        // Stampa a blocchi (arduino println tronca ~ 1500 char)
        const uint32_t CHUNK = 768;
        for (uint32_t i=0;i<bodyLen;i+=CHUNK) {
          uint32_t n = ((i+CHUNK) < bodyLen) ? CHUNK : (bodyLen-i);
          String sub = body.substring(i, i+n);
          sub.trim();
          Serial.print(sub);
          Serial.flush();
        }
        Serial.println();
      }
      int idxOff = body.indexOf("\"as5600_offset_deg_100x\"");
      Serial.printf_P(PSTR("[CFG-POST] indexOf(as5600_offset_deg_100x) = %d (key presente? %s)\n"),
                       idxOff, (idxOff >= 0 ? "SI" : "NO"));
      Serial.println(F("[CFG-POST] ----- END BODY -----"));
    }
    Config_t next = g_config;
    if (body.indexOf("\"__DEFAULT__\"") >= 0 || body.indexOf("__DEFAULT__:true") >= 0) {
        config_setDefaults(&next);
    } else {
        char tmp[128];
        httpGetStrField(body, "device_name",         tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.device_name,         tmp, sizeof(next.device_name));
        httpGetStrField(body, "sensor_as5600_name",  tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.sensor_as5600_name,  tmp, sizeof(next.sensor_as5600_name));
        httpGetStrField(body, "sensor_bme280_name",  tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.sensor_bme280_name,  tmp, sizeof(next.sensor_bme280_name));
        httpGetStrField(body, "sensor_hall_name",    tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.sensor_hall_name,    tmp, sizeof(next.sensor_hall_name));
        httpGetStrField(body, "sensor_ltr390_name",  tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.sensor_ltr390_name,  tmp, sizeof(next.sensor_ltr390_name));
        httpGetStrField(body, "sensor_sht31_name",   tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.sensor_sht31_name,   tmp, sizeof(next.sensor_sht31_name));
        httpGetStrField(body, "wifi_ssid",           tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.wifi_ssid,           tmp, sizeof(next.wifi_ssid));
        httpGetStrField(body, "wifi_password",       tmp, sizeof(tmp)); if (tmp[0]||body.indexOf("\"wifi_password\"")>=0) util_scpy(next.wifi_password, tmp, sizeof(next.wifi_password));
        httpGetStrField(body, "ap_ssid",             tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.ap_ssid,             tmp, sizeof(next.ap_ssid));
        httpGetStrField(body, "ap_password",         tmp, sizeof(tmp)); if (tmp[0]||body.indexOf("\"ap_password\"")>=0) util_scpy(next.ap_password, tmp, sizeof(next.ap_password));
        httpGetStrField(body, "mqtt_server",         tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.mqtt_server,         tmp, sizeof(next.mqtt_server));
        httpGetStrField(body, "mqtt_user",           tmp, sizeof(tmp)); if (tmp[0]||body.indexOf("\"mqtt_user\"")>=0) util_scpy(next.mqtt_user, tmp, sizeof(next.mqtt_user));
        httpGetStrField(body, "mqtt_password",       tmp, sizeof(tmp)); if (tmp[0]||body.indexOf("\"mqtt_password\"")>=0) util_scpy(next.mqtt_password, tmp, sizeof(next.mqtt_password));
        httpGetStrField(body, "mqtt_device_id",      tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.mqtt_device_id,      tmp, sizeof(next.mqtt_device_id));
        if (body.indexOf("\"mqtt_port\"")>=0)      next.mqtt_port                = (uint16_t)httpGetU32Field(body, "mqtt_port",                next.mqtt_port);
        if (body.indexOf("\"as5600_zpos\"")>=0)     next.as5600_zpos              = (uint16_t)httpGetU32Field(body, "as5600_zpos",              next.as5600_zpos);
        if (body.indexOf("\"as5600_mpos\"")>=0)     next.as5600_mpos              = (uint16_t)httpGetU32Field(body, "as5600_mpos",              next.as5600_mpos);
        if (body.indexOf("\"as5600_mang\"")>=0)     next.as5600_mang              = (uint16_t)httpGetU32Field(body, "as5600_mang",              next.as5600_mang);
        if (body.indexOf("\"as5600_offset_deg_100x\"")>=0) {
            int32_t parsedRaw = httpGetI32Field(body, "as5600_offset_deg_100x", (int32_t)next.as5600_offset_deg_100x);
            Serial.printf_P(PSTR("[CFG-POST] PARSE OFFSET: raw=%d (int32) → cast int16=%d  (prev in next: %d)\n"),
                             (int)parsedRaw, (int)(int16_t)parsedRaw, (int)next.as5600_offset_deg_100x);
            next.as5600_offset_deg_100x = (int16_t)parsedRaw;
        } else {
            Serial.printf_P(PSTR("[CFG-POST] PARSE OFFSET: KEY NON TROVATA → non modificato (keep %d)\n"),
                             (int)next.as5600_offset_deg_100x);
        }
        if (body.indexOf("\"as5600_agc_min\"")>=0)       next.as5600_agc_min    = (uint8_t)httpGetU32Field(body, "as5600_agc_min",    next.as5600_agc_min);
        if (body.indexOf("\"as5600_agc_max\"")>=0)       next.as5600_agc_max    = (uint8_t)httpGetU32Field(body, "as5600_agc_max",    next.as5600_agc_max);
        if (body.indexOf("\"as5600_mag_min\"")>=0)       cfg_setAs5600MagMin(&next, (uint16_t)httpGetU32Field(body, "as5600_mag_min", cfg_getAs5600MagMin(&next)));
        if (body.indexOf("\"bme280_altitude_ref_m\"")>=0)   next.bme280_altitude_ref_m  = httpGetFltField(body, "bme280_altitude_ref_m",          next.bme280_altitude_ref_m);
        if (body.indexOf("\"bme280_sealevel_hpa\"")>=0)     next.bme280_sealevel_hpa    = httpGetFltField(body, "bme280_sealevel_hpa",            next.bme280_sealevel_hpa);
        if (body.indexOf("\"ltr390_gain_id\"")>=0)  next.ltr390_gain_id           = (uint8_t)httpGetU32Field(body, "ltr390_gain_id",            next.ltr390_gain_id);
        if (body.indexOf("\"ltr390_res_it_id\"")>=0)next.ltr390_res_it_id         = (uint8_t)httpGetU32Field(body, "ltr390_res_it_id",          next.ltr390_res_it_id);
        if (body.indexOf("\"ltr390_window_factor\"")>=0)    next.ltr390_window_factor  = httpGetFltField(body, "ltr390_window_factor",            next.ltr390_window_factor);
        if (body.indexOf("\"hall_magnets_per_rev\"")>=0)    next.hall_magnets_per_rev  = (uint8_t)httpGetU32Field(body, "hall_magnets_per_rev",    next.hall_magnets_per_rev);
        if (body.indexOf("\"hall_anemo_radius_m\"")>=0)     next.hall_anemo_radius_m   = httpGetFltField(body, "hall_anemo_radius_m",             next.hall_anemo_radius_m);
        if (body.indexOf("\"hall_anemo_calib_k\"")>=0)      next.hall_anemo_calib_k    = httpGetFltField(body, "hall_anemo_calib_k",              next.hall_anemo_calib_k);
        httpGetStrField(body, "sensor_rain_name", tmp, sizeof(tmp)); if (tmp[0]) util_scpy(next.sensor_rain_name, tmp, sizeof(next.sensor_rain_name));
        if (body.indexOf("\"rain_funnel_dia_mm\"")>=0)      next.rain_funnel_dia_mm    = httpGetFltField(body, "rain_funnel_dia_mm",             next.rain_funnel_dia_mm);
        if (body.indexOf("\"rain_tip_mass_g\"")>=0)         next.rain_tip_mass_g       = httpGetFltField(body, "rain_tip_mass_g",                next.rain_tip_mass_g);
        if (body.indexOf("\"rain_calib_k\"")>=0)            next.rain_calib_k          = httpGetFltField(body, "rain_calib_k",                   next.rain_calib_k);
        if (body.indexOf("\"rain_dead_ms\"")>=0)            next.rain_dead_ms          = (uint16_t)httpGetU32Field(body, "rain_dead_ms",         next.rain_dead_ms);
    }
    // ====================================================================
    // [NUOVO] Parse Heading Filter Config (struttura separata in EEPROM offset 3072)
    // ====================================================================
    HeadingFilterConfig_t next_hdrf = g_hdrf_cfg;
    bool hdrf_has_changes = false;
    if (body.indexOf("\"__DEFAULT__\"") >= 0 || body.indexOf("__DEFAULT__:true") >= 0) {
        hdrf_setDefaults(&next_hdrf);
        hdrf_has_changes = true;
    } else {
        if (body.indexOf("\"hdgf_filter_enable\"") >= 0) {
            // Parse sia booleano JSON (true/false) sia intero 1/0 (retrocompat)
            bool fe = (body.indexOf("\"hdgf_filter_enable\":true")  >= 0 ||
                       body.indexOf("\"hdgf_filter_enable\":1")     >= 0 ||
                       body.indexOf("\"hdgf_filter_enable\": 1")    >= 0);
            next_hdrf.filter_enable = fe ? (uint8_t)1U : (uint8_t)0U;
            hdrf_has_changes = true;
        }
        if (body.indexOf("\"hdgf_spike_threshold_deg\"") >= 0) {
            float v = httpGetFltField(body, "hdgf_spike_threshold_deg", (float)next_hdrf.spike_threshold_deg_x100 / 100.0f);
            int32_t x100 = (int32_t)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
            next_hdrf.spike_threshold_deg_x100 = (int16_t)x100;
            hdrf_has_changes = true;
        }
        if (body.indexOf("\"hdgf_movavg_window\"") >= 0) {
            next_hdrf.movavg_window = (uint8_t)httpGetU32Field(body, "hdgf_movavg_window", (uint32_t)next_hdrf.movavg_window);
            hdrf_has_changes = true;
        }
        if (body.indexOf("\"hdgf_output_period_ms\"") >= 0) {
            next_hdrf.output_period_ms = (uint16_t)httpGetU32Field(body, "hdgf_output_period_ms", (uint32_t)next_hdrf.output_period_ms);
            hdrf_has_changes = true;
        }
        if (body.indexOf("\"hdgf_ema_alpha\"") >= 0) {
            float v = httpGetFltField(body, "hdgf_ema_alpha", (float)next_hdrf.ema_alpha_x100 / 100.0f);
            uint32_t x100 = (uint32_t)(v * 100.0f + 0.5f);
            next_hdrf.ema_alpha_x100 = (uint8_t)x100;
            hdrf_has_changes = true;
        }
    }
    if (hdrf_has_changes) {
        // Sanitize e salvataggio HeadingFilterConfig_t (struttura separata, NON tocca Config_t CRC)
        hdrf_sanitize(&next_hdrf);
        Serial.printf_P(PSTR("[CFG-POST] HDGF: filter=%s spike=%.2f° win=%u out=%ums ema=%.2f  (save EEPROM? %s)\n"),
            next_hdrf.filter_enable ? "ON" : "OFF (RAW)",
            (double)((float)next_hdrf.spike_threshold_deg_x100 / 100.0f),
            (unsigned)next_hdrf.movavg_window,
            (unsigned)next_hdrf.output_period_ms,
            (double)((float)next_hdrf.ema_alpha_x100 / 100.0f),
            hdrf_has_changes ? "SI" : "NO");
        hdrf_saveToEEPROM(&next_hdrf);
        g_hdrf_cfg = next_hdrf;
        hdrf_applyRuntime(&g_hdrf_cfg);
    }
    // ====================================================================
    // [NUOVO] Parse Sensor Filter Config (struttura separata in EEPROM offset 3584 magic SFIL)
    //         (Filtri: Hall EMA + Hold decay; Rain Rate EMA)
    // ====================================================================
    SensorFilterConfig_t next_sfil = g_sfil_cfg;
    bool sfil_has_changes = false;
    if (body.indexOf("\"__DEFAULT__\"") >= 0 || body.indexOf("__DEFAULT__:true") >= 0) {
        sfil_setDefaults(&next_sfil);
        sfil_has_changes = true;
    } else {
        if (body.indexOf("\"sfil_hall_ema\"") >= 0) {
            float v = httpGetFltField(body, "sfil_hall_ema", (float)next_sfil.hall_ema_alpha_x1000 / 1000.0f);
            uint32_t x1000 = (uint32_t)(v * 1000.0f + 0.5f);
            next_sfil.hall_ema_alpha_x1000 = (uint16_t)x1000;
            sfil_has_changes = true;
        }
        if (body.indexOf("\"sfil_hall_decay\"") >= 0) {
            float v = httpGetFltField(body, "sfil_hall_decay", (float)next_sfil.hall_hold_decay_x10000 / 10000.0f);
            uint32_t x10000 = (uint32_t)(v * 10000.0f + 0.5f);
            next_sfil.hall_hold_decay_x10000 = (uint16_t)x10000;
            sfil_has_changes = true;
        }
        if (body.indexOf("\"sfil_rain_rate_ema\"") >= 0) {
            float v = httpGetFltField(body, "sfil_rain_rate_ema", (float)next_sfil.rain_rate_ema_x1000 / 1000.0f);
            uint32_t x1000 = (uint32_t)(v * 1000.0f + 0.5f);
            next_sfil.rain_rate_ema_x1000 = (uint16_t)x1000;
            sfil_has_changes = true;
        }
    }
    if (sfil_has_changes) {
        sfil_sanitize(&next_sfil);
        Serial.printf_P(PSTR("[CFG-POST] SFIL: hall_ema=%.3f hall_decay=%.4f rain_rate_ema=%.3f (save EEPROM SI)\n"),
            (double)((float)next_sfil.hall_ema_alpha_x1000 / 1000.0f),
            (double)((float)next_sfil.hall_hold_decay_x10000 / 10000.0f),
            (double)((float)next_sfil.rain_rate_ema_x1000 / 1000.0f));
        sfil_saveToEEPROM(&next_sfil);
        g_sfil_cfg = next_sfil;
        sfil_applyRuntime(&g_sfil_cfg);
    }
    bool needReboot = false;
    {
        int32_t dbg1 = next.as5600_offset_deg_100x;
        bool sanitized = config_sanitize(&next);
        (void)sanitized;
        int32_t dbg2 = next.as5600_offset_deg_100x;
        Serial.printf_P(PSTR("[CFG-POST] as5600_offset_deg_100x: HTTP-PARSED=%d  AFTER-SANITIZE=%d  (g_config BEFORE=%d)\n"),
                         (int)dbg1, (int)dbg2, (int)g_config.as5600_offset_deg_100x);
    }
    // --- Confronta campi WiFi (modificati → reboot) ---
    if (strcmp(next.wifi_ssid,       g_config.wifi_ssid)       != 0 ||
        strcmp(next.wifi_password,   g_config.wifi_password)   != 0 ||
        strcmp(next.ap_ssid,         g_config.ap_ssid)         != 0 ||
        strcmp(next.ap_password,     g_config.ap_password)     != 0) {
        needReboot = true;
        g_wifi_config_changed = true;
    }
    // --- Confronta campi MQTT (non reboot ma riconnessione + nuovo discovery) ---
    if (strcmp(next.mqtt_server,     g_config.mqtt_server)     != 0 ||
        strcmp(next.mqtt_user,       g_config.mqtt_user)       != 0 ||
        strcmp(next.mqtt_password,   g_config.mqtt_password)   != 0 ||
        strcmp(next.mqtt_device_id,  g_config.mqtt_device_id)  != 0 ||
        next.mqtt_port               != g_config.mqtt_port     ||
        strcmp(next.device_name,     g_config.device_name)     != 0 ||
        strcmp(next.sensor_as5600_name, g_config.sensor_as5600_name) != 0 ||
        strcmp(next.sensor_as3935_name, g_config.sensor_as3935_name) != 0 ||
        strcmp(next.sensor_bme280_name, g_config.sensor_bme280_name) != 0 ||
        strcmp(next.sensor_hall_name,   g_config.sensor_hall_name)   != 0 ||
        strcmp(next.sensor_ltr390_name, g_config.sensor_ltr390_name) != 0 ||
        strcmp(next.sensor_sht31_name,  g_config.sensor_sht31_name)  != 0 ||
        strcmp(next.sensor_rain_name,   g_config.sensor_rain_name)   != 0) {
        g_mqtt_config_changed = true;
    }
    // Salva
    config_saveToEeprom(&next);
    g_config = next;
    Serial.printf_P(PSTR("[CFG-POST] POST-SAVE: g_config.as5600_offset_deg_100x = %d  (%.2f°)\n"),
                     (int)g_config.as5600_offset_deg_100x,
                     (float)g_config.as5600_offset_deg_100x / 100.0f);

    // --- Applica runtime (se non serve reboot per WiFi) ---
    //   Se WiFi è cambiato needReboot=true → verrà fatto restart tra 2s dal JS.
    //   Altrimenti applichiamo SUBITO tutti i parametri senza attesa del boot.
    if (!needReboot) {
        config_applyRuntimeFromWeb();
    }

    String s = "{";
    s += "\"ok\":true,";
    s += "\"need_reboot\":"; s += (needReboot ? "true" : "false");
    s += ",\"mqtt_changed\":"; s += (g_mqtt_config_changed ? "true" : "false");
    s += ",\"wifi_changed\":"; s += (g_wifi_config_changed ? "true" : "false");
    // [DEBUG] Echo offset al client per controllo immediato nel browser (Console)
    s += ",\"dbg_offset_deg_100x\":"; s += (int)g_config.as5600_offset_deg_100x;
    s += "}";
    g_http.send(200, "application/json", s);
}

static void handleApiRainReset(void) {
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    if (g_http.method() != HTTP_POST) {
        g_http.send(405, "application/json", "{\"ok\":false,\"error\":\"POST expected\"}");
        return;
    }
    uint32_t now_ticks = 0;
    {
        noInterrupts();
        now_ticks = g_rain_tick_count;
        interrupts();
    }
    uint32_t prev_offset = g_rain_total_offset_ticks;
    // --- RICHIESTA UTENTE: reset via pagina web resetta PIAGGIA TOTALE ---
    g_rain_total_offset_ticks = now_ticks;
    // (pioggia oggi NON si tocca: si resetta AUTOMATICAMENTE a mezzanotte)
    g_mqtt_force_repub_all = true;
    // --- Persistenza IMMEDIATA in EEPROM ---
    rainWritePersistImmediate("HTTP reset totale");
    Serial.printf_P(PSTR("[RAIN] Reset HTTP: TOTALE = 0 (prev_tot_off=%lu → new_tot_off=%lu actual=%lu). OGGI invariato (reset auto a mezzanotte)\n"),
                      (unsigned long)prev_offset,
                      (unsigned long)g_rain_total_offset_ticks,
                      (unsigned long)now_ticks);
    // --- Publish MQTT IMMEDIATO (senza aspettare scheduler) per bug fix reset invisibile ---
    if (g_mqtt.connected()) {
        // forza aggiornamento g_snap_rain_total_mm con i nuovi offset:
        //   attendiamo il prossimo loop (≤ 20ms) tramite il flag g_mqtt_force_repub_all
        //   già attivo; scheduler lo eseguirà in meno di 250ms (GrpA period).
        //   Ulteriore safety: subito dopo l'iterazione successiva, non c'è bisogno
        //   di calcolare lo snapshot qui perché sarebbe disallineato dal loop.
    }
    g_http.send(200, "application/json", "{\"ok\":true,\"total_offset_ticks\":" +
                String((unsigned long)g_rain_total_offset_ticks) +
                ",\"comment\":\"pioggia totale azzerata; pioggia oggi invariata (reset auto mezzanotte)\"}");
}

// ============================================================================
// MQTT Ingress: comando da Home Assistant → reset PIAGGIA TOTALE (non OGGI)
//   Topic: homeassistant/sensor/<device_id>/cmd/rain_reset
//   Payload: qualsiasi stringa non vuota triggera il reset
//   Pioggia OGGI si resetta AUTOMATICAMENTE a MEZZANOTTE
// ============================================================================
static void mqttOnMessage(char* topic, uint8_t* payload, unsigned int length) {
    char expectedTopic[160];
    snprintf_P(expectedTopic, sizeof(expectedTopic),
               PSTR("homeassistant/sensor/%s/cmd/rain_reset"), g_config.mqtt_device_id);
    if (strcmp(topic, expectedTopic) != 0) return;

    const bool trigger = (length > 0);
    if (!trigger) return;

    uint32_t now_ticks = 0;
    {
        noInterrupts();
        now_ticks = g_rain_tick_count;
        interrupts();
    }
    uint32_t prev_offset = g_rain_total_offset_ticks;
    // --- RICHIESTA UTENTE: reset MQTT Hassio resetta TOTALE (non più evento/oggi) ---
    g_rain_total_offset_ticks = now_ticks;
    // Pioggia oggi NON si resetta via MQTT → la lascia proseguire; si azzera a mezzanotte.
    g_mqtt_force_repub_all = true;
    // Persistenza IMMEDIATA in EEPROM: dopo reset, se si riavvia torna a 0 correttamente
    rainWritePersistImmediate("MQTT reset totale");
    Serial.printf_P(PSTR("[RAIN] Reset MQTT Hassio: TOTALE = 0 (prev_tot_off=%lu → new_tot_off=%lu actual=%lu). OGGI invariato (reset auto a mezzanotte)\n"),
                      (unsigned long)prev_offset,
                      (unsigned long)g_rain_total_offset_ticks,
                      (unsigned long)now_ticks);
}

// ============================================================================
// [DIAG I2C] GET /api/i2cdiag · sonde live read-only (nessuna modifica di stato
//   dei sensori). Serve perché l'unico canale di log (seriale) non è accessibile.
// ============================================================================
static int diag_readReg(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t e = Wire.endTransmission(false);
    if (e != 0) return e;
    if (Wire.requestFrom(addr, n) != n) return 0xFE;
    for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
    return 0;
}
static void handleApiI2cDiag(void) {
    String s; s.reserve(900);
    if (g_safe_mode) {   // Wire non inizializzato in safe mode: nessuna sonda I2C
        s = F("{\"safe_mode\":1,\"boot_count\":"); s += g_safe_boot_count;
        s += F(",\"uptime_s\":"); s += (unsigned long)(millis() / 1000UL); s += '}';
        g_http.send(200, "application/json", s);
        return;
    }
    // ?khz=N (5..400): frequenza SOLO per questa sonda, poi si torna a 100 kHz (test cavo lungo)
    uint32_t khz = I2C_CLOCK_HZ / 1000UL;
    if (g_http.hasArg("khz")) { khz = (uint32_t)g_http.arg("khz").toInt(); if (khz < 5) khz = 5; if (khz > 400) khz = 400; }
    Wire.setClock(khz * 1000UL);
    s += F("{\"safe_mode\":0,\"khz\":"); s += khz;
    s += F(",\"boot_count\":"); s += g_safe_boot_count;
    s += F(",\"sda_pin\":"); s += digitalRead(I2C_SDA_PIN);
    s += F(",\"scl_pin\":"); s += digitalRead(I2C_SCL_PIN);
    s += F(",\"scan\":[");
    bool first = true;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission(true) == 0) {
            if (!first) s += ',';
            char h[8]; snprintf(h, sizeof(h), "\"0x%02X\"", a); s += h; first = false;
        }
        if ((a & 0x0F) == 0) yield();
    }
    s += ']';
    uint8_t b[6]; int e;
    // BME280: chip-id 0xD0 (0x60=BME280, 0x58=BMP280)
    for (uint8_t a = 0x76; a <= 0x77; a++) {
        b[0] = 0; e = diag_readReg(a, 0xD0, b, 1);
        s += F(",\"bme_0x"); s += String(a, HEX); s += F("\":{\"err\":"); s += e;
        s += F(",\"chip_id\":"); s += b[0]; s += '}';
    }
    // LTR390: PART_ID (atteso 0xB2 = 178) + MAIN_CTRL(0x00) + MAIN_STATUS(0x07)
    b[0] = b[1] = b[2] = 0;
    e = diag_readReg(LTR390_ADDR, LTR390_REG_PART_ID, &b[0], 1);
    (void)diag_readReg(LTR390_ADDR, 0x00, &b[1], 1);
    (void)diag_readReg(LTR390_ADDR, 0x07, &b[2], 1);
    s += F(",\"ltr390\":{\"err\":"); s += e; s += F(",\"part_id\":"); s += b[0];
    s += F(",\"main_ctrl\":"); s += b[1]; s += F(",\"main_status\":"); s += b[2];
    s += F(",\"ok_flag\":"); s += (int)g_ltr390_ok; s += F(",\"als_cnt\":"); s += g_ltr390_als_cnt;
    s += F(",\"recov\":"); s += g_ltr390_recovery_count; s += '}';
    // AS5600: STATUS(0x0B) RAW(0x0C..0D) AGC(0x1A) MAGNITUDE(0x1B..1C)
    uint8_t st = 0, agc = 0, raw[2] = {0,0}, mag[2] = {0,0};
    e = diag_readReg(AS5600_ADDR, 0x0B, &st, 1);
    int e2 = diag_readReg(AS5600_ADDR, 0x0C, raw, 2);
    (void)diag_readReg(AS5600_ADDR, 0x1A, &agc, 1);
    (void)diag_readReg(AS5600_ADDR, 0x1B, mag, 2);
    s += F(",\"as5600\":{\"err\":"); s += e; s += F(",\"err_raw\":"); s += e2;
    s += F(",\"status\":"); s += st;
    s += F(",\"md\":"); s += (st & AS5600_ST_MD) ? 1 : 0;
    s += F(",\"ml\":"); s += (st & AS5600_ST_ML) ? 1 : 0;
    s += F(",\"mh\":"); s += (st & AS5600_ST_MH) ? 1 : 0;
    s += F(",\"raw\":"); s += (unsigned)((((uint16_t)raw[0] << 8) | raw[1]) & 0x0FFF);
    s += F(",\"agc\":"); s += agc;
    s += F(",\"mag\":"); s += (unsigned)((((uint16_t)mag[0] << 8) | mag[1]) & 0x0FFF);
    s += F(",\"heading\":"); s += isfinite(g_snap_heading_deg) ? String(g_snap_heading_deg, 1) : String("null");
    s += '}';
    // SHT31 0x44 e 0x45: status register (cmd 0xF32D), solo lettura
    for (uint8_t a = 0x44; a <= 0x45; a++) {
        Wire.beginTransmission(a);
        Wire.write((uint8_t)(SHT31_CMD_READ_STATUS >> 8));
        Wire.write((uint8_t)(SHT31_CMD_READ_STATUS & 0xFF));
        e = Wire.endTransmission(true);
        uint16_t sr = 0; int got = 0;
        if (e == 0) {
            got = Wire.requestFrom(a, (uint8_t)3);
            if (got >= 2) sr = ((uint16_t)Wire.read() << 8) | (uint16_t)Wire.read();
            while (Wire.available()) (void)Wire.read();
        }
        s += F(",\"sht_0x"); s += String(a, HEX); s += F("\":{\"err\":"); s += e;
        s += F(",\"got\":"); s += got; s += F(",\"status\":"); s += sr; s += '}';
    }
    s += F(",\"sht31_ok_flag\":"); s += (int)g_sht31_ok;
    s += F(",\"sht31_i2c_err\":"); s += g_sht31_i2c_error_count;
    s += F(",\"sht31_crc_err\":"); s += g_sht31_crc_error_count;
    s += F(",\"bme_ok_flag\":"); s += (int)g_bme_ok;
    s += F(",\"uptime_s\":"); s += (unsigned long)(millis() / 1000UL);
    s += '}';
    Wire.setClock(I2C_CLOCK_HZ);   // ripristina la frequenza di esercizio
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    g_http.send(200, "application/json", s);
}

// ============================================================================
// [DIAG I2C] GET /api/i2ctest?sensor=as5600|ltr390|sht31|bme280&n=50&khz=100&gcreset=0
//   Test di un singolo sensore: N sonde sull'indirizzo (conteggio ACK/errori),
//   opzionale general-call reset (0x00,0x06), poi init completo col driver del FW.
// ============================================================================
static void handleApiI2cTest(void) {
    if (g_safe_mode) { g_http.send(200, "application/json", "{\"safe_mode\":1}"); return; }
    String sen = g_http.hasArg("sensor") ? g_http.arg("sensor") : String("");
    uint8_t addr;
    if      (sen == "as5600") addr = AS5600_ADDR;
    else if (sen == "ltr390") addr = LTR390_ADDR;
    else if (sen == "sht31")  addr = SHT31_ADDR;
    else if (sen == "sht31b") addr = 0x45;
    else if (sen == "bme280") addr = BME280_ADDR_0;
    else { g_http.send(400, "application/json", "{\"error\":\"sensor=as5600|ltr390|sht31|sht31b|bme280\"}"); return; }
    uint32_t n = g_http.hasArg("n") ? (uint32_t)g_http.arg("n").toInt() : 50;  if (n < 1) n = 1; if (n > 200) n = 200;
    uint32_t khz = g_http.hasArg("khz") ? (uint32_t)g_http.arg("khz").toInt() : I2C_CLOCK_HZ / 1000UL; if (khz < 5) khz = 5; if (khz > 400) khz = 400;
    bool gc = g_http.hasArg("gcreset") && g_http.arg("gcreset") == "1";
    Wire.setClock(khz * 1000UL);
    String s; s.reserve(400);
    s += F("{\"sensor\":\""); s += sen; s += F("\",\"addr\":"); s += addr;
    s += F(",\"khz\":"); s += khz;
    if (gc) {   // general call reset: i device che lo supportano (SHT3x) fanno soft reset
        Wire.beginTransmission((uint8_t)0x00); Wire.write((uint8_t)0x06);
        s += F(",\"gcreset_err\":"); s += Wire.endTransmission(true);
        delay(5);
    }
    uint32_t ack = 0, e[6] = {0,0,0,0,0,0};
    unsigned long t0 = micros();
    for (uint32_t i = 0; i < n; i++) {
        Wire.beginTransmission(addr);
        uint8_t r = Wire.endTransmission(true);
        if (r == 0) ack++;
        e[r < 5 ? r : 5]++;
        delayMicroseconds(500);
        if ((i & 0x0F) == 0) yield();
    }
    s += F(",\"probes\":"); s += n; s += F(",\"ack\":"); s += ack;
    s += F(",\"err_nack_addr\":"); s += e[2]; s += F(",\"err_nack_data\":"); s += e[3];
    s += F(",\"err_other\":"); s += (e[1] + e[4] + e[5]);
    s += F(",\"probe_us\":"); s += (unsigned long)(micros() - t0);
    // Init completo col driver del firmware (stesso codice del setup)
    if (sen == "sht31" || sen == "sht31b") {
        sht31_init();
        s += F(",\"init_ok\":"); s += (int)g_sht31_ok;
        s += F(",\"i2c_err\":"); s += g_sht31_i2c_error_count;
        s += F(",\"crc_err\":"); s += g_sht31_crc_error_count;
    } else if (sen == "ltr390") {
        g_ltr390_ok = ltr390_init();
        s += F(",\"init_ok\":"); s += (int)g_ltr390_ok;
    } else if (sen == "bme280") {
        bme280_init();
        s += F(",\"init_ok\":"); s += (int)g_bme_ok;
    } else {   // as5600: nessun init, lettura STATUS
        uint8_t st = 0; uint8_t r = as5600_readReg8(AS5600_REG_STATUS, &st);
        s += F(",\"init_ok\":"); s += (r == 0) ? 1 : 0;
        s += F(",\"status\":"); s += st;
    }
    Wire.setClock(I2C_CLOCK_HZ);
    s += F(",\"sda_pin\":"); s += digitalRead(I2C_SDA_PIN);
    s += F(",\"scl_pin\":"); s += digitalRead(I2C_SCL_PIN);
    s += '}';
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    g_http.send(200, "application/json", s);
}

static void handleApiRestart(void) {
    g_http.sendHeader("Cache-Control", "no-store, no-cache");
    g_http.send(200, "application/json", "{\"ok\":true,\"restarting\":true}");
    crash_capturePanicContext("HTTP_API_RESTART");
    delay(100);
    ESP.restart();
}

// ============================================================================
// Funzioni MQTT Home Assistant
// ============================================================================
static void mqttGetBaseTopic(char* buf, size_t bufLen) {
    const char* did = (g_config.mqtt_device_id[0]) ? g_config.mqtt_device_id : DEFAULT_MQTT_DEVICE_ID;
    snprintf(buf, bufLen, "homeassistant/sensor/%s/", did);
}
static void mqttGetAvailTopic(char* buf, size_t bufLen) {
    const char* did = (g_config.mqtt_device_id[0]) ? g_config.mqtt_device_id : DEFAULT_MQTT_DEVICE_ID;
    snprintf(buf, bufLen, "homeassistant/sensor/%s/availability", did);
}
static size_t mqttBuildTopic(char* buf, size_t bufLen, const char* suffix)
{
    char base[96]; mqttGetBaseTopic(base, sizeof(base));
    return snprintf(buf, bufLen, "%s%s", base, suffix);
}

static void mqttPublishScalar(const char* stateSuffix, float v, uint8_t decimals)
{
    if (!isfinite(v)) return;
    char topic[96];
    char value[24];
    mqttBuildTopic(topic, sizeof(topic), stateSuffix);
    strncat(topic, "/state", sizeof(topic) - strlen(topic) - 1);
    snprintf(value, sizeof(value), "%.*f", decimals, v);
    g_mqtt.publish(topic, value, true);
}

static void mqttPublishScalarStr(const char* stateSuffix, const char* s)
{
    if (!s) return;
    char topic[96];
    mqttBuildTopic(topic, sizeof(topic), stateSuffix);
    strncat(topic, "/state", sizeof(topic) - strlen(topic) - 1);
    g_mqtt.publish(topic, s, true);
}

static void mqttPublishHexU32(const char* stateSuffix, uint32_t v)
{
    char topic[96];
    char value[16];
    mqttBuildTopic(topic, sizeof(topic), stateSuffix);
    strncat(topic, "/state", sizeof(topic) - strlen(topic) - 1);
    snprintf(value, sizeof(value), "0x%08X", (unsigned)v);
    g_mqtt.publish(topic, value, true);
}

static void mqttPublishU32(const char* stateSuffix, uint32_t v)
{
    char topic[96];
    char value[16];
    mqttBuildTopic(topic, sizeof(topic), stateSuffix);
    strncat(topic, "/state", sizeof(topic) - strlen(topic) - 1);
    snprintf(value, sizeof(value), "%lu", (unsigned long)v);
    g_mqtt.publish(topic, value, true);
}

static void mqttPublishDiscoveryOne(
    const char* sensorId,
    const char* name,
    const char* devClass,
    const char* unit,
    const char* icon,
    const char* stateClass)
{
    const char* did = (g_config.mqtt_device_id[0]) ? g_config.mqtt_device_id : DEFAULT_MQTT_DEVICE_ID;
    const char* dnm = (g_config.device_name[0])     ? g_config.device_name     : DEFAULT_DEVICE_NAME;
    char availTopic[128]; mqttGetAvailTopic(availTopic, sizeof(availTopic));
    char topic[128];
    char stateTopic[128];
    char uniqueId[96];

    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/%s/config", did, sensorId);
    {
        char base[96]; mqttGetBaseTopic(base, sizeof(base));
        snprintf(stateTopic, sizeof(stateTopic), "%s%s/state", base, sensorId);
    }
    snprintf(uniqueId, sizeof(uniqueId), "%s_%s", did, sensorId);

    String swVerStr = ESP.getCoreVersion();
    const char* swVer = swVerStr.c_str();
    const char* mfr = "DIY";
    const char* mdl = "ESP8266 D1 Mini Meteo";

    static char payload[1024];
    const size_t bufSize = sizeof(payload);
    int written = snprintf(payload, bufSize,
        "{"
        "\"name\":\"%s\","
        "\"unique_id\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"availability_topic\":\"%s\","
        "\"payload_available\":\"online\","
        "\"payload_not_available\":\"offline\"",
        name, uniqueId, stateTopic, availTopic);

    if (written > 0 && written < (int)bufSize - 1) {
        if (devClass && devClass[0]) {
            written += snprintf(payload + written, bufSize - written,
                ",\"device_class\":\"%s\"", devClass);
        }
    }
    if (written > 0 && written < (int)bufSize - 1) {
        if (unit && unit[0]) {
            written += snprintf(payload + written, bufSize - written,
                ",\"unit_of_measurement\":\"%s\"", unit);
        }
    }
    if (written > 0 && written < (int)bufSize - 1) {
        if (stateClass && stateClass[0]) {
            written += snprintf(payload + written, bufSize - written,
                ",\"state_class\":\"%s\"", stateClass);
        }
    }
    if (written > 0 && written < (int)bufSize - 1) {
        if (icon && icon[0]) {
            written += snprintf(payload + written, bufSize - written,
                ",\"icon\":\"%s\"", icon);
        }
    }
    if (written > 0 && written < (int)bufSize - 1) {
        written += snprintf(payload + written, bufSize - written,
            ",\"device\":{"
            "\"identifiers\":[\"%s\"],"
            "\"name\":\"%s\","
            "\"manufacturer\":\"%s\","
            "\"model\":\"%s\","
            "\"sw_version\":\"%s\""
            "}"
            "}",
            did, dnm, mfr, mdl, swVer);
    }

    if (written > 0 && written < (int)bufSize - 1) {
        g_mqtt.publish(topic, payload, true);
    }
}

static void mqttPublishAllDiscovery(void)
{
    // --- Bussola / Direzione Vento (Heading Indicator AS5600) ---
    mqttPublishDiscoveryOne("compass_deg",               "Heading Gradi",             "wind_direction",       "°",     "mdi:compass-outline",     "measurement");
    mqttPublishDiscoveryOne("compass_rose16",            "Quadrante 16 Venti",        NULL,                   NULL,    "mdi:compass-rose",        NULL);

    // --- Anemometro Hall A3144 ---
    //   wind_speed_kmh RIMOSSO (stessa misura di wind_speed_ms in unità diversa;
    //   Home Assistant converte automaticamente m/s → km/h se device_class=wind_speed)
    mqttPublishDiscoveryOne("wind_speed_ms",             "Velocità Vento",            "wind_speed",           "m/s",   "mdi:weather-windy",       "measurement");
    mqttPublishDiscoveryOne("wind_beaufort_name",        "Beaufort Nome",             NULL,                   NULL,    "mdi:information",         NULL);
    mqttPublishDiscoveryOne("anemometer_count",          "Impulsi Conteggiati",       NULL,                   NULL,    "mdi:counter",             "total_increasing");
    mqttPublishDiscoveryOne("wind_gust_max_ms",          "Raffica Max 10min",         "wind_speed",           "m/s",   "mdi:weather-windy",       "measurement");

    // --- SHT31-D · 9 sensori (T + RH + indici termici outdoor coerenti) ---
    mqttPublishDiscoveryOne("sht31_temp_c",              "SHT31 Temperatura",         "temperature",          "°C",    "mdi:thermometer",         "measurement");
    mqttPublishDiscoveryOne("sht31_humidity_rh",         "SHT31 Umidità Relativa",    "humidity",             "%",     "mdi:water-percent",       "measurement");
    mqttPublishDiscoveryOne("sht31_dew_point_c",         "SHT31 Punto Rugiada",       "temperature",          "°C",    "mdi:water-thermometer",   "measurement");
    mqttPublishDiscoveryOne("sht31_frost_point_c",       "SHT31 Punto Brina",         "temperature",          "°C",    "mdi:snowflake-melt",      "measurement");
    mqttPublishDiscoveryOne("sht31_wet_bulb_c",          "SHT31 Wet Bulb",            "temperature",          "°C",    "mdi:air-humidifier",      "measurement");
    mqttPublishDiscoveryOne("sht31_vpd_kpa",             "SHT31 VPD kPa",             NULL,                   "kPa",   "mdi:leaf",                "measurement");
    mqttPublishDiscoveryOne("sht31_humidity_absolute",   "SHT31 Umidità Assoluta",    NULL,                   "g/m³",  "mdi:water-opacity",       "measurement");
    mqttPublishDiscoveryOne("sht31_heat_index_c",        "SHT31 Heat Index",          "temperature",          "°C",    "mdi:sun-thermometer",     "measurement");
    mqttPublishDiscoveryOne("sht31_humidex",             "SHT31 Humidex",             NULL,                   NULL,    "mdi:heat-wave",           "measurement");
    mqttPublishDiscoveryOne("sht31_heater_on",           "SHT31 Heater",              NULL,                   NULL,    "mdi:radiator",            NULL);

    // --- Temperatura Modulo BME280 (board interna, non outdoor) ---
    mqttPublishDiscoveryOne("temp_module_c",             "Temperatura Modulo BME",    "temperature",          "°C",    "mdi:thermometer",         "measurement");

    // --- Pressione Atmosferica BME280 (solo formato scientifico hPa, derivabili in HA) ---
    mqttPublishDiscoveryOne("pressure_hpa",              "Pressione Atmosferica",     "atmospheric_pressure", "hPa",   "mdi:gauge",               "measurement");

    // --- Umidità Relativa BME280 (interna modulo, per confronto) ---
    mqttPublishDiscoveryOne("humidity_rh",               "Umidità Relativa",          "humidity",             "%",     "mdi:water-percent",       "measurement");

    // --- Indici Termici Combinati (fonte T+RH = SHT31 COERENTE outdoor, no BME mixed) ---
    mqttPublishDiscoveryOne("wind_chill_c",              "Wind Chill",                "temperature",          "°C",    "mdi:snowflake-thermometer","measurement");
    mqttPublishDiscoveryOne("apparent_temp_c",           "Apparent Temperature",      "temperature",          "°C",    "mdi:human",               "measurement");

    // --- LTR390 ALS (Light Photopic) + UV Sensor (solo valori calibrati con unità) ---
    mqttPublishDiscoveryOne("ltr_lux",            "Illuminanza",               "illuminance",     "lx",   "mdi:sun-wireless",    "measurement");
    mqttPublishDiscoveryOne("ltr_uv_index",       "Indice UV",                 NULL,              "UVI",  "mdi:sun-wireless",    "measurement");
    mqttPublishDiscoveryOne("ltr_uv_level",       "UV Level",                  NULL,              NULL,   "mdi:sun-alert",       NULL);
    mqttPublishDiscoveryOne("ltr_light_class",    "Luce Classe",               NULL,              NULL,   "mdi:theme-light-dark",NULL);

    // --- Pioggia Bascula Hall (diagnostica: valori utili) ---
    mqttPublishDiscoveryOne("rain_tick_count",      "Conteggio Tick Bascula",    NULL,                   "count","mdi:counter",           "total_increasing");
    mqttPublishDiscoveryOne("rain_total_mm",        "Pioggia Totale Storica",    "precipitation",        "mm",    "mdi:weather-rainy",     "total_increasing");
    mqttPublishDiscoveryOne("rain_today_mm",        "Pioggia Oggi (giorno corrente, auto-reset 00:00)","precipitation","mm","mdi:weather-pouring",   "measurement");
    mqttPublishDiscoveryOne("rain_rate_mmh",        "Intensità Pioggia",         "precipitation_intensity","mm/h","mdi:weather-rainy",     "measurement");
    mqttPublishDiscoveryOne("rain_intensity_class", "Classe Intensità WMO",      NULL,                   NULL,    "mdi:signal-distance-variant", NULL);
    mqttPublishDiscoveryOne("rain_last_tick_age_s", "Sec ultimo Tick Pioggia",   NULL,                   "s",     "mdi:clock-outline",     "measurement");

    // --- Stato Sistema ---
    mqttPublishDiscoveryOne("sensor_status_bitmask",     "Stato Sensori Bitmask",     NULL,                   NULL,    "mdi:chip",                NULL);
    mqttPublishDiscoveryOne("sensor_status_list",        "Sensori Collegati",         NULL,                   NULL,    "mdi:clipboard-list",      NULL);

    // --- Sistema e Rete (NUOVI M1-M4-M6-M7) ---
    mqttPublishDiscoveryOne("wifi_rssi_dbm",             "WiFi RSSI",                 "signal_strength",      "dBm",   "mdi:wifi",                "measurement");
    mqttPublishDiscoveryOne("wifi_ip_address",           "WiFi IP Address",           NULL,                   NULL,    "mdi:ip-network",          NULL);
    mqttPublishDiscoveryOne("system_uptime_s",           "Uptime",                    NULL,                   "s",     "mdi:timer-outline",       "total_increasing");
    mqttPublishDiscoveryOne("system_free_heap_b",        "RAM Libera",                "data_size",            "B",     "mdi:memory",              "measurement");
    mqttPublishDiscoveryOne("fw_build_version",          "Firmware Build",            NULL,                   NULL,    "mdi:chip",                NULL);
    mqttPublishDiscoveryOne("system_mqtt_reconnect_count","MQTT Riconnessioni",        NULL,                   NULL,    "mdi:swap-horizontal",     "total_increasing");
}

// ============================================================================
// On-Change Publish Threshold Cache · evita publish di valori stabili
// ============================================================================
typedef struct {
    // SHT31-D (9+1 heater)
    float    f_sht31_t, f_sht31_rh, f_sht31_dp, f_sht31_fp, f_sht31_wb;
    float    f_sht31_vpd, f_sht31_ah, f_sht31_hi, f_sht31_hx;
    uint32_t u_sht31_heater_on;

    // BME280 (3 + 2 press deriv rimossi)
    float    f_bme_t, f_bme_p, f_bme_h;

    // Indici termici coerenti outdoor (fonte SHT31 T+RH)
    float    f_wind_chill, f_appt;

    // Luce (4 base calibrati)
    float    f_lux, f_uvi;
    char     s_light_class[32], s_uv_level[32];

    // Vento (4 base + gust max)
    // f_wind_kmh RIMOSSO (dati duplicati in un diversa unità, ora calcolabile da HA)
    float    f_wind_ms, f_wind_gust_max_ms;
    uint32_t u_hall_count;
    char     s_beaufort_name[32];

    // Bussola (2)
    float    f_heading_deg;
    char     s_rose16[8];

    // Pioggia (6)
    uint32_t u_rain_tick_count;
    uint32_t u_rain_last_age_s;
    float    f_rain_total, f_rain_today, f_rain_rate;
    float    f_rain_last_age_s_f;
    char     s_rain_class[32];

    // Stato sistema
    uint32_t u_status_bits;
    char     s_status_list[128];

    // Sistema e Rete (NUOVI M1-M4-M6-M7)
    int32_t  i_wifi_rssi_dbm;
    uint32_t u_system_uptime_s;
    uint32_t u_system_free_heap_b;
    char     s_fw_build_version[32];
    char     s_wifi_ip_address[24];
    uint32_t u_system_mqtt_reconnect_count;
} LastPublishedValues;

static LastPublishedValues s_lastPub = {
    // SHT31
    .f_sht31_t  = NAN, .f_sht31_rh = NAN, .f_sht31_dp = NAN, .f_sht31_fp = NAN, .f_sht31_wb = NAN,
    .f_sht31_vpd = NAN, .f_sht31_ah = NAN, .f_sht31_hi = NAN, .f_sht31_hx = NAN,
    .u_sht31_heater_on = UINT32_MAX,
    // BME280
    .f_bme_t = NAN, .f_bme_p = NAN, .f_bme_h = NAN,
    // Indici termici combinati
    .f_wind_chill = NAN, .f_appt = NAN,
    // Luce
    .f_lux = NAN, .f_uvi = NAN,
    .s_light_class = {0}, .s_uv_level = {0},
    // Vento
    .f_wind_ms = NAN, .f_wind_gust_max_ms = NAN,
    .u_hall_count = UINT32_MAX,
    .s_beaufort_name = {0},
    // Bussola
    .f_heading_deg = NAN,
    .s_rose16 = {0},
    // Pioggia
    .u_rain_tick_count = UINT32_MAX,
    .u_rain_last_age_s = UINT32_MAX,
    .f_rain_total = NAN, .f_rain_today = NAN, .f_rain_rate = NAN,
    .f_rain_last_age_s_f = NAN,
    .s_rain_class = {0},
    // Stato sistema
    .u_status_bits = UINT32_MAX,
    .s_status_list = {0},
    // Sistema e Rete
    .i_wifi_rssi_dbm = INT32_MAX,
    .u_system_uptime_s = UINT32_MAX,
    .u_system_free_heap_b = UINT32_MAX,
    .s_fw_build_version = {0},
    .s_wifi_ip_address = {0},
    .u_system_mqtt_reconnect_count = UINT32_MAX,
};

static inline bool changedFloat(float oldV, float newV, float thresh) {
    if (!isfinite(newV)) return false;
    if (!isfinite(oldV)) return true;
    return fabsf(newV - oldV) >= thresh;
}
static inline bool changedPctFloat(float oldV, float newV, float pctThresh) {
    if (!isfinite(newV)) return false;
    if (!isfinite(oldV)) return true;
    if (oldV == 0.0f) return (fabsf(newV) > 0.0001f);
    return (fabsf(newV - oldV) / fabsf(oldV)) * 100.0f >= pctThresh;
}
static inline bool changedU32(uint32_t oldV, uint32_t newV) {
    return (oldV == UINT32_MAX) || (oldV != newV);
}
static inline bool changedStr(const char* oldS, const char* newS) {
    if (!newS) return false;
    if (!oldS || oldS[0] == '\0') return true;
    return strcmp(oldS, newS) != 0;
}

// ============================================================================
// Wrapper FAST-TRACK vento — DISATTIVATO (BUG#2/4)
//   Il fast-track a 4Hz duplicava wind_chill/apparent_temp/Beaufort generando
//   ~92k msg/h extra. Sostituito con soglie on-change in Gruppo A (vento) e
//   Gruppo B (indici termici a 5s). Le funzioni rimangono come stub vuoti per
//   non rompere riferimenti durante il refactor; lo scheduler nel loop è stato
//   rimosso (cfr. lastMqttWindFastMs).
// ============================================================================
static void mqttPublishWindFastGrpB(bool force) { (void)force; }
static void mqttPublishWindFastGrpC(bool force) { (void)force; }

// ============================================================================
// Gruppo A · Alta dinamica (4 Hz / 250ms)
//   Direzione vento (heading) + velocità vento
//   Soglie on-change: Heading 2°, WindSpeed 0.005 m/s ≈ 0.018 km/h
//   NOTA: NON pubblica più SEMPRE 6 campi; usa soglie per evitare 86.4k msg/h
//         a vento fermo (BUG#4).
// ============================================================================
static void mqttPublishGrpA(bool force) {
    // windKmh locale non più necessario in GrpA (rimosso publish km/h duplicato);
    // GrpB calcola la propria copia locale per wind_chill/apparent_temp.
    uint8_t bf = beaufortOf(g_snap_wind_ms);
    const char* bn = beaufortName(bf);

    bool do_hdg     = force || changedFloat(s_lastPub.f_heading_deg, g_snap_heading_deg, THRESH_HEADING_DEG);
    bool do_rose16  = force || changedStr(s_lastPub.s_rose16, rose16Of(g_snap_heading_deg));

    const bool wind_changed_master = force || changedFloat(s_lastPub.f_wind_ms, g_snap_wind_ms, 0.005f);

    bool do_wms     = wind_changed_master;
    bool do_bname   = force || changedStr(s_lastPub.s_beaufort_name, bn);

    if (do_hdg)    { mqttPublishScalar("compass_deg", g_snap_heading_deg, 3);
                     Serial.printf_P(PSTR("[MQTT-A] Pub compass_deg = %.3f°  (force=%d, lastPub=%.3f, Δ=%.3f, soglia=%.1f)\n"),
                                     (double)g_snap_heading_deg, (int)force,
                                     isfinite(s_lastPub.f_heading_deg)?(double)s_lastPub.f_heading_deg:-999.999,
                                     (isfinite(s_lastPub.f_heading_deg)&&isfinite(g_snap_heading_deg))
                                        ?(double)fabsf(g_snap_heading_deg-s_lastPub.f_heading_deg):-999.999,
                                     (double)THRESH_HEADING_DEG);
                     s_lastPub.f_heading_deg = g_snap_heading_deg; }
    if (do_rose16) { const char* r = rose16Of(g_snap_heading_deg); mqttPublishScalarStr("compass_rose16", r);
                     strncpy(s_lastPub.s_rose16, r, sizeof(s_lastPub.s_rose16)-1); }
    if (do_wms)    { mqttPublishScalar("wind_speed_ms",  g_snap_wind_ms, 3); s_lastPub.f_wind_ms = g_snap_wind_ms; }
    // wind_speed_kmh RIMOSSO (stessa grandezza wind_speed_ms in unità diversa;
    //   HA fa la conversione automaticamente per device_class wind_speed)
    if (do_bname)  { mqttPublishScalarStr("wind_beaufort_name", bn);
                     strncpy(s_lastPub.s_beaufort_name, bn, sizeof(s_lastPub.s_beaufort_name)-1); }
}

// ============================================================================
// Gruppo B · Ambientali (0.2 Hz / 5s + on-change)
//   SHT31 (9 indici termici COERENTI outdoor), BME280 T/P/RH modulo,
//   wind_chill + apparent_temp (SHT31 COERENTE, non mixed),
//   LTR390 lux/UV (solo valori calibrati con unità, no raw/setup),
//   vento: raffica max 10min (M8), pioggia: rate
// ============================================================================
static void mqttPublishGrpB(bool force) {
    float windKmh = g_snap_wind_ms * 3.6f;
    unsigned long now = millis();

    bool pressure_changed = force || changedFloat(s_lastPub.f_bme_p, g_snap_bme_p, THRESH_PRESSURE_HPA);
    bool sht_t_changed    = force || changedFloat(s_lastPub.f_sht31_t, g_snap_sht31_t, THRESH_TEMP_C);
    bool sht_rh_changed   = force || changedFloat(s_lastPub.f_sht31_rh, g_snap_sht31_rh, THRESH_RH_PCT);

    // --- SHT31 (9 sensori · già coerenti T+RH dallo stesso sensore) ---
    if (isfinite(g_snap_sht31_t)  && (sht_t_changed  || force)) { mqttPublishScalar("sht31_temp_c", g_snap_sht31_t, 3); s_lastPub.f_sht31_t = g_snap_sht31_t; }
    if (isfinite(g_snap_sht31_rh) && (sht_rh_changed || force)) { mqttPublishScalar("sht31_humidity_rh", g_snap_sht31_rh, 2); s_lastPub.f_sht31_rh = g_snap_sht31_rh; }
    if (isfinite(g_snap_sht31_dp) && (force || changedFloat(s_lastPub.f_sht31_dp, g_snap_sht31_dp, THRESH_TEMP_C))) { mqttPublishScalar("sht31_dew_point_c", g_snap_sht31_dp, 2); s_lastPub.f_sht31_dp = g_snap_sht31_dp; }
    if (isfinite(g_snap_sht31_fp) && (force || changedFloat(s_lastPub.f_sht31_fp, g_snap_sht31_fp, THRESH_TEMP_C))) { mqttPublishScalar("sht31_frost_point_c", g_snap_sht31_fp, 2); s_lastPub.f_sht31_fp = g_snap_sht31_fp; }
    if (isfinite(g_snap_sht31_wb) && (force || changedFloat(s_lastPub.f_sht31_wb, g_snap_sht31_wb, THRESH_TEMP_C))) { mqttPublishScalar("sht31_wet_bulb_c", g_snap_sht31_wb, 2); s_lastPub.f_sht31_wb = g_snap_sht31_wb; }
    if (isfinite(g_snap_sht31_vpd) && (force || changedFloat(s_lastPub.f_sht31_vpd, g_snap_sht31_vpd, THRESH_VPD_KPA))) { mqttPublishScalar("sht31_vpd_kpa", g_snap_sht31_vpd, 3); s_lastPub.f_sht31_vpd = g_snap_sht31_vpd; }
    if (isfinite(g_snap_sht31_ah) && (force || changedFloat(s_lastPub.f_sht31_ah, g_snap_sht31_ah, 0.2f))) { mqttPublishScalar("sht31_humidity_absolute", g_snap_sht31_ah, 3); s_lastPub.f_sht31_ah = g_snap_sht31_ah; }
    if (isfinite(g_snap_sht31_hi) && (force || changedFloat(s_lastPub.f_sht31_hi, g_snap_sht31_hi, THRESH_TEMP_C))) { mqttPublishScalar("sht31_heat_index_c", g_snap_sht31_hi, 2); s_lastPub.f_sht31_hi = g_snap_sht31_hi; }
    if (isfinite(g_snap_sht31_hx) && (force || changedFloat(s_lastPub.f_sht31_hx, g_snap_sht31_hx, THRESH_TEMP_C))) { mqttPublishScalar("sht31_humidex", g_snap_sht31_hx, 2); s_lastPub.f_sht31_hx = g_snap_sht31_hx; }

    // --- BME280: Temperatura modulo (interna) ---
    if (isfinite(g_snap_bme_t) && (force || changedFloat(s_lastPub.f_bme_t, g_snap_bme_t, THRESH_TEMP_C))) {
        mqttPublishScalar("temp_module_c", g_snap_bme_t, 3); s_lastPub.f_bme_t = g_snap_bme_t;
    }

    // --- Pressione BME280: SOLO hPa (formato standard). mmhg/bar/delta/alt sono RIMOSSI (derivabili in HA) ---
    if (isfinite(g_snap_bme_p) && pressure_changed) {
        mqttPublishScalar("pressure_hpa", g_snap_bme_p, 2);
        s_lastPub.f_bme_p = g_snap_bme_p;
    }

    // --- BME280 Umidità Relativa (interna modulo, per confronto SHT31) ---
    bool bme_h_changed = force || changedFloat(s_lastPub.f_bme_h, g_snap_bme_h, THRESH_RH_PCT);
    if (isfinite(g_snap_bme_h) && bme_h_changed) {
        mqttPublishScalar("humidity_rh", g_snap_bme_h, 2); s_lastPub.f_bme_h = g_snap_bme_h;
    }

    // --- Wind Chill + Apparent Temperature (COERENTI SHT31 T+RH, no BME mixed) ---
    //   BUG#3 FIX: prima usavano T=SHT31 + RH=BME280 (accoppiato sbagliato).
    //   Adesso TUTTO da SHT31 + vento.
    bool anyCoherentThermal = sht_t_changed || sht_rh_changed ||
        (force || changedFloat(s_lastPub.f_wind_ms, g_snap_wind_ms, THRESH_WIND_MS));
    if (isfinite(g_snap_sht31_t) && isfinite(g_snap_wind_ms) && anyCoherentThermal) {
        float wc = windChillC(g_snap_sht31_t, windKmh);
        mqttPublishScalar("wind_chill_c", wc, 2); s_lastPub.f_wind_chill = wc;
    }
    if (isfinite(g_snap_sht31_t) && isfinite(g_snap_wind_ms) && isfinite(g_snap_sht31_rh) && anyCoherentThermal) {
        float at = apparentTempC(g_snap_sht31_t, windKmh, g_snap_sht31_rh);
        mqttPublishScalar("apparent_temp_c", at, 2); s_lastPub.f_appt = at;
    }

    // --- LTR390: SOLO lux + UV calibrati (no raw count, no gain/integ/res setup) ---
    // Watchdog 15min: se raw == 0 da oltre 15min, forza publish a zero
    const bool wd_zero_als =
        (g_ltr390_als_zero_ms != 0) && ((now - g_ltr390_als_zero_ms) >= LTR390_ZERO_WD_MS);
    const bool wd_zero_uv  =
        (g_ltr390_uv_zero_ms  != 0) && ((now - g_ltr390_uv_zero_ms)  >= LTR390_ZERO_WD_MS);

    bool ltr_lux_chg = force || wd_zero_als || changedPctFloat(s_lastPub.f_lux, g_snap_ltr_lux, THRESH_LUX_PCT);
    bool ltr_uvi_chg = force || wd_zero_uv  || changedFloat(s_lastPub.f_uvi, g_snap_ltr_uvi, THRESH_UVI);
    if (isfinite(g_snap_ltr_lux) && ltr_lux_chg) {
        mqttPublishScalar("ltr_lux", g_snap_ltr_lux, 2); s_lastPub.f_lux = g_snap_ltr_lux;
        const char* lc = ltr390_lightClassStr(g_snap_ltr_lux);
        if (force || wd_zero_als || changedStr(s_lastPub.s_light_class, lc)) {
            mqttPublishScalarStr("ltr_light_class", lc);
            strncpy(s_lastPub.s_light_class, lc, sizeof(s_lastPub.s_light_class)-1);
        }
    }
    if (isfinite(g_snap_ltr_uvi) && ltr_uvi_chg) {
        mqttPublishScalar("ltr_uv_index", g_snap_ltr_uvi, 2); s_lastPub.f_uvi = g_snap_ltr_uvi;
        const char* ul = ltr390_uvLevelStr(g_snap_ltr_uvi);
        if (force || wd_zero_uv || changedStr(s_lastPub.s_uv_level, ul)) {
            mqttPublishScalarStr("ltr_uv_level", ul);
            strncpy(s_lastPub.s_uv_level, ul, sizeof(s_lastPub.s_uv_level)-1);
        }
    }

    // --- Raffica Max 10 minuti (M8) · sliding window ---
    extern float g_windGustMax10Min;
    if (isfinite(g_windGustMax10Min) && (force || changedFloat(s_lastPub.f_wind_gust_max_ms, g_windGustMax10Min, 0.05f))) {
        mqttPublishScalar("wind_gust_max_ms", g_windGustMax10Min, 3);
        s_lastPub.f_wind_gust_max_ms = g_windGustMax10Min;
    }

    // --- Pioggia: rate e intensità pubblichiamo nel gruppo B ---
    bool rate_chg = force || changedFloat(s_lastPub.f_rain_rate, g_snap_rain_rate_mmh, 0.1f);
    if (isfinite(g_snap_rain_rate_mmh) && rate_chg) {
        mqttPublishScalar("rain_rate_mmh", g_snap_rain_rate_mmh, 2); s_lastPub.f_rain_rate = g_snap_rain_rate_mmh;
        const char* rc = rainClassWmoStr(g_snap_rain_rate_mmh);
        if (force || changedStr(s_lastPub.s_rain_class, rc)) {
            mqttPublishScalarStr("rain_intensity_class", rc);
            strncpy(s_lastPub.s_rain_class, rc, sizeof(s_lastPub.s_rain_class)-1);
        }
    }
}

// ============================================================================
// Gruppo C · Diagnostica (0.1 Hz / 10s + on-change per statici)
//   Stato sensori (2 rappresentazioni), conteggio impulsi anemometro,
//   età ultimo tick pioggia.
//   NUOVI (M1-M2-M3-M4-M6-M7-M9): RSSI WiFi, uptime, heap libero,
//   versione firmware build, IP WiFi, contatore riconnessioni MQTT,
//   stato heater SHT31 anti-condensa.
// ============================================================================
static void mqttPublishGrpC(bool force) {
    // Heading e Vento: pubblicati SOLO in Gruppo A (on-change soglia)
    // LTR gain/integ/res: rimossi (diagnostica driver non utile ad utente)
    // Rain mm per tick: rimosso (costante di calibrazione statica)

    // --- Conteggio impulsi hall anemometro (totalizzatore) ---
    if (force || changedU32(s_lastPub.u_hall_count, g_snap_hall_count)) {
        mqttPublishU32("anemometer_count", g_snap_hall_count); s_lastPub.u_hall_count = g_snap_hall_count;
    }

    // --- Pioggia: età ultimo tick ---
    if (force || changedU32(s_lastPub.u_rain_last_age_s, g_snap_rain_last_age_s)) {
        mqttPublishU32("rain_last_tick_age_s", g_snap_rain_last_age_s); s_lastPub.u_rain_last_age_s = g_snap_rain_last_age_s;
    }

    // --- Stato Sensori: 2 rappresentazioni (bitmask hex 32-bit + lista nomi) ---
    //   sensor_status_hexbin RIMOSSO (ridondanza: bitmask hex = stesso contenuto).
    if (force || changedU32(s_lastPub.u_status_bits, g_snap_status_bits)) {
        mqttPublishHexU32("sensor_status_bitmask", g_snap_status_bits);
        const char* sl = sensorStatusList(g_snap_status_bits);
        mqttPublishScalarStr("sensor_status_list", sl);
        s_lastPub.u_status_bits = g_snap_status_bits;
        strncpy(s_lastPub.s_status_list, sl, sizeof(s_lastPub.s_status_list)-1);
    }

    // --- NUOVO: Diagnostica Sistema e Rete ---
    // M1 WiFi RSSI dBm (Soglia 3 dBm)
    int32_t rssi = WiFi.RSSI();
    if (force || (s_lastPub.i_wifi_rssi_dbm == INT32_MAX) ||
        labs((long)s_lastPub.i_wifi_rssi_dbm - (long)rssi) >= 3) {
        mqttPublishScalar("wifi_rssi_dbm", (float)rssi, 0);
        s_lastPub.i_wifi_rssi_dbm = rssi;
    }
    // M6 WiFi IP Address (pubblica se cambia o force)
    String ipStr = WiFi.localIP().toString();
    if (force || (s_lastPub.s_wifi_ip_address[0] == '\0') ||
        strcmp(s_lastPub.s_wifi_ip_address, ipStr.c_str()) != 0) {
        mqttPublishScalarStr("wifi_ip_address", ipStr.c_str());
        strncpy(s_lastPub.s_wifi_ip_address, ipStr.c_str(), sizeof(s_lastPub.s_wifi_ip_address)-1);
    }
    // M2 Uptime (ogni 10s on-change sempre per totalizzatore monotono)
    uint32_t uptime_s = (uint32_t)(millis() / 1000UL);
    if (force || changedU32(s_lastPub.u_system_uptime_s, uptime_s)) {
        mqttPublishU32("system_uptime_s", uptime_s);
        s_lastPub.u_system_uptime_s = uptime_s;
    }
    // M3 Free Heap (soglia 512 byte)
    uint32_t heap = (uint32_t)ESP.getFreeHeap();
    if (force || (s_lastPub.u_system_free_heap_b == UINT32_MAX) ||
        ((uint32_t)labs((long)s_lastPub.u_system_free_heap_b - (long)heap) >= 512u)) {
        mqttPublishU32("system_free_heap_b", heap);
        s_lastPub.u_system_free_heap_b = heap;
    }
    // M4 Firmware Build Version (statica, pubblicata 1 volta dopo boot)
    extern const char g_fwBuildVersion[];
    if (force || (s_lastPub.s_fw_build_version[0] == '\0')) {
        mqttPublishScalarStr("fw_build_version", g_fwBuildVersion);
        strncpy(s_lastPub.s_fw_build_version, g_fwBuildVersion, sizeof(s_lastPub.s_fw_build_version)-1);
    }
    // M7 Contatore riconnessioni MQTT (total_increasing)
    extern uint32_t g_mqttReconnectCount;
    if (force || changedU32(s_lastPub.u_system_mqtt_reconnect_count, g_mqttReconnectCount)) {
        mqttPublishU32("system_mqtt_reconnect_count", g_mqttReconnectCount);
        s_lastPub.u_system_mqtt_reconnect_count = g_mqttReconnectCount;
    }
    // M9 Heater SHT31 ON/OFF state
    uint32_t htr_on = (g_sht31_heater_state != SHT31_HTR_OFF) ? 1u : 0u;
    if (force || changedU32(s_lastPub.u_sht31_heater_on, htr_on)) {
        mqttPublishU32("sht31_heater_on", htr_on);
        s_lastPub.u_sht31_heater_on = htr_on;
    }
}

// ============================================================================
// Gruppo D · Pioggia (event-driven su tick + ratifica gruppo B)
//   tick_count, total_mm, today_mm pubblicati ISTANTANEAMENTE sul tick HW
// ============================================================================
static void mqttPublishGrpDRain(bool force) {
    bool tick_chg   = force || changedU32(s_lastPub.u_rain_tick_count, g_snap_rain_tick_count);
    bool tot_chg    = force || changedFloat(s_lastPub.f_rain_total, g_snap_rain_total_mm, THRESH_RAIN_MM);
    bool today_chg  = force || changedFloat(s_lastPub.f_rain_today, g_snap_rain_today_mm, THRESH_RAIN_MM);

    if (tick_chg)  { mqttPublishU32("rain_tick_count", g_snap_rain_tick_count); s_lastPub.u_rain_tick_count = g_snap_rain_tick_count; }
    if (tot_chg && isfinite(g_snap_rain_total_mm)) { mqttPublishScalar("rain_total_mm", g_snap_rain_total_mm, 2); s_lastPub.f_rain_total = g_snap_rain_total_mm; }
    if (today_chg && isfinite(g_snap_rain_today_mm)) { mqttPublishScalar("rain_today_mm", g_snap_rain_today_mm, 2); s_lastPub.f_rain_today = g_snap_rain_today_mm; }
    // --- Diagnostica publish MQTT GrpD (solo quando pubblica qualcosa o force=true) ---
    if (force || tick_chg || tot_chg || today_chg) {
        Serial.printf_P(PSTR("[MQTT-GrpD] force=%u tick_chg=%u tot_chg=%u today_chg=%u → tick=%lu total=%.2fmm today=%.2fmm (snap)\r\n"),
            (unsigned)force, (unsigned)tick_chg, (unsigned)tot_chg, (unsigned)today_chg,
            (unsigned long)g_snap_rain_tick_count,
            (double)g_snap_rain_total_mm,
            (double)g_snap_rain_today_mm);
    }
}

// Backward compat wrapper (non più usato attivamente ma tenuto per safety)
static void mqttPublishAllData(void) {
    mqttPublishGrpA(true);
    mqttPublishGrpB(true);
    mqttPublishGrpC(true);
    mqttPublishGrpDRain(true);
}

// ============================================================================
// ArduinoOTA · Callback statiche + init helper
// ============================================================================
static void ota_arduino_onStart(void) {
    Serial.println(F("[OTA-Arduino] Inizio upload via IDE"));
    ota_bypassSupervisorBegin();
    if (ArduinoOTA.getCommand() == U_FLASH) {
        Serial.println(F("[OTA-Arduino] Tipo: sketch (firmware)"));
    } else {
        Serial.println(F("[OTA-Arduino] Tipo: filesystem (non supportato in questo FW)"));
    }
}

static void ota_arduino_onEnd(void) {
    Serial.println(F("[OTA-Arduino] Fine upload"));
    ota_bypassSupervisorEnd();
}

static void ota_arduino_onProgress(unsigned int progress, unsigned int total) {
    (void)total;
    static unsigned int lastPct = UINT_MAX;
    unsigned int pct = total ? (progress * 100U) / total : 0;
    // WDT feed ogni 2%
    ota_wdtFeedIfNeeded( (pct % 5) == 0 || pct != lastPct );
    if (pct != lastPct && (pct % 10) == 0) {
        Serial.printf("[OTA-Arduino] %u%%  (%u / %u byte)\r\n",
            pct, (unsigned)progress, (unsigned)total);
        lastPct = pct;
    }
}

static void ota_arduino_onError(ota_error_t err) {
    Serial.printf("[OTA-Arduino] ERRORE codice=%d\r\n", (int)err);
    if (g_ota_in_progress) ota_bypassSupervisorEnd();
}

static void ota_mdns_arduino_begin(void) {
    // Hostname MDNS-safe da device_name
    ota_sanitizeHostname(g_ota_hostname, sizeof(g_ota_hostname), g_config.device_name);
    Serial.printf("[OTA] hostname MDNS: \"%s\"\r\n", g_ota_hostname);

    // ---- MDNS ----
    bool mdnsOk = MDNS.begin(g_ota_hostname);
    if (mdnsOk) {
        MDNS.addService("http", "tcp", HTTP_PORT);
        MDNS.addServiceTxt("http", "tcp", "path", "/");
        MDNS.addService("arduino", "tcp", OTA_ARDUINO_PORT);
        Serial.printf("[MDNS] Avviato · %s.local + servizi http/%d arduino/%d\r\n",
            g_ota_hostname, HTTP_PORT, OTA_ARDUINO_PORT);
    } else {
        Serial.println(F("[MDNS] Fallito (RAM bassa o porta occupata) — proseguo senza MDNS"));
    }

    // ---- ArduinoOTA (solo se abbiamo IP) ----
    bool haveIp = (WiFi.status() == WL_CONNECTED) ||
                  (WiFi.getMode() & WIFI_AP);
    if (haveIp) {
        ArduinoOTA.setPort(OTA_ARDUINO_PORT);
        ArduinoOTA.setHostname(g_ota_hostname);
        // Nessuna password (come da decisione utente)
        ArduinoOTA.onStart(ota_arduino_onStart);
        ArduinoOTA.onEnd(ota_arduino_onEnd);
        ArduinoOTA.onProgress(ota_arduino_onProgress);
        ArduinoOTA.onError(ota_arduino_onError);
        ArduinoOTA.begin();
        Serial.printf("[OTA-Arduino] Pronto · porta UDP %d · host %s\r\n",
            OTA_ARDUINO_PORT, g_ota_hostname);
    } else {
        Serial.println(F("[OTA-Arduino] WiFi non pronto — salto init"));
    }
}

// ============================================================================
// [REGOLA OTA #5] Safe mode: contatore boot in RTC user memory (blocco 180,
//   lontano da word 0..90 usate dal FW e da eboot_command 128..159).
//   Azzerato dopo 60 s di uptime; al 4° boot consecutivo (3 reset entro 60 s
//   ciascuno) parte SOLO Wi-Fi + OTA/HTTP, senza I2C, sensori, ISR, MQTT.
//   Il power-on azzera il contatore (non è un crash-loop).
// ============================================================================
#define SAFEMODE_RTC_BLOCK   180
#define SAFEMODE_MAGIC       0x5AFE0B07UL
#define SAFEMODE_MAX_RESETS  3
#define SAFEMODE_CLEAR_MS    60000UL
struct SafeBootCnt_t { uint32_t magic; uint32_t count; };

static void safemode_store(uint32_t count) {
    SafeBootCnt_t bc = { SAFEMODE_MAGIC, count };
    system_rtc_mem_write(SAFEMODE_RTC_BLOCK, &bc, sizeof(bc));
}
static void safemode_evaluateAtBoot(void) {
    SafeBootCnt_t bc = { 0, 0 };
    system_rtc_mem_read(SAFEMODE_RTC_BLOCK, &bc, sizeof(bc));
    const rst_info* ri = ESP.getResetInfoPtr();
    if (bc.magic != SAFEMODE_MAGIC || bc.count > 100 || (ri && ri->reason == REASON_DEFAULT_RST)) bc.count = 0;
    g_safe_boot_count = bc.count + 1;
    safemode_store(g_safe_boot_count);
    g_safe_mode = (g_safe_boot_count > SAFEMODE_MAX_RESETS);
    Serial.printf("[SAFEMODE] boot consecutivi=%u reason=%u → %s\r\n",
        (unsigned)g_safe_boot_count, ri ? (unsigned)ri->reason : 99U,
        g_safe_mode ? "SAFE MODE (solo Wi-Fi+OTA)" : "normale");
}

// ============================================================================
// setup / loop
// ============================================================================
void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    safemode_evaluateAtBoot();
    delay(50);
    Serial.println();
    Serial.println(F("=============================================="));
    Serial.println(F("[BOOT-SYNC-MARKER] setup() INIZIO — attendi 2s"));
    Serial.print(F("[FW] Build: ")); Serial.println(FW_BUILD_STR);
    Serial.println(F("=============================================="));
    Serial.flush();
    delay(2000);
    Serial.flush();
    EEPROM.begin(EEPROM_SIZE_BYTES);
    if (!config_loadFromEeprom(&g_config)) {
        Serial.println(F("[EEPROM] Config non valida o vuota → defaults + salvo"));
        config_setDefaults(&g_config);
        config_sanitize(&g_config);
        config_saveToEeprom(&g_config);
    } else {
        bool changed = config_sanitize(&g_config);
        if (changed) {
            Serial.println(F("[EEPROM] Config sanitizzata (clamps), salvo"));
            config_saveToEeprom(&g_config);
        }
    }

    // --- Heading Filter: carica parametri da EEPROM offset 3072 ---
    hdrf_loadFromEEPROM();
    g_hdrf_initialized = true;

    // --- Sensor Filter Config (vento + rain rate): carica parametri EEPROM offset 3584 ---
    sfil_loadFromEEPROM();
    g_sfil_reset_ema_requested = true;

    crash_initRtcHeartbeat();
    crash_printResetReport();
    crash_spiffsStackWatermark();

    supervisor_init();
    supervisor_clearTrip();
    g_mqtt_discovery_sent = false;
    g_mqtt_config_changed = false;
    g_wifi_config_changed = false;

    // [REGOLA OTA #3] Wi-Fi + OTA + HTTP PRIMA di I2C/sensori: un sensore guasto non deve impedire l'OTA.
    //   (Tentativo a 100 kHz falliva sul BME280; ritentato con bus a 50 kHz, vedi I2C_CLOCK_HZ.)
    netStartSTAorAP();
    supervisor_wdtFeed();

    ota_mdns_arduino_begin();
    supervisor_wdtFeed();

    g_http.on("/",              HTTP_GET,  handleRoot);
    g_http.on("/api/labels",    HTTP_GET,  handleApiLabels);
    g_http.on("/api/config",    HTTP_GET,  handleApiConfigGet);
    g_http.on("/api/config",    HTTP_POST, handleApiConfigPost);
    g_http.on("/api/rain/reset",HTTP_POST, handleApiRainReset);
    g_http.on("/api/rain/reset",HTTP_GET,  [](){ g_http.send(405, "application/json", "{\"ok\":false,\"error\":\"method not allowed (POST expected)\"}"); });
    g_http.on("/api/restart",   HTTP_POST, handleApiRestart);
    g_http.on("/api/restart",   HTTP_GET,  [](){ g_http.send(405, "application/json", "{\"ok\":false,\"error\":\"method not allowed (POST expected)\"}"); });
    g_http.on("/debug/rtc_reset",HTTP_POST, handleDebugRtcReset);
    g_http.on("/debug/rtc_reset",HTTP_GET,  [](){ g_http.send(405, "application/json", "{\"ok\":false,\"error\":\"POST expected; chiama via curl o tool\"}"); });
    g_http.on("/api/i2cdiag",   HTTP_GET,  handleApiI2cDiag);
    g_http.on("/api/i2ctest",   HTTP_GET,  handleApiI2cTest);
    g_http.on("/update",        HTTP_GET,  handleOtaGet);
    g_http.on("/update",        HTTP_POST, handleOtaPostFinish, handleOtaUpload);
    g_http.begin();
    g_ws.begin();
    supervisor_wdtFeed();

    if (g_safe_mode) {
        Serial.println(F("[SAFEMODE] I2C, sensori, ISR e MQTT NON inizializzati"));
    } else {
    // --- I2C: diag pin + bus recovery (9 clock) + Wire.begin + scan ---
    i2c_diagPins("PRE-RECOVERY");
    (void)i2c_busRecover();
    i2c_diagPins("POST-RECOVERY");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    i2c_scanBus();

    {
        uint8_t gid = g_config.ltr390_gain_id;
        uint8_t rid = g_config.ltr390_res_it_id;
        if (gid > 4) gid = LTR390_GAIN_ID_DEFAULT;
        if (rid > 7) rid = LTR390_RES_IT_ID_DEFAULT;
        g_ltr390_gain_id = gid;
        g_ltr390_res_it_id = rid;
    }
    // --- Cache configurazione ISR: rain dead-time (min 150ms safety) ---
    {
        uint16_t rd = g_config.rain_dead_ms;
        if (rd < RAIN_DEAD_MS_DEF) rd = RAIN_DEAD_MS_DEF;
        noInterrupts();
        g_rain_isr_dead_ms = (uint32_t)rd;
        interrupts();
        Serial.printf_P(PSTR("[RAIN] ISR dead-time configurato: %u ms (EEPROM=%u, min=%u)\n"),
                          (unsigned)rd, (unsigned)g_config.rain_dead_ms, (unsigned)RAIN_DEAD_MS_DEF);
    }
    as5600_applyConfig();
    bme280_init();
    // LTR390_init() contiene Wire.begin() DUPLICATA → eseguirla PRIMA di SHT31,
    //  cosicché SHT31 init trova bus I2C stabile (non viene disturbato DOPO init OK).
    //  Subito dopo Wire.begin() in ltr390_init, sblocchiamo SDA se rimane bassa.
    g_ltr390_ok = ltr390_init();
    {
      char b[160];
      snprintf(b, sizeof(b),
        "[LTR390 SETUP] init_ret=%d g_ltr390_ok=%d gain_id=%u res_id=%u",
        (int)g_ltr390_ok, (int)g_ltr390_ok,
        (unsigned)g_ltr390_gain_id, (unsigned)g_ltr390_res_it_id);
      Serial.println(b);
    }
    // Sblocco sicuro SDA low (9-clock recovery) PRIMA di SHT31 (Sensirion è sensibile)
    (void)i2c_busRecover();
    delay(2);
    sht31_init();
    as3935_init();
    supervisor_wdtFeed();

    pinMode(HALL_PIN, INPUT_PULLUP);
    noInterrupts();
    g_hall_last_us    = 0;
    g_hall_period_us  = 0;
    g_hall_new_period = false;
    g_hall_count      = 0;
    interrupts();
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hall_isr, FALLING);

    pinMode(RAIN_PIN, INPUT_PULLUP);
    noInterrupts();
    g_rain_last_ms    = 0;
    g_rain_tick_count = 0;   // RAM resettata a 0; v3 rain_loadPersistedCounters RIPRISTINERÀ actual al cumulativo salvato.
    g_rain_new_tick   = false;
    interrupts();
    attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rain_isr, FALLING);
    rain_loadPersistedCounters();   // v3: ripristina automaticamente actual RAM = cum_saved + calcola tot_off
    // --- End setup rain: persistenza v3 caricata (rain_tick_count è già = cumulativo salvato) ---
    }   // fine blocco sensori (saltato in safe mode)

    Serial.print(F("HTTP su :"));
    Serial.println(HTTP_PORT);
    Serial.print(F("WS   su :"));
    Serial.println(WS_PORT);
    Serial.print(F("MQTT cfg "));
    Serial.print(g_config.mqtt_server[0] ? g_config.mqtt_server : "<non impostato>");
    Serial.print(':');
    Serial.print(g_config.mqtt_port ? g_config.mqtt_port : (uint16_t)1883);
    Serial.print(F("  device_id="));
    Serial.println(g_config.mqtt_device_id[0] ? g_config.mqtt_device_id : DEFAULT_MQTT_DEVICE_ID);
    supervisor_wdtFeed();
    ESP.wdtFeed();
}

void loop()
{
    static unsigned long lastSampleMs = 0;
    static unsigned long lastShtMs    = 0;
    static unsigned long lastBmeMs    = 0;
    static unsigned long lastLtrMs    = 0;
    static unsigned long lastOnlineRefreshMs = 0;
    static unsigned long lastNetReportMs = 0;
    float hall_smoothed_ms     = 0.0f;   // NON static: referenziato da extern in sfil_ per applyRuntime
    static float hall_last_valid_ms   = 0.0f;   // [FIX vento lento] ultimo valore calcolato da trigger valido
    static uint32_t hall_last_us_copy = 0;      // cache dell'ultimo g_hall_last_us (per detect NUOVO trigger)
    static uint32_t hall_period_us_hold = 0;    // [HOLD] ultimo periodo valido (riusato tra trigger)
    static float last_sht31_t         = NAN;
    static float last_sht31_rh        = NAN;
    static float last_bme_t           = NAN;
    static float last_bme_p           = NAN;
    static float last_bme_h           = NAN;
    static uint32_t last_ltr_als_raw  = 0;
    static uint32_t last_ltr_uv_raw   = 0;
    static float    last_ltr_lux      = NAN;
    static float    last_ltr_uvi      = NAN;
    static uint8_t  last_ltr_gainid   = 0;
    static uint8_t  last_ltr_resid    = 0;
    unsigned long now = millis();

    // --- Subsystems Safety-Critical polling (PRIMA del resto) ---
    supervisor_poll();
    crash_heartbeatPoll();
    netFallbackPollRetry();
    supervisor_wdtFeed();

    // --- STAMPELLA NONOS SDK: marca firmware "GOOD IDLE" dopo 60s + WiFi connesso
    //      Elimina definitivamente rollback bootloader dual-bank su qualsiasi crash futuro.
    {
        static bool s_upgrade_flag_stamped = false;
        if (!s_upgrade_flag_stamped &&
            millis() >= 60000UL &&
            WiFi.status() == WL_CONNECTED) {
            system_upgrade_flag_set(UPGRADE_FLAG_IDLE);
            s_upgrade_flag_stamped = true;
            Serial.println(F("[OTA-LOCK] Stampella NONOS SDK upgrade_flag = IDLE (NESSUN ROLLBACK MAI PIU')"));
        }
    }

    // --- OTA + MDNS polling (serviti spesso, no delay) ---
    ArduinoOTA.handle();
    {
        static unsigned long lastMdnsMs = 0;
        if ((int32_t)(now - lastMdnsMs) >= 300) {
            MDNS.update();
            lastMdnsMs = now;
        }
    }

    // --- Gestione flag g_wifi_config_changed: reboot dopo 2.5s per lasciare tempo alla risposta HTTP ---
    if (g_wifi_config_changed) {
        static unsigned long wifiRebootTs = 0;
        if (wifiRebootTs == 0) wifiRebootTs = now + 2500;
        if (now >= wifiRebootTs) {
            crash_capturePanicContext("WIFI_CONFIG_CHANGED_RESTART");
            delay(80);
            ESP.restart();
        }
    }

    // --- Gestione flag g_mqtt_config_changed: disconnect + forza re-discovery ---
    if (g_mqtt_config_changed) {
        g_mqtt_config_changed = false;
        if (g_mqtt.connected()) {
            Serial.println(F("[MQTT] Config cambiata → disconnect per riconnettere pulito"));
            g_mqtt.disconnect();
            delay(50);
        }
        g_mqtt_discovery_sent = false;
    }

    g_http.handleClient();
    g_ws.loop();

    // [REGOLA OTA #5] azzera il contatore boot dopo 60 s di uptime; in safe mode stop qui (solo Wi-Fi+OTA+HTTP)
    {
        static bool s_safe_cnt_cleared = false;
        if (!s_safe_cnt_cleared && millis() >= SAFEMODE_CLEAR_MS) {
            safemode_store(0);
            s_safe_cnt_cleared = true;
        }
    }
    if (g_safe_mode) return;

    g_mqtt.loop();
    mqtt_ensureConnected();
    if (g_mqtt.connected()) {
        g_mqtt_was_connected = true;
    }

    // State machine LTR390
    ltr390_poll();
    as3935_poll();

    // --- Report IP/Network ogni 10s (sempre visibile nel monitor seriale) ---
    if ((int32_t)(now - lastNetReportMs) >= 10000) {
        lastNetReportMs = now;
        const wl_status_t ws = WiFi.status();
        const uint8_t mode = WiFi.getMode();
        if (ws == WL_CONNECTED) {
            Serial.printf("[NET] STA SSID=%s  IP=%s  GW=%s  RSSI=%ddBm\r\n",
                WiFi.SSID().c_str(),
                WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str(),
                WiFi.RSSI());
        } else if ((mode & WIFI_AP)) {
            Serial.printf("[NET] AP SSID=%s  IP=%s  CH=%d\r\n",
                WiFi.softAPSSID().c_str(),
                WiFi.softAPIP().toString().c_str(),
                WiFi.channel());
        } else {
            Serial.printf("[NET] DISCONNECTED  mode=%d  status=%d\r\n", (int)mode, (int)ws);
        }
    }

    if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
        lastSampleMs = now;

        // ==================================================================
        // [NUOVO] Bussola AS5600 · FILTRO A 3 STADI
        //   STAGE 1 · Spike Rejection: campioni che deviano >soglia da ultimo valido → scartati
        //   STAGE 2 · Circular Moving Average · Media VETTORIALE N campioni (non aritmetica!)
        //   STAGE 3 · Output Decimation: aggiorna heading_deg solo ogni output_period_ms
        // ==================================================================
        float heading_deg = NAN;
        uint16_t rawAngle = 0xFFFF;   // sentinella "lettura fallita" usata a valle (status bit 0x01, raw snapshot)
        {
            // FIX 1.b: variabili diagnostica AS5600 reali (non hardcoded)
            uint8_t  as5600_status = 0;   // registro STATUS (0x0B)
            uint8_t  as5600_agc    = 0;   // registro AGC (0x1A, 1 byte)
            uint16_t as5600_mag    = 0;   // registro MAGNITUDE (0x1B, 12-bit)
            float    as5600_quality = NAN;

            // --- Lettura raw + correzione inversione asse + offset ---
            float rawDegWithOffset = NAN;
            // FIX 1.a: usa REG_RAWANGLE (0x0C, vero raw 0-4095 senza AS5600 slow filter) invece di REG_ANGLE (0x0E, gia' filtrato HW)
            bool readOk = (as5600_readReg16(AS5600_REG_RAWANGLE, &rawAngle) == 0);
            // FIX 1.bis: leggi STATUS/AGC/MAGNITUDE per qualita' magnete
            {
                uint8_t s8 = 0;
                if (as5600_readReg8(AS5600_REG_STATUS, &s8) == 0)  as5600_status = s8;
                if (as5600_readReg8(AS5600_REG_AGC,    &s8) == 0)  as5600_agc    = s8;
                uint16_t m16 = 0;
                if (as5600_readReg16(AS5600_REG_MAGNITUDE, &m16) == 0) as5600_mag = (uint16_t)(m16 & 0x0FFF);
            }
            // FIX 1.tris: calcolo QUALITY (2=OK 1=WARN 0=FAIL)
            // [FIX BUG #2 #3 #4 CRITICI] Soglie AGC e Magnitude troppo restrittive:
            //   · AGC basso (0..63) = magnete FORTE (OK!). Il vecchio check agc>=16 BLOCCAVA
            //     i moduli cinesi con magnete troppo vicino → quality=FAIL per sempre.
            //   · agcMin da ignorare sul lato basso (solo agcMax serve per magnete DEBOLE).
            //   · magMin default 1024 → troppo alto per breakout cinesi (stanno ~500-900).
            //   · I flag MH/ML dello status register hanno priorità sui numeri AGC.
            {
                const uint8_t  stMh   = (as5600_status & AS5600_ST_MH) ? 1U : 0U;  // troppo forte
                const uint8_t  stMl   = (as5600_status & AS5600_ST_ML) ? 1U : 0U;  // troppo debole
                const uint8_t  md_std = (as5600_status & AS5600_ST_MD) ? 1U : 0U;
                const uint8_t  md_alt = (as5600_status & 0x04U) ? 1U : 0U;
                const uint8_t  md     = (md_std | md_alt);
                // agcMin usato solo come "floor" se l'utente lo vuole; default 16 → ma noi
                //   lo forziamo a 0 perché AGC basso = campo forte = buono (non è un errore)
                uint8_t        agcMin = g_config.as5600_agc_min ? g_config.as5600_agc_min : (uint8_t)AS5600_AGC_MIN_DEF;
                if (agcMin > 16U) agcMin = 0U;   // utente può alzare, altrimenti permettiamo 0..255
                uint8_t        agcMax = g_config.as5600_agc_max ? g_config.as5600_agc_max : (uint8_t)AS5600_AGC_MAX_DEF;
                if (agcMax < 128U) agcMax = (uint8_t)AS5600_AGC_MAX_DEF;  // valore utente pazzo → default
                // magMin: se l'utente non l'ha personalizzato, rilassiamo a 512 per moduli cinesi
                uint16_t magMin = cfg_getAs5600MagMin(&g_config);
                if (magMin >= (uint16_t)AS5600_MAG_MIN_DEF) {
                    // se è al default 1024 o superiore, rilassa a 512 (breakout cinesi ~500-900)
                    magMin = 512U;
                }
                const uint16_t magThrWarn = (uint16_t)(magMin / 3U);
                (void)stMh;   // MH (troppo forte) non è bloccante, agcMin lo forziamo a 0
                (void)agcMin; // pertanto queste variabili non entrano nei check OK/WARN
                // OK: MD presente, AGC non saturo lato alto, MAG buona, NO flag ML
                bool ok   = (md != 0U) &&
                            (stMl == 0U) &&
                            (as5600_agc <= agcMax) &&
                            (as5600_mag >= magMin);
                // WARN: MD presente ma un po' borderline, o ha MH=troppo forte ma legge comunque
                bool warn = !ok && (md != 0U) &&
                            (as5600_mag >= magThrWarn);
                if (ok)        as5600_quality = AS5600_QUALITY_OK;    // 2.0
                else if (warn) as5600_quality = AS5600_QUALITY_WARN;  // 1.0
                else           as5600_quality = AS5600_QUALITY_FAIL;  // 0.0
            }

            // Calcolo heading raw (solo se readOk e qualita' non FAIL)
            // [FIX BUG #1] REG_RAWANGLE (0x0C) ha i 12-bit ALLINEATI A DESTRA (bits 11:0),
            //   quindi maschera 0x0FFF SENZA shift >> 4. Il vecchio >>4 buttava via 4 LSB
            //   e leggeva bits 15:4 producendo valori errati (256 livelli invece di 4096).
            if (readOk && isfinite(as5600_quality) && as5600_quality >= AS5600_QUALITY_WARN) {
                uint16_t raw12 = (uint16_t)(rawAngle & 0x0FFF);
                float rawDeg = ((float)raw12 * 360.0f) / 4096.0f;
                // Inversione asse Nord-Sud (E/W swap)
                rawDeg = 360.0f - rawDeg;
                if (rawDeg >= 360.0f) rawDeg = 0.0f;
                // Offset calibrazione
                int32_t off_100x = (int32_t)g_config.as5600_offset_deg_100x;
                if (off_100x != 0) {
                    rawDeg += (float)off_100x / 100.0f;
                    while (rawDeg <    0.0f) rawDeg += 360.0f;
                    while (rawDeg >= 360.0f) rawDeg -= 360.0f;
                }
                rawDegWithOffset = rawDeg;
            } else {
                // Qualita' FAIL o lettura NACK: raw non valido → non alimentare il filtro
                rawDegWithOffset = NAN;
            }
            g_hdrf_last_raw_in_deg = rawDegWithOffset;   // snapshot pre-filtro per diagnosi UI

            // ========================================================================
            // MASTER BYPASS: se filter_enable = 0 → salta TUTTI i 4 stadi
            // ========================================================================
            const bool filterOn = (g_hdrf_cfg.filter_enable != 0U);
            if (!filterOn) {
                heading_deg = rawDegWithOffset;
                g_hdrf_smoothed_deg    = rawDegWithOffset;
                g_hdrf_ema_out_deg     = rawDegWithOffset;
                g_hdrf_last_valid_deg  = rawDegWithOffset;
                g_hdrf_ema_initialized = false;
                g_hdrf_circbuf_fill    = 0U;
                g_hdrf_last_output_ms  = now;
                g_hdrf_last_valid_ms   = now;    // FIX 2: timestamp ultimo valido (anti-freeze)
                g_hdrf_consec_rejects  = 0;      // FIX 2: reset rejections consecutive
            } else {
            // --- Lettura parametri correnti ---
            const uint8_t  N_win   = (g_hdrf_cfg.movavg_window >= HDRFILT_WINDOW_MIN &&
                                      g_hdrf_cfg.movavg_window <= HDRFILT_WINDOW_MAX)
                                        ? g_hdrf_cfg.movavg_window : (uint8_t)HDRFILT_WINDOW_DEF;
            const float    thr_deg = (float)g_hdrf_cfg.spike_threshold_deg_x100 / 100.0f;
            const uint16_t out_ms  = (g_hdrf_cfg.output_period_ms >= HDRFILT_OUTPUT_MS_MIN &&
                                      g_hdrf_cfg.output_period_ms <= HDRFILT_OUTPUT_MS_MAX)
                                        ? g_hdrf_cfg.output_period_ms : (uint16_t)HDRFILT_OUTPUT_MS_DEF;
            const bool  spikeDisable = (thr_deg <= 0.001f || thr_deg >= (180.0f - 0.001f));
            const float thr_abs      = spikeDisable ? 360.0f : thr_deg;
            uint8_t ema_ax100 = (g_hdrf_cfg.ema_alpha_x100 >= HDRFILT_EMA_ALPHA_MIN_X100 &&
                                 g_hdrf_cfg.ema_alpha_x100 <= HDRFILT_EMA_ALPHA_MAX_X100)
                                     ? g_hdrf_cfg.ema_alpha_x100 : (uint8_t)HDRFILT_EMA_ALPHA_DEF_X100;
            const float ema_alpha = (float)ema_ax100 / 100.0f;
            const bool emaBypass = (ema_alpha <= 0.001f || ema_alpha >= 0.999f);

            // ========================================================================
            // STAGE 1 · SPIKE REJECTION
            // ========================================================================
            bool   sampleAccepted = false;
            float  acceptedSample = NAN;
            if (isfinite(rawDegWithOffset)) {
                // FIX 2: MAX-AGE 2s ANTI-FREEZE (20 samples @ 20Hz consecutive scartati + 2s = reset)
                if (isfinite(g_hdrf_last_valid_deg) &&
                    g_hdrf_consec_rejects >= 20U &&
                    ((now - g_hdrf_last_valid_ms) >= 2000UL)) {
                    Serial.printf_P(PSTR("[HDGF-SPIKE] MAX-AGE 2s REACHED #%lu: reset last_valid (%.2f° → accept %.2f°)\r\n"),
                        (unsigned long)g_hdrf_consec_rejects,
                        (double)g_hdrf_last_valid_deg,
                        (double)rawDegWithOffset);
                    g_hdrf_last_valid_deg = NAN;   // forza "primo campione"
                    g_hdrf_consec_rejects  = 0;
                }
                if (!isfinite(g_hdrf_last_valid_deg)) {
                    // Primo campione dopo boot (o reset MAX-AGE): accettato sempre
                    acceptedSample    = rawDegWithOffset;
                    sampleAccepted    = true;
                    g_hdrf_last_valid_deg = acceptedSample;
                    g_hdrf_last_valid_ms  = now;   // FIX 2: timestamp
                    g_hdrf_consec_rejects  = 0;    // FIX 2: reset
                } else {
                    float delta = circularDeltaDeg(rawDegWithOffset, g_hdrf_last_valid_deg);
                    if (fabsf(delta) <= thr_abs) {
                        // Delta entro soglia → accetta
                        acceptedSample    = rawDegWithOffset;
                        sampleAccepted    = true;
                        g_hdrf_last_valid_deg = acceptedSample;
                        g_hdrf_last_valid_ms  = now; // FIX 2
                        g_hdrf_consec_rejects  = 0;  // FIX 2
                    } else {
                        // Delta oltre soglia → SCARTA (spike)
                        g_hdrf_spike_reject_cnt++;
                        g_hdrf_consec_rejects++;   // FIX 2: contatore consecutivo
                        if ((g_hdrf_spike_reject_cnt % 50UL) == 1UL) {
                            Serial.printf_P(PSTR("[HDGF-SPIKE] REJECT #%lu [consec=%lu]: raw=%.2f°  last_valid=%.2f°  |Δ|=%.2f°  (soglia=%.2f°)\r\n"),
                                (unsigned long)g_hdrf_spike_reject_cnt,
                                (unsigned long)g_hdrf_consec_rejects,
                                (double)rawDegWithOffset,
                                (double)g_hdrf_last_valid_deg,
                                (double)fabsf(delta),
                                (double)thr_abs);
                        }
                        // Mantieni finestra piena: ultimo valido come "sample"
                        acceptedSample = g_hdrf_last_valid_deg;
                        sampleAccepted = true;
                    }
                }
            }

            // ========================================================================
            // STAGE 2 · CIRCULAR MOVING AVERAGE (MEDIA VETTORIALE)
            // ========================================================================
            if (sampleAccepted && isfinite(acceptedSample)) {
                g_hdrf_circbuf[g_hdrf_circbuf_head] = acceptedSample;
                g_hdrf_circbuf_head = (uint8_t)((g_hdrf_circbuf_head + 1U) % (uint8_t)N_win);
                if (g_hdrf_circbuf_fill < N_win) g_hdrf_circbuf_fill = (uint8_t)(g_hdrf_circbuf_fill + 1U);
                if (g_hdrf_circbuf_fill >= 2U) {
                    g_hdrf_smoothed_deg = circularVectorMeanDeg(g_hdrf_circbuf, g_hdrf_circbuf_fill);
                } else {
                    g_hdrf_smoothed_deg = acceptedSample;
                }
            }
            if (!isfinite(g_hdrf_smoothed_deg) && isfinite(g_hdrf_last_valid_deg)) {
                g_hdrf_smoothed_deg = g_hdrf_last_valid_deg;
            }

            // ========================================================================
            // STAGE 2.5 · EMA VETTORIALE (smoothing extra)
            // ========================================================================
            if (emaBypass) {
                g_hdrf_ema_out_deg     = g_hdrf_smoothed_deg;
                g_hdrf_ema_initialized = false;
            } else if (isfinite(g_hdrf_smoothed_deg)) {
                float r = g_hdrf_smoothed_deg * (float)(PI / 180.0);
                float s_new = sinf(r);
                float c_new = cosf(r);
                if (!g_hdrf_ema_initialized) {
                    g_hdrf_ema_s_sin = s_new;
                    g_hdrf_ema_s_cos = c_new;
                    g_hdrf_ema_initialized = true;
                } else {
                    const float om = 1.0f - ema_alpha;
                    g_hdrf_ema_s_sin = om * g_hdrf_ema_s_sin + ema_alpha * s_new;
                    g_hdrf_ema_s_cos = om * g_hdrf_ema_s_cos + ema_alpha * c_new;
                }
                float ema_r = atan2f(g_hdrf_ema_s_sin, g_hdrf_ema_s_cos);
                float ema_deg = ema_r * (float)(180.0 / PI);
                while (ema_deg <    0.0f) ema_deg += 360.0f;
                while (ema_deg >= 360.0f) ema_deg -= 360.0f;
                g_hdrf_ema_out_deg = ema_deg;
            }
            float out_stage_ema = isfinite(g_hdrf_ema_out_deg) ? g_hdrf_ema_out_deg : g_hdrf_smoothed_deg;

            // ========================================================================
            // STAGE 3 · OUTPUT DECIMATION
            // ========================================================================
            {
                static float s_last_published_deg = NAN;
                if (!isfinite(s_last_published_deg) && isfinite(out_stage_ema)) {
                    s_last_published_deg = out_stage_ema;
                    g_hdrf_last_output_ms = now;
                }
                if (isfinite(out_stage_ema) &&
                    ((now - g_hdrf_last_output_ms) >= (unsigned long)out_ms)) {
                    s_last_published_deg = out_stage_ema;
                    g_hdrf_last_output_ms = now;
                    // FIX 1: log seriale HDGF-OUT include anche AGC/MAG/QUALITY
                    {
                        static uint32_t cnt = 0;
                        cnt++;
                        if ((cnt % 20UL) == 0UL) {
                            Serial.printf_P(PSTR("[HDGF-OUT] ema=%.3f°  smoothed=%.3f°  window=%u  α=%.2f  rd=%s  agc=%u  mag=%u  q=%.1f  spike_cnt=%lu\r\n"),
                                (double)s_last_published_deg,
                                (double)g_hdrf_smoothed_deg,
                                (unsigned)g_hdrf_circbuf_fill,
                                (double)ema_alpha,
                                readOk ? "OK" : "READ_ERR",
                                (unsigned)as5600_agc,
                                (unsigned)as5600_mag,
                                (double)as5600_quality,
                                (unsigned long)g_hdrf_spike_reject_cnt);
                        }
                    }
                }
                heading_deg = s_last_published_deg;
            }
            } // end: if (filterOn)

            // --- Trace offset calibrato (se diverso da 0, ogni 0.1° cambiamento) ---
            {
                static int32_t  s_last_off_100x = INT32_MAX;
                static float    s_last_hdg_out  = NAN;
                int32_t off_100x = (int32_t)g_config.as5600_offset_deg_100x;
                if (isfinite(rawDegWithOffset) && s_last_off_100x != off_100x) {
                    Serial.printf_P(PSTR("[AS5600] Offset cambiato: offset_x100=%d  rawDeg_no_off=%.2f  heading_with_offset=%.2f\n"),
                                     (int)off_100x,
                                     (double)((off_100x!=0)?(double)((float)rawDegWithOffset-(float)off_100x/100.0f):(double)rawDegWithOffset),
                                     (double)rawDegWithOffset);
                    s_last_off_100x = off_100x;
                } else if (isfinite(heading_deg) && (!isfinite(s_last_hdg_out) || fabsf(circularDeltaDeg(heading_deg, s_last_hdg_out)) > 0.1f)) {
                    s_last_hdg_out = heading_deg;
                }
            }
            // FIX 3: Snapshot diagnostici AS5600 REALI (non piu' 0/2.0 hardcoded)
            g_snap_as5600_status   = as5600_status;
            g_snap_as5600_agc      = as5600_agc;
            g_snap_as5600_mag      = as5600_mag;
            g_snap_as5600_quality  = as5600_quality;
        }

        // --- Anemometro Hall A3144 ---
        float wind_ms = 0.0f;
        {
            uint32_t period_us_copy = 0;
            uint32_t last_us_copy   = 0;
            bool new_period_copy    = false;
            noInterrupts();
            period_us_copy   = g_hall_period_us;
            last_us_copy     = g_hall_last_us;
            new_period_copy  = g_hall_new_period;
            if (new_period_copy) g_hall_new_period = false;
            interrupts();

            uint32_t now_us = micros();
            uint32_t ms_since_pulse = (last_us_copy == 0 || now_us < last_us_copy)
                ? (HALL_WATCHDOG_MS + 1)
                : (now_us - last_us_copy) / 1000UL;

            // ---------------------------------------------------------------
            // [FIX Vento Lento]
            // 1) Trigger detection: consideriamo "nuovo dato" sia new_period_copy
            //    (ISR lo ha segnalato) SIA un cambio di last_us_copy rispetto
            //    alla cache locale hall_last_us_copy (safety-net anti-mancato
            //    flag in ISR in casi edge).
            bool is_new_trigger = new_period_copy || (last_us_copy != hall_last_us_copy);

            // 2) Signal OK = ultimo impulso entro HALL_WATCHDOG_MS (ora 5s)
            //    E periodo valido (anti-bounce). Tra trigger e trigger se
            //    ms_since_pulse < watchdog → siamo ancora nella "stessa"
            //    misura lenta → RIPETI ultimo valore (HOLD).
            bool watchdog_ok  = (ms_since_pulse < HALL_WATCHDOG_MS);
            bool period_ok    = (period_us_copy >= HALL_MIN_PERIOD_US) && (period_us_copy > 0);

            // 3) Se arriva un NUOVO trigger e periodo è ok → aggiorna
            //    hall_period_us_hold e ricalcola velocità (raw_ms).
            if (is_new_trigger && period_ok) {
                hall_period_us_hold = period_us_copy;
            }

            // 4) Calcolo velocità:
            //    - Nuovo trigger valido     → calcolo dal NUOVO periodo
            //    - Senza trigger ma WDT OK  → HOLD ultimo periodo valido
            //                                 (riusa hall_period_us_hold)
            //    - WDT scaduto              → 0 (calma)
            float raw_ms = 0.0f;
            uint32_t effective_period_us = 0;
            if (watchdog_ok && hall_period_us_hold >= HALL_MIN_PERIOD_US) {
                effective_period_us = hall_period_us_hold;
            } else if (is_new_trigger && period_ok) {
                effective_period_us = period_us_copy;
                hall_period_us_hold = period_us_copy;
            }
            if (effective_period_us >= HALL_MIN_PERIOD_US) {
                float period_s = (float)effective_period_us / 1000000.0f;
                float pulses_per_sec = 1.0f / period_s;
                uint8_t mags = (g_config.hall_magnets_per_rev >= 1 && g_config.hall_magnets_per_rev <= 32)
                              ? g_config.hall_magnets_per_rev : (uint8_t)HALL_MAGNETS_PER_REV;
                float r_m    = (g_config.hall_anemo_radius_m > 0.01f && g_config.hall_anemo_radius_m < 2.0f)
                              ? g_config.hall_anemo_radius_m : ANEMO_RADIUS_M;
                float k_cal  = (g_config.hall_anemo_calib_k >= 0.1f && g_config.hall_anemo_calib_k <= 5.0f)
                              ? g_config.hall_anemo_calib_k : ANEMO_CALIB_K;
                float rps = pulses_per_sec / (float)mags;
                raw_ms = rps * 2.0f * PI * r_m * k_cal;
            }

            // --- Runtime SFIL params (filtri vento) ---
            float hall_ema_f = (float)g_sfil_cfg.hall_ema_alpha_x1000 / 1000.0f;
            if (!(hall_ema_f >= 0.01f && hall_ema_f <= 1.00f)) hall_ema_f = (float)SFIL_HALL_EMA_DEF_X1000 / 1000.0f;
            float hall_decay_f = (float)g_sfil_cfg.hall_hold_decay_x10000 / 10000.0f;
            if (!(hall_decay_f >= 0.90f && hall_decay_f <= 0.9999f)) hall_decay_f = (float)SFIL_HALL_DECAY_DEF_X10000 / 10000.0f;
            if (g_sfil_reset_ema_requested) {
                hall_smoothed_ms = 0.0f;
                hall_last_valid_ms = 0.0f;
                hall_period_us_hold = 0;
            }
            // 5) Aggiornamento EMA → SOLO all'arrivo di un NUOVO trigger
            //    (filtro adattivo ai dati veri, non a tick del loop a 20Hz).
            if (is_new_trigger && period_ok) {
                hall_smoothed_ms = hall_smoothed_ms +
                                   hall_ema_f * (raw_ms - hall_smoothed_ms);
                hall_last_valid_ms = hall_smoothed_ms;
            } else if (watchdog_ok && hall_period_us_hold >= HALL_MIN_PERIOD_US) {
                // -------------------------------------------------------------
                // HOLD + DECAY LENTO: tra trigger (es 3s a vento lento)
                // invece di azzerare → scendiamo solo del (1-decay)% a tick (50ms)
                // curva LINEARE e COSTANTE, nessun "buchino" zero in UI/MQTT.
                // -------------------------------------------------------------
                float decay_target = hall_last_valid_ms * 0.85f;   // soft floor: min 85% ultimo vero
                hall_smoothed_ms = hall_smoothed_ms * hall_decay_f;
                if (hall_smoothed_ms < decay_target) hall_smoothed_ms = decay_target;
                // Ora ricalibra smoothed sul raw_ms (calcolato su ultimo periodo HOLDato):
                // blend minimo per non allontanarci troppo se il periodo è lo stesso
                hall_smoothed_ms = hall_smoothed_ms + 0.05f * (raw_ms - hall_smoothed_ms);
            } else {
                // WATCHDOG SCADUTO (nessun impulso per 5s) → VERO ZERO / CALMA.
                // Decay veloce per non mostrare vento quando non c'è.
                hall_smoothed_ms = hall_smoothed_ms * 0.70f;
                hall_last_valid_ms = hall_last_valid_ms * 0.50f;
                if (hall_smoothed_ms < 0.03f) {
                    hall_smoothed_ms   = 0.0f;
                    hall_last_valid_ms = 0.0f;
                    hall_period_us_hold = 0;
                }
            }

            // 6) Aggiorna cache locale dell'ultimo last_us per la prossima iterazione
            hall_last_us_copy = last_us_copy;

            wind_ms = hall_smoothed_ms;
        }

        // --- SHT31-D: Single-Shot ogni SHT31_SAMPLE_MS (1Hz default) ---
        if (now - lastShtMs >= SHT31_SAMPLE_MS) {
            lastShtMs = now;

            // --- [FIX 3: AUTO-RECOVERY RUNTIME SHT31] ogni 15s se sensore morto ---
            {
                static unsigned long s_sht31_last_reinit_ts = 0;
                static uint32_t      s_sht31_reinit_count    = 0;
                if (!g_sht31_ok && (now - s_sht31_last_reinit_ts) >= 15000UL) {
                    s_sht31_last_reinit_ts = now;
                    s_sht31_reinit_count++;
                    Serial.print(F("[SHT31 AUTO-RECOVERY] sensore OFFLINE da "));
                    Serial.print((unsigned long)(now - g_sht31_heater_ts));
                    Serial.print(F("ms · tentativo re-init #"));
                    Serial.print((unsigned long)s_sht31_reinit_count);
                    Serial.println(F(" · bus recovery + sht31_init()"));
                    (void)i2c_busRecover();
                    delay(5);
                    sht31_init();
                    if (g_sht31_ok) {
                        Serial.print(F("[SHT31 AUTO-RECOVERY] RIUSCITO al tentativo #"));
                        Serial.println((unsigned long)s_sht31_reinit_count);
                        s_sht31_reinit_count = 0;
                    }
                }
            }

            if (g_sht31_ok) {
                float st, srh;
                uint8_t r = sht31_singleShot(&st, &srh);
                if (r == 0 &&
                    isfinite(st) && st > -50.0f && st < 150.0f &&
                    isfinite(srh) && srh >= -5.0f && srh <= 105.0f) {
                    last_sht31_t  = st;
                    last_sht31_rh = srh;
                }
                // Heater FSM anti-ciclismo (sempre, anche se misura non valida mantiene stato)
                sht31_heater_fsm_step(last_sht31_rh);
            }
            // --- Calcoli indici derivati SHT31 (solo 1 Hz, NON 20 Hz nel loop master) ---
            const bool s_ok = g_sht31_ok &&
                isfinite(last_sht31_t) && last_sht31_t > -50.0f && last_sht31_t < 150.0f &&
                isfinite(last_sht31_rh) && last_sht31_rh >= 0.0f && last_sht31_rh <= 100.0f;
            if (s_ok) {
                float tc = last_sht31_t;
                float rh = last_sht31_rh;
                float pForWB = isfinite(last_bme_p) ? last_bme_p : NAN;
                s_sht_cache_t   = tc;
                s_sht_cache_rh  = rh;
                s_sht_cache_dp  = dewPointC(tc, rh);
                s_sht_cache_fp  = frostPointC(tc, rh);
                s_sht_cache_wb  = wetBulbStullC(tc, rh, pForWB);
                s_sht_cache_vpd = vaporPressureDeficitKPa(tc, rh);
                s_sht_cache_ah  = absHumidityGm3(tc, rh);
                s_sht_cache_hi  = heatIndexC(tc, rh);
                s_sht_cache_hx  = humidexOf(tc, rh);
                s_sht_cache_ok  = true;
                // Aggiorna snapshot globali in anticipo (MQTT / UI li useranno senza ricalcoli)
                g_snap_sht31_t  = s_sht_cache_t;
                g_snap_sht31_rh = s_sht_cache_rh;
                g_snap_sht31_dp = s_sht_cache_dp;
                g_snap_sht31_fp = s_sht_cache_fp;
                g_snap_sht31_wb = s_sht_cache_wb;
                g_snap_sht31_vpd = s_sht_cache_vpd;
                g_snap_sht31_ah = s_sht_cache_ah;
                g_snap_sht31_hi = s_sht_cache_hi;
                g_snap_sht31_hx = s_sht_cache_hx;
            } else {
                s_sht_cache_ok = false;
                s_sht_cache_t = s_sht_cache_rh = s_sht_cache_dp = s_sht_cache_fp = s_sht_cache_wb = NAN;
                s_sht_cache_vpd = s_sht_cache_ah = s_sht_cache_hi = s_sht_cache_hx = NAN;
                g_snap_sht31_t = g_snap_sht31_rh = g_snap_sht31_dp = g_snap_sht31_fp = g_snap_sht31_wb = NAN;
                g_snap_sht31_vpd = g_snap_sht31_ah = g_snap_sht31_hi = g_snap_sht31_hx = NAN;
            }
        }

        // --- BME280: lettura ogni BME280_SAMPLE_MS ---
        if (now - lastBmeMs >= BME280_SAMPLE_MS) {
            lastBmeMs = now;
            float bt, bp, bh;
            if (bme280_read(&bt, &bp, &bh)) {
                last_bme_t = bt;
                last_bme_p = bp;
                last_bme_h = bh;
            }
        }

        // --- LTR390: snapshot valori raw ogni LTR390_SAMPLE_MS ---
        if (now - lastLtrMs >= LTR390_SAMPLE_MS) {
            lastLtrMs = now;
            uint32_t als = 0, uv = 0;
            uint8_t gid = 0, rid = 0;
            bool als_new = false, uv_new = false;
            if (ltr390_read(&als, &uv, &gid, &rid, &als_new, &uv_new)) {
                if (als_new) last_ltr_als_raw = als;
                if (uv_new)  last_ltr_uv_raw  = uv;
                // Refresh config id sempre (non cambiano spesso ma leggiamoli)
                last_ltr_gainid = gid;
                last_ltr_resid  = rid;
                float g   = ltr390_gainFactor(gid);
                float ims = ltr390_integMs(rid);
                last_ltr_lux = ltr390_calcLux(last_ltr_als_raw, g, ims);
                last_ltr_uvi = ltr390_calcUVI(last_ltr_uv_raw,  g, ims);
            }
        }

        // --- SHT31-D: sentinel LETTO DA CACHE 1Hz (nessun ricalcolo nel loop master 20Hz) ---
        // (tutti i calcoli sono già stati eseguiti nel gate SHT31_SAMPLE_MS sopra)
        float sht31_t    = s_sht_cache_t;
        float sht31_rh   = s_sht_cache_rh;
        float sht31_dp   = s_sht_cache_dp;
        float sht31_fp   = s_sht_cache_fp;
        float sht31_vpd  = s_sht_cache_vpd;
        bool  sht31_htr_on = g_snap_sht31_htr_on;
        const bool sht31_ok_local = s_sht_cache_ok;
        // Variabili sentinella WS per SHT31 (5 float in coda al payload):
        // sht31_t_ws, sht31_rh_ws, sht31_dp_ws, sht31_fp_ws, sht31_vpd_ws
        // (wb, ah, hi, hx pubblicati via MQTT e UI HTML solo, non WS binary)
        float sht31_t_ws  = -1.0f;
        float sht31_rh_ws = -1.0f;
        float sht31_dp_ws = -1.0f;
        float sht31_fp_ws = -1.0f;
        float sht31_vpd_ws = -1.0f;
        if (sht31_ok_local) {
            sht31_t_ws  = sht31_t;
            sht31_rh_ws = sht31_rh;
            if (isfinite(sht31_dp))  sht31_dp_ws  = sht31_dp;
            if (isfinite(sht31_fp))  sht31_fp_ws  = sht31_fp;
            if (isfinite(sht31_vpd)) sht31_vpd_ws = sht31_vpd;
        }

        float bme_t   = isnan(last_bme_t) ? -127.0f : last_bme_t;
        float bme_p   = isnan(last_bme_p) ?    0.0f : last_bme_p;
        float bme_h   = isnan(last_bme_h) ?   -1.0f : last_bme_h;
        // Sentinel coerenti per LTR390: -1.0f = dato non disponibile.
        // Il check su lato client è `isFinite(x) && x >= 0`.
        // CH1 è inesistente sul LTR390: sempre sentinel -1.
        // [FIX F3] Guardia OK:
        // - g_ltr390_ok = init riuscito (PART_ID=0xB2 + I2C risponde)
        // - almeno 1 misura completata (als_cnt>0 OR uv_cnt>0) — DRDY passato
        // - lux/UVI calcolabili (isfinite e >= 0)
        // NOTA: last_ltr_als_raw == 0 è LEGITTIMO (ambiente completamente buio),
        // non deve invalidare lo stato OK del sensore.
        const bool ltr_ok_local = g_ltr390_ok && isfinite(last_ltr_lux) && isfinite(last_ltr_uvi) &&
                                  (last_ltr_lux >= -1e-6f) && (last_ltr_uvi >= -1e-6f) &&
                                  (g_ltr390_als_cnt > 0 || g_ltr390_uv_cnt > 0);
        float ltr_lux_f    = -1.0f;
        float ltr_uvi_f    = -1.0f;
        float ltr_als_f    = -1.0f;   // ex ltr_ch0
        float ltr_uv_f     = -1.0f;   // ex ltr_uv_raw
        float ltr_ch1_rsv  = -1.0f;   // CH1 inesistente: reserved, sempre NC
        if (ltr_ok_local) {
            ltr_lux_f = last_ltr_lux;
            ltr_uvi_f = last_ltr_uvi;
            ltr_als_f = (float)last_ltr_als_raw;
            ltr_uv_f  = (float)last_ltr_uv_raw;
            // ltr_ch1_rsv resta -1.0f
        }

        // --- Dati raw aggiuntivi per diagnostica completa ---
        float as5600_raw_f = (float)((rawAngle == 0xFFFF) ? 0U : ((uint16_t)(rawAngle & 0x0FFF)));
        float hall_period_f = 0.0f;
        float hall_count_f  = 0.0f;
        float sensor_status_f = 0.0f;
        {
            uint32_t period_us_copy = 0;
            uint32_t count_copy = 0;
            noInterrupts();
            period_us_copy = g_hall_period_us;
            count_copy     = g_hall_count;
            interrupts();
            hall_period_f = (float)period_us_copy;
            hall_count_f  = (float)count_copy;
        }

        // --- Rain Gauge: snapshot volatili + calcoli mm/tick ---
        uint32_t rain_ticks_copy = 0;
        uint32_t rain_lastms_copy = 0;
        bool     rain_newflag_copy = false;
        {
            noInterrupts();
            rain_ticks_copy    = g_rain_tick_count;
            rain_lastms_copy   = g_rain_last_ms;
            rain_newflag_copy  = g_rain_new_tick;
            if (rain_newflag_copy) g_rain_new_tick = false;
            interrupts();
        }
        static float    s_mmPerTick_cache = NAN;
        static uint32_t s_rain_ticks_snap = 0;
        static uint32_t s_rain_prev_tick_ms = 0;       // timestamp tick PRECEDENTE (per rate inter-tick)
        static uint32_t s_rain_today_first_ms = 0;     // timestamp PRIMO tick OGGI corrente (per tick #1 formula 1800s)
        static float    s_rain_rate_inst = NAN;        // intensità istantanea (NO EMA) — aggiornata AD OGNI TICK
        // Se i parametri in config cambiano, ricalcola mm/tick (validazione già fatta da sanitize)
        float expected_mm = rainCalcMmPerTick(g_config.rain_funnel_dia_mm,
                                               g_config.rain_tip_mass_g,
                                               g_config.rain_calib_k);
        if (!isfinite(s_mmPerTick_cache) ||
            fabsf(s_mmPerTick_cache - expected_mm) > 0.0001f) {
            s_mmPerTick_cache = expected_mm;
        }
        // Dead-time secondario da EEPROM: ignora tick arrivati prima di rain_dead_ms
        // (ISR usa hardcoded RAIN_DEAD_MS_DEF come floor minimo safety; qui
        //  effettuiamo un secondo filtro se l'utente ha configurato dead-time > default)
        uint32_t actual_ticks = rain_ticks_copy;
        // --- AUTO RESET MEZZANOTTE PIOGGIA OGGI (NTP epoch-day change detection) ---
        {
            uint32_t todayDay = rainGetTodayEpochDay();
            if (todayDay != 0 && todayDay != g_rain_today_day_epoch) {
                // giorno diverso → mezzanotte passata → reset pioggia oggi
                g_rain_today_day_epoch    = todayDay;
                g_rain_today_offset_ticks = actual_ticks;
                s_rain_today_first_ms     = 0;   // forza tick #1 formula 1800s per oggi nuovo
                // --- SALVATAGGIO IMMEDIATO EEPROM (GARANTITO ---
                //   Non si passa più per g_rain_persist_dirty → chiama direttamente;
                //   rain_writeBackIfNeeded rimane per hourly safety backup ogni 1h (safety).
                rainWritePersistImmediate("mezzanotte auto-reset oggi");
                g_mqtt_force_repub_all    = true;
                Serial.printf_P(PSTR("[RAIN] MEZZANOTTE → reset OGGI. today_day=%lu  today_off=%lu (tick attuali)\n"),
                    (unsigned long)todayDay, (unsigned long)actual_ticks);
            }
        }
        // (Dead-time logic placeholder — filtro principale rimane nella ISR)
        // Calcolo nuovo tick: intensità PUNTUALE inter-tick, NO EMA, aggiornata AD OGNI TICK
        if (actual_ticks != s_rain_ticks_snap) {
            uint32_t dticks = actual_ticks - s_rain_ticks_snap;
            // Nuovo giorno: tick #1 dell'evento oggi → usa dt=1800s come da formula
            uint32_t today_ticks_since_reset = actual_ticks - g_rain_today_offset_ticks;
            if (s_rain_ticks_snap == 0 || s_rain_today_first_ms == 0 || today_ticks_since_reset <= dticks) {
                s_rain_today_first_ms = now;   // primo tick oggi
                s_rain_prev_tick_ms   = now;
                // Tick #1 del giorno (o dopo reset totale o reset oggi mezzanotte):
                //   proiezione finestra 30 minuti (1800s) come da formula richiesta
                float dt_sec = 1800.0f;
                s_rain_rate_inst = (s_mmPerTick_cache * (float)dticks * 3600.0f) / dt_sec;
            } else {
                // Tick #N, N>=2: rate inter-tick preciso
                uint32_t dt_ms = (now > s_rain_prev_tick_ms) ? (now - s_rain_prev_tick_ms) : 1U;
                float dt_sec = (float)dt_ms / 1000.0f;
                if (dt_sec < 0.05f) dt_sec = 0.05f;   // floor 50ms safety
                s_rain_rate_inst = (s_mmPerTick_cache * (float)dticks * 3600.0f) / dt_sec;
                s_rain_prev_tick_ms = now;
            }
            if (!isfinite(s_rain_rate_inst) || s_rain_rate_inst < 0.0f) s_rain_rate_inst = 0.0f;
            s_rain_ticks_snap = actual_ticks;
            // ==================================================================
            // RICHIESTA UTENTE EEPROM AD OGNI TICK: salvataggio IMMEDIATO e
            //   DIRETTO (non tramite dirty flag indiretto).
            //   RainPersist_t contiene SOLO gli offset (tot_off/today_off/day),
            //   nessun valore cumulativo → NONE dei campi viene modificato da
            //   un TICK NORMALE. Però l'utente ha richiesto persistenza ad ogni
            //   tick → salviamo comunque (EEPROM: ~125 writes/25mm di pioggia
            //   = wear per 137 anni su AT24C32 100k cicli).
            //   g_rain_persist_dirty=true non serve più per tick ma lo settiamo
            //   comunque affinché rain_writeBackIfNeeded() (hourly safety) non
            //   sovrascriva se non c'è nulla di nuovo (è no-op).
            // ==================================================================
            g_rain_persist_dirty = true;
            rainWritePersistImmediate("tick bascula");
        }
        // Dopo aver gestito TUTTE le sezioni reset flag (vento + rain)
        if (g_sfil_reset_ema_requested) g_sfil_reset_ema_requested = false;
        // Calcola età ultimo tick in secondi
        uint32_t rain_age_s = 0;
        if (rain_lastms_copy != 0) {
            if (now >= rain_lastms_copy) rain_age_s = (now - rain_lastms_copy) / 1000u;
        }
        // Rain rate: dopo 10 min senza tick -> dimezza, dopo 30 min -> 0 mm/h (pioggia finita)
        if (actual_ticks > 0 && isfinite(s_rain_rate_inst)) {
            if (rain_age_s > 600u)  s_rain_rate_inst *= 0.5f;
            if (rain_age_s > 1800u) s_rain_rate_inst  = 0.0f;
        }
        // --- Contatori finali con OFFSET ---
        // Pioggia TOTALE (reset via MQTT cmd rain_reset da Home Assistant):
        //   conteggio = max(0, actual - total_offset)
        uint32_t total_ticks = (actual_ticks > g_rain_total_offset_ticks) ? (actual_ticks - g_rain_total_offset_ticks) : 0u;
        float rain_total_mm  = (float)total_ticks * s_mmPerTick_cache;
        // Pioggia OGGI (auto-reset OGNI MEZZANOTTE):
        uint32_t off_today = g_rain_today_offset_ticks;
        if (off_today > actual_ticks) off_today = 0;
        uint32_t today_ticks = actual_ticks - off_today;
        float rain_today_mm  = (float)today_ticks * s_mmPerTick_cache;
        // Se reset totale (total_offset appena cambiato) → re-seed first_tick_today per nuova baseline
        static uint32_t s_last_total_offset_seen = 0;
        static uint32_t s_last_today_offset_seen = 0;
        if (s_last_total_offset_seen != g_rain_total_offset_ticks ||
            s_last_today_offset_seen != g_rain_today_offset_ticks) {
            s_last_total_offset_seen = g_rain_total_offset_ticks;
            s_last_today_offset_seen = g_rain_today_offset_ticks;
            if (total_ticks == 0) s_rain_today_first_ms = 0;  // forzerà tick #1 al prossimo
            s_rain_today_first_ms = (today_ticks > 0) ? (rain_lastms_copy ? rain_lastms_copy : now) : 0u;
            s_rain_prev_tick_ms   = s_rain_today_first_ms;
        }
        float rain_rate_out = NAN;
        if (actual_ticks == 0) {
            rain_rate_out = 0.0f;
        } else if (isfinite(s_rain_rate_inst) && s_rain_rate_inst >= 0.0f) {
            rain_rate_out = s_rain_rate_inst;
        } else {
            rain_rate_out = 0.0f;
        }
        // Fix tautologia: sensore pioggia OK se ha ricevuto almeno 1 tick dalla boot
        // oppure l'ISR ha registrato un timestamp (rain_lastms_copy != 0) → hardware vivo
        bool rain_ok_local = (actual_ticks > 0) || (rain_lastms_copy != 0);
        // Aggiorna snapshot globali PRIMA di pubblicare MQTT/WSS → sempre coerenti
        // · tick_count = actual_ticks (conteggio ISR NON resettabile, total_increasing compat con HA Utility Meter)
        // · total/today mm = con offset (resettabili come da spec)
        g_snap_rain_tick_count  = actual_ticks;
        g_snap_rain_total_mm    = rain_total_mm;
        g_snap_rain_today_mm    = rain_today_mm;
        g_snap_rain_rate_mmh    = rain_rate_out;
        g_snap_rain_last_age_s  = rain_age_s;
        g_snap_rain_mm_per_tick = s_mmPerTick_cache;
        g_snap_rain_ok          = rain_ok_local;
        rain_writeBackIfNeeded();
        // --- Diagnostica PIAGGIO flood-safe (ogni 2000ms) · NON flood seriale ---
        {   static unsigned long s_last_rain_diag_ms = 0;
            if (now - s_last_rain_diag_ms >= 2000UL) {
                s_last_rain_diag_ms = now;
                uint32_t cum_disp = (actual_ticks > g_rain_total_offset_ticks) ?
                                    (actual_ticks - g_rain_total_offset_ticks) : 0u;
                Serial.printf_P(PSTR("[RAIN-DIAG] actual=%lu  tot_off=%lu  tot_tck=%lu (cum=%lu) → total_mm=%.3f | today_off=%lu today_tck=%lu → today_mm=%.3f | snap_total=%.3f snap_today=%.3f | mmpt=%.4f rate=%.2f\r\n"),
                    (unsigned long)actual_ticks,
                    (unsigned long)g_rain_total_offset_ticks,
                    (unsigned long)total_ticks,
                    (unsigned long)cum_disp,
                    (double)rain_total_mm,
                    (unsigned long)g_rain_today_offset_ticks,
                    (unsigned long)today_ticks,
                    (double)rain_today_mm,
                    (double)g_snap_rain_total_mm,
                    (double)g_snap_rain_today_mm,
                    (double)s_mmPerTick_cache,
                    (double)(isfinite(rain_rate_out)?rain_rate_out:-1.0));
            }
        }

        // --- MQTT EVENT-DRIVEN RAIN (tick appena ricevuto) → PUBBLICO DOPO snapshot ---
        if (rain_newflag_copy && g_mqtt.connected()) {
            mqttPublishGrpDRain(false);
            g_mqtt_lastGoodPubMs = now;
            static uint32_t s_last_serial_rain_pub_ms = 0;
            if ((now - s_last_serial_rain_pub_ms) >= 50UL) {
                s_last_serial_rain_pub_ms = now;
                Serial.printf_P(PSTR("[MQTT-R] Tick bascula → Pub tick_ISR=%lu (non reset) tot=%.2fmm oggi=%.2fmm rate=%.2fmm/h\r\n"),
                    (unsigned long)actual_ticks,
                    (double)rain_total_mm,
                    (double)rain_today_mm,
                    (double)(isfinite(rain_rate_out)?rain_rate_out:0.0));
            }
        }

        // WS rain sentinel floats (-1 = NA, >= 0 valido per UI DataView)
        //   · ticks = actual_ticks (NON resettabile → coerente con UI label "da avvio")
        //   · total/today mm = con offset (resettabili)
        float rain_ticks_ws    = (float)actual_ticks;
        float rain_total_ws    = (total_ticks > 0 || rain_total_mm >= 0.0f) ? rain_total_mm : -1.0f;
        float rain_today_ws    = rain_today_mm;
        float rain_rate_ws     = isfinite(rain_rate_out) && rain_rate_out >= -1e-6f ? rain_rate_out : -1.0f;
        float rain_age_ws      = (float)rain_age_s;
        float rain_mmpt_ws     = s_mmPerTick_cache;
        float rain_resv1_ws    = -1.0f;
        float rain_resv2_ws    = -1.0f;

        // --- Sensor status bitmask (uint16_t cast a float: backward compat mantissa >= 24bit) ---
        // bit0  0x001 AS5600 OK (raw != 0xFFFF)
        // bit1  0x002 RESERVED (ex AS3935 Lightning · RIMOSSO 2026-09-23)
        // bit2  0x004 BME280 OK (T/P/H validi)
        // bit3  0x008 Hall OK (periodo >= min_period or count > 0)
        // bit4  0x010 LTR390 OK (g_ltr390_ok e dati lux >= 0)
        // bit5  0x020 SHT31 OK (I2C + CRC + T/RH entro limiti)
        // bit6  0x040 SHT31 HEATER attivo (WARMUP o ON)
        // bit7  0x080 AS5600 quality OK (diag off, or quality >= 1.0 WARN+OK)
        // bit8  0x100 RAIN Gauge OK (ISR installata + pin Hall leggibile)
        uint16_t status_bits = 0;
        if (rawAngle != 0xFFFF) status_bits |= 0x01;
        // bit1 RESERVED (ex AS3935) · sempre 0
        if (sht31_ok_local) status_bits |= 0x20;
        if (sht31_htr_on)   status_bits |= 0x40;
        if (bme_t > -100.0f  && bme_t < 150.0f)  status_bits |= 0x04;
        if (bme_p > 300.0f  && bme_p < 1200.0f)  status_bits |= 0x04;
        if (bme_h >= 0.0f   && bme_h <= 100.0f)  status_bits |= 0x04;
        if (hall_period_f >= (float)HALL_MIN_PERIOD_US) status_bits |= 0x08;
        if (ltr_ok_local) status_bits |= 0x10;
        // bit7: AS5600 SEMPRE valido (nessun gating quality · lettura diretta raw)
        if (isfinite(heading_deg)) status_bits |= 0x80;
        // bit8: Rain Gauge Hall disponibile
        if (rain_ok_local) status_bits |= 0x100;
        sensor_status_f = (float)status_bits;

        // --- Output seriale SEMPLIFICATO · 1 riga / 1s ---
        // Formato: [DATA] <uptime_s>  <heading_deg>°  <wind_ms>m/s  <rain_tick>/<rain_mm>mm/<rain_rate>mm/h  SHT:<sht_t>°C/<sht_rh>%/htr:<0|1>  BME:<bme_t>°C/<bme_p>hPa/<bme_h>%  LTR:<lux>lx/<uvi>UVI  HALL:<hall_us>µs/cnt:<hall_cnt>  RAIN:ok=<0|1>  AS5600:raw=<raw>  STS:0x<status_hex>
        char lnbuf[320];
        const char* htrStr = sht31_ok_local ? (sht31_htr_on ? "1" : "0") : "-";
        const char* shtStr = sht31_ok_local ? "OK" : "NC";
        int nw = snprintf(lnbuf, sizeof(lnbuf),
            "[DATA] t=%lus  HDG=%+07.2f°  W=%06.3fm/s  RAIN:%lu/%06.2fmm/%05.2fmm/h  SHT:%s T=%05.2f°C/RH=%04.1f%%/htr=%s  BME:T=%05.2f°C/P=%07.2fhPa/RH=%04.1f%%  LTR:L=%07.1flx/UV=%04.1fuvi  AS5600:raw=%04u  HALL:%06luµs/cnt:%05lu  RAINok=%u  STS=0x%03X",
            (unsigned long)(now / 1000UL),
            (double)heading_deg,
            (double)wind_ms,
            (unsigned long)actual_ticks,
            (double)rain_total_mm,
            (double)(isfinite(rain_rate_out) && rain_rate_out >= -1e-6f ? rain_rate_out : 0.0f),
            shtStr,
            (double)(sht31_ok_local ? sht31_t : -999.0f),
            (double)(sht31_ok_local ? sht31_rh : 0.0f),
            htrStr,
            (double)(isfinite(bme_t) ? bme_t : -999.0f),
            (double)(isfinite(bme_p) ? bme_p : 0.0f),
            (double)(isfinite(bme_h) ? bme_h : 0.0f),
            (double)(isfinite(last_ltr_lux) ? last_ltr_lux : 0.0f),
            (double)(isfinite(last_ltr_uvi) ? last_ltr_uvi : 0.0f),
            (unsigned)(rawAngle == 0xFFFF ? 0 : ((uint16_t)(rawAngle & 0x0FFF))),
            (unsigned long)((uint32_t)hall_period_f),
            (unsigned long)((uint32_t)hall_count_f),
            (unsigned)(rain_ok_local ? 1 : 0),
            (unsigned)status_bits);
        if (nw > 0 && (size_t)nw >= sizeof(lnbuf)) {
            lnbuf[sizeof(lnbuf) - 1] = 0;
        }
        Serial.println(lnbuf);
        // AS3935 · RIMOSSO · nessun log seriale

        // --- Slot diagnostici AS5600 per WS: valori REALI da g_snap_as5600_* (FIX 3) ---
        const float as_status_f  = (float)g_snap_as5600_status;
        const float as_agc_f     = (float)g_snap_as5600_agc;
        const float as_mag_f     = (float)g_snap_as5600_mag;
        const float as_quality   = g_snap_as5600_quality;

        // --- Broadcast WS: 168 byte = 42 float32 LE (backward compat: 0..127 byte invariati) ---
        // Layout:
        //   offset   0=heading_deg         (invariato v1..v5)
        //   offset   4=wind_ms             (invariato v1..v5)
        //   offset   8=lightning_dist_km   (ex DS18_T, riuso slot v5)
        //   offset  12=bme_t               (invariato v1..v5)
        //   offset  16=bme_p               (invariato v1..v5)
        //   offset  20=bme_h               (invariato v1..v5)
        //   offset  24=as5600_raw          (invariato v1..v5)
        //   offset  28=hall_us_period      (invariato v1..v5)
        //   offset  32=hall_cnt            (invariato v1..v5)
        //   offset  36=status_bits         (invariato v1..v5)
        //   offset  40=ltr_lux_f           (invariato v1..v5)
        //   offset  44=ltr_uvi_f           (invariato v1..v5)
        //   offset  48=ltr_als_f           (invariato v1..v5)
        //   offset  52=ltr_uv_f            (invariato v1..v5)
        //   offset  56=ltr_ch1_rsv         (invariato v1..v5 · NC)
        //   ---- 60B fine vecchio frame v1 ----
        //   offset  60=as5600_status       (76B layout v2..v5)
        //   offset  64=as5600_agc          (76B layout v2..v5)
        //   offset  68=as5600_mag          (76B layout v2..v5)
        //   offset  72=as5600_quality      (76B layout v2..v5)
        //   ---- 76B fine frame v2 ----
        //   offset  76=sht31_t_c           (96B layout v3..v5 · T PRIMARIA)
        //   offset  80=sht31_rh_pct        (96B layout v3..v5)
        //   offset  84=sht31_dp_c          (96B layout v3..v5)
        //   offset  88=sht31_fp_c          (96B layout v3..v5)
        //   offset  92=sht31_vpd_kpa       (96B layout v3..v5)
        //   ---- 96B fine frame v3 ----
        //   offset  96=rain_ticks          (128B layout v4..v5 · Rain Gauge)
        //   offset 100=rain_total_mm       (128B layout v4..v5)
        // offset 104 → adesso è sempre PIOGGIA OGGI (prima era rain_event_mm per retrocompatibilità).
        // Aggiornato il nome anche in MQTT discovery + UI label HTML; offset 104 rimane invariato
        // per non rompere la compatibilità con pagine web caricate in cache (v4 frame).
        //   offset 108=rain_rate_mmh       (128B layout v4..v5)
        //   offset 112=rain_last_age_s     (128B layout v4..v5)
        //   offset 116=rain_mm_per_tick    (128B layout v4..v5)
        //   offset 120=rain_resv1          (128B layout v4..v5 · NC 0)
        //   offset 124=rain_resv2          (128B layout v4..v5 · NC 0)
        // ---- 128B fine frame v4 ----
        //   offset 128..164: Riusati per Heading Filter (AS3935 rimosso dal progetto)
        //     128 = hdgf_spike_reject_cnt   (float · conteggio picchi scartati da avvio)
        //     132 = hdgf_smoothed_deg       (float ° · uscita stage2 media vettoriale)
        //     136 = hdgf_buf_fill           (float · N campioni nel buffer circolare 0..win)
        //     140 = hdgf_spike_deg          (float ° · soglia spike attuale)
        //     144 = hdgf_window             (float · N finestra media configurata)
        //     148 = hdgf_out_ms             (float · periodo output ms)
        //     152 = hdgf_raw_in_deg         (float ° · post invert+offset, pre filtro stage1)
        //     156,160,164 = reserved (0.0 per compatibilita)
        //   ---- 168B fine frame v5 invariato · 168/4=42 float32 ----
        const uint8_t  h_win   = (g_hdrf_cfg.movavg_window >= HDRFILT_WINDOW_MIN &&
                                  g_hdrf_cfg.movavg_window <= HDRFILT_WINDOW_MAX)
                                    ? g_hdrf_cfg.movavg_window : (uint8_t)HDRFILT_WINDOW_DEF;
        const float    h_thr   = (g_hdrf_cfg.spike_threshold_deg_x100 >= HDRFILT_SPIKE_DEG_MIN_X100 &&
                                  g_hdrf_cfg.spike_threshold_deg_x100 <= HDRFILT_SPIKE_DEG_MAX_X100)
                                    ? (float)g_hdrf_cfg.spike_threshold_deg_x100 / 100.0f
                                    : (float)HDRFILT_SPIKE_DEG_DEF_X100 / 100.0f;
        const uint16_t h_outms = (g_hdrf_cfg.output_period_ms >= HDRFILT_OUTPUT_MS_MIN &&
                                  g_hdrf_cfg.output_period_ms <= HDRFILT_OUTPUT_MS_MAX)
                                    ? g_hdrf_cfg.output_period_ms : (uint16_t)HDRFILT_OUTPUT_MS_DEF;
        float lg_count_f   = (float)g_hdrf_spike_reject_cnt;
        float lg_energy_f  = g_hdrf_smoothed_deg;
        float lg_age_f     = (float)g_hdrf_circbuf_fill;
        float lg_nf_f      = h_thr;
        float lg_wd_f      = (float)h_win;
        float lg_srej_f    = (float)h_outms;
        float lg_tun_f     = g_hdrf_last_raw_in_deg;
        float lg_min_f     = (float)((g_hdrf_cfg.ema_alpha_x100 >= HDRFILT_EMA_ALPHA_MIN_X100 &&
                                      g_hdrf_cfg.ema_alpha_x100 <= HDRFILT_EMA_ALPHA_MAX_X100)
                                        ? g_hdrf_cfg.ema_alpha_x100 : HDRFILT_EMA_ALPHA_DEF_X100) / 100.0f;
        float lg_osc_f     = g_hdrf_ema_out_deg;   // uscita EMA vettoriale finale (heading smoothed 2x)
        float lg_ts_f      = 0.0f;   // reserved
        uint8_t buf[WS_PAYLOAD_BYTES];
        memset(buf, 0, sizeof(buf));
        memcpy(buf +   0, &heading_deg,   sizeof(float));
        memcpy(buf +   4, &wind_ms,       sizeof(float));
        {   // offset 8: AS3935 distanza km (ex DS18 temperatura)
            float d = g_snap_lightning_distance_km;
            memcpy(buf +   8, &d, sizeof(float));
        }
        memcpy(buf +  12, &bme_t,         sizeof(float));
        memcpy(buf +  16, &bme_p,         sizeof(float));
        memcpy(buf +  20, &bme_h,         sizeof(float));
        memcpy(buf +  24, &as5600_raw_f,  sizeof(float));
        memcpy(buf +  28, &hall_period_f, sizeof(float));
        memcpy(buf +  32, &hall_count_f,  sizeof(float));
        memcpy(buf +  36, &sensor_status_f, sizeof(float));
        memcpy(buf +  40, &ltr_lux_f,     sizeof(float));
        memcpy(buf +  44, &ltr_uvi_f,     sizeof(float));
        memcpy(buf +  48, &ltr_als_f,     sizeof(float));
        memcpy(buf +  52, &ltr_uv_f,      sizeof(float));
        memcpy(buf +  56, &ltr_ch1_rsv,   sizeof(float));
        memcpy(buf +  60, &as_status_f,   sizeof(float));
        memcpy(buf +  64, &as_agc_f,      sizeof(float));
        memcpy(buf +  68, &as_mag_f,      sizeof(float));
        memcpy(buf +  72, &as_quality,    sizeof(float));
        memcpy(buf +  76, &sht31_t_ws,    sizeof(float));
        memcpy(buf +  80, &sht31_rh_ws,   sizeof(float));
        memcpy(buf +  84, &sht31_dp_ws,   sizeof(float));
        memcpy(buf +  88, &sht31_fp_ws,   sizeof(float));
        memcpy(buf +  92, &sht31_vpd_ws,  sizeof(float));
        memcpy(buf +  96, &rain_ticks_ws,    sizeof(float));
        memcpy(buf + 100, &rain_total_ws,    sizeof(float));
        memcpy(buf + 104, &rain_today_ws,    sizeof(float));
        memcpy(buf + 108, &rain_rate_ws,     sizeof(float));
        memcpy(buf + 112, &rain_age_ws,      sizeof(float));
        memcpy(buf + 116, &rain_mmpt_ws,     sizeof(float));
        memcpy(buf + 120, &rain_resv1_ws,    sizeof(float));
        memcpy(buf + 124, &rain_resv2_ws,    sizeof(float));
        memcpy(buf + 128, &lg_count_f,       sizeof(float));
        memcpy(buf + 132, &lg_energy_f,      sizeof(float));
        memcpy(buf + 136, &lg_age_f,         sizeof(float));
        memcpy(buf + 140, &lg_nf_f,          sizeof(float));
        memcpy(buf + 144, &lg_wd_f,          sizeof(float));
        memcpy(buf + 148, &lg_srej_f,        sizeof(float));
        memcpy(buf + 152, &lg_tun_f,         sizeof(float));
        memcpy(buf + 156, &lg_min_f,         sizeof(float));
        memcpy(buf + 160, &lg_osc_f,         sizeof(float));
        memcpy(buf + 164, &lg_ts_f,          sizeof(float));
        g_ws.broadcastBIN(buf, sizeof(buf));

        // --- Aggiorna snapshot per publish MQTT 1Hz ---
        g_snap_heading_deg    = heading_deg;
        g_snap_as5600_raw     = (uint16_t)((rawAngle == 0xFFFF) ? 0 : ((uint16_t)(rawAngle & 0x0FFF)));
        g_snap_wind_ms        = wind_ms;
        gustTickSample(wind_ms, now);   // M8: aggiorna sliding window raffica max 10min
        g_snap_hall_period_us = (uint32_t)hall_period_f;
        g_snap_hall_count     = (uint32_t)hall_count_f;
        if (bme_t  > -100.0f && bme_t  < 150.0f) g_snap_bme_t = bme_t;
        else g_snap_bme_t = NAN;
        if (bme_p  >= 300.0f && bme_p  <= 1200.0f) g_snap_bme_p = bme_p;
        else g_snap_bme_p = NAN;
        if (bme_h  >= 0.0f   && bme_h  <= 100.0f)  g_snap_bme_h = bme_h;
        else g_snap_bme_h = NAN;
        g_snap_ltr_als_raw  = last_ltr_als_raw;
        g_snap_ltr_uv_raw   = last_ltr_uv_raw;
        if (ltr_ok_local) {
            g_snap_ltr_lux = last_ltr_lux;
            g_snap_ltr_uvi = last_ltr_uvi;
        } else {
            g_snap_ltr_lux = NAN;
            g_snap_ltr_uvi = NAN;
        }
        g_snap_ltr_gain   = last_ltr_gainid;
        g_snap_ltr_res    = last_ltr_resid;

        // --- Watchdog LTR390 · 15min a zero → forza publish MQTT ---
        if (g_snap_ltr_als_raw == 0) {
            if (g_ltr390_als_zero_ms == 0) g_ltr390_als_zero_ms = now;
        } else {
            g_ltr390_als_zero_ms = 0;
        }
        if (g_snap_ltr_uv_raw == 0) {
            if (g_ltr390_uv_zero_ms == 0) g_ltr390_uv_zero_ms = now;
        } else {
            g_ltr390_uv_zero_ms = 0;
        }

        g_snap_status_bits   = status_bits;
    }

    // --- Publish MQTT Scheduler Multi-Fascia ---
    static unsigned long lastMqttGrpAMs = 0;
    static unsigned long lastMqttGrpBMs = 0;
    static unsigned long lastMqttGrpCMs = 0;
    if (g_mqtt.connected()) {
        bool anyPublished = false;
        // [BUG FIX heading offset] Consuma flag globale g_mqtt_force_repub_all:
        //   appena cambia un qualsiasi parametro runtime, forza publish
        //   IMMEDIATO dei 3 gruppi (non attende lo scheduler periodico)
        //   + passa force=true dentro ogni publish per superare soglie on-change.
        bool frc = g_mqtt_force_repub_all;
        if (frc) g_mqtt_force_repub_all = false;
        // Gruppo A (alta dinamica: 250ms / 4Hz · vento + direz + beaufort name)
        if (frc || (now - lastMqttGrpAMs >= (unsigned long)MQTT_GRP_A_PERIOD_MS)) {
            lastMqttGrpAMs = now;
            mqttPublishGrpA(frc);
            anyPublished = true;
        }
        // Gruppo B (ambientali: 5s / 0.2Hz)
        if (frc || (now - lastMqttGrpBMs >= (unsigned long)MQTT_GRP_B_PERIOD_MS)) {
            lastMqttGrpBMs = now;
            mqttPublishGrpB(frc);
            anyPublished = true;
        }
        // Gruppo C (diagnostica: 10s / 0.1Hz)
        if (frc || (now - lastMqttGrpCMs >= (unsigned long)MQTT_GRP_C_PERIOD_MS)) {
            lastMqttGrpCMs = now;
            mqttPublishGrpC(frc);
            anyPublished = true;
        }
        // --- FIX CRITICO: GrpD (Pioggia: tick/total/today) con force=true se frc. ---
        //   Senza questo fix: dopo reset HTTP/MQTT di pioggia totale, il flag
        //   g_mqtt_force_repub_all veniva consumato da A/B/C, ma GrpD rain
        //   veniva chiamato SOLO sul tick HW (event-driven). Quindi
        //   rain_total_mm (che ora è 0.00 mm) NON veniva ri-pubblicato su MQTT
        //   finché non arrivava un NUOVO tick hardware → Home Assistant vedeva
        //   per MINUTI/SEMPRE il valore vecchio (prima del reset). → Bug utente.
        //   Nota: GrpD in modalità event-driven (tick) è ancora usato in
        //   sezione rain loop per la publish immediata sul tick HW (false).
        if (frc) {
            mqttPublishGrpDRain(true);
            anyPublished = true;
        }
        // NOTA: WindFast track RIMOSSA (BUG#2/4) - wrapper sono stub vuoti.
        //       Prima 4Hz duplicava indici/Beaufort → 92k msg/h inutili.
        if (anyPublished) {
            g_mqtt_lastGoodPubMs = now;
        }
        if (frc) {
            Serial.println(F("[MQTT] Force publish globale eseguito (runtime config applicata)."));
        }
        // --- Availability topic refresh (Last Will complementare) ogni MQTT_ONLINE_REFRESH_MS ---
        if ((now - lastOnlineRefreshMs) >= (unsigned long)MQTT_ONLINE_REFRESH_MS) {
            lastOnlineRefreshMs = now;
            char avail[128]; mqttGetAvailTopic(avail, sizeof(avail));
            g_mqtt.publish(avail, "online", true);
        }
    }
}
