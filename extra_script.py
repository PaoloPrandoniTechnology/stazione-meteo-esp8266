import os
import shutil
import time

Import("env")

def on_build_firmware(source, target, env):
    project_dir = env.subst("$PROJECT_DIR")
    build_dir = env.subst("$BUILD_DIR")
    prog_name = env.subst("$PROGNAME")

    firmware_src = os.path.normpath(os.path.join(build_dir, prog_name + ".bin"))
    output_dir = os.path.normpath(os.path.join(project_dir, "firmware_output"))

    if not os.path.isdir(output_dir):
        try:
            os.makedirs(output_dir, exist_ok=True)
        except Exception as e:
            print("[BUILD][ERR] Impossibile creare la cartella di output %s: %s" % (output_dir, str(e)))
            return

    if not os.path.isfile(firmware_src):
        print("[BUILD][WARN] File firmware sorgente non trovato: %s" % firmware_src)
        return

    try:
        ts = time.strftime("%Y%m%d_%H%M%S")
        ts_name = "firmware_%s_%s.bin" % (prog_name, ts)
        latest_name = "firmware_latest.bin"

        dest_ts = os.path.normpath(os.path.join(output_dir, ts_name))
        dest_latest = os.path.normpath(os.path.join(output_dir, latest_name))

        shutil.copy2(firmware_src, dest_ts)
        shutil.copy2(firmware_src, dest_latest)

        size_src = os.path.getsize(firmware_src)
        print("")
        print("============================================================")
        print("[BUILD] .bin copiato in firmware_output/")
        print("       Timestamped  : %s (%d bytes)" % (ts_name, size_src))
        print("       Latest (sym) : %s (%d bytes)" % (latest_name, size_src))
        print("       Path completo: %s" % output_dir)
        print("============================================================")
        print("")
    except Exception as e:
        print("[BUILD][ERR] Fallita copia firmware: %s" % str(e))

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", on_build_firmware)
