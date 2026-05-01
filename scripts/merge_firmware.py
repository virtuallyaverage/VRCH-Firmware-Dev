Import("env")

def merge_firmware(source, target, env):
    import subprocess, os

    platform = env.get("PIOPLATFORM")
    if platform == "espressif8266":
        return

    chip = {
        "espressif32": env.BoardConfig().get("build.mcu", "esp32")
    }.get(platform, "esp32")

    build_dir = env.subst("$BUILD_DIR")
    proj_dir  = env.subst("$PROJECT_DIR")
    out_dir   = os.path.join(proj_dir, "build_output", "merged")
    os.makedirs(out_dir, exist_ok=True)

    fw_name = env.subst("$PIOENV")

    if chip in ("esp32", "esp32s2", "esp32s3"):
        bootloader_offset = "0x1000" if chip == "esp32" else "0x0"
    elif chip in ("esp32c3", "esp32c6", "esp32h2"):
        bootloader_offset = "0x0"
    else:
        bootloader_offset = "0x1000"

    bootloader = os.path.join(build_dir, "bootloader.bin")
    partitions = os.path.join(build_dir, "partitions.bin")
    firmware   = os.path.join(build_dir, "firmware.bin")

    boot_app0_local = os.path.join(build_dir, "boot_app0.bin")
    if os.path.exists(boot_app0_local):
        boot_app0 = boot_app0_local
    else:
        boot_app0 = os.path.join(
            env.PioPlatform().get_package_dir("framework-arduinoespressif32"),
            "tools", "partitions", "boot_app0.bin"
        )

    output = os.path.join(out_dir, f"{fw_name}-merged.bin")

    flash_size = env.BoardConfig().get("upload.flash_size", "4MB")

    # Use PlatformIO's bundled esptool instead of system Python
    esptool = os.path.join(
        env.PioPlatform().get_package_dir("tool-esptoolpy"),
        "esptool.py"
    )

    subprocess.check_call([
        env.subst("$PYTHONEXE"), esptool,
        "--chip", chip,
        "merge_bin", "-o", output,
        "--flash_mode", "dio",
        "--flash_size", flash_size,
        bootloader_offset, bootloader,
        "0x8000", partitions,
        "0xe000", boot_app0,
        "0x10000", firmware,
    ])
    print(f"Merged firmware: {output}")

env.AddPostAction("$BUILD_DIR/firmware.bin", merge_firmware)