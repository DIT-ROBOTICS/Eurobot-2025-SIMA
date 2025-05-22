import os
import shutil

# Define parameters
output_dir = "build-x"  # Directory to store the output firmware files
config_file = "include/config.h"  # Path to the configuration file
platformio_command = "platformio run"  # Command to build the firmware

# Ensure the output directory exists
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Define the mapping of HOSTNAME and SIMA_NUM
sima_configs = [
    {"hostname": "DIT-SIMA-01", "sima_num": 1},
    {"hostname": "DIT-SIMA-02", "sima_num": 2},
    {"hostname": "DIT-SIMA-03", "sima_num": 3},
    {"hostname": "DIT-SIMA-04", "sima_num": 4},
    {"hostname": "DIT-SIMA-05", "sima_num": 1},
    {"hostname": "DIT-SIMA-06", "sima_num": 2},
    {"hostname": "DIT-SIMA-07", "sima_num": 3},
    {"hostname": "DIT-SIMA-08", "sima_num": 4},
]

def update_config_file(hostname, sima_num):
    """Update HOSTNAME and SIMA_NUM in the config.h file."""
    with open(config_file, "r") as file:
        lines = file.readlines()

    with open(config_file, "w") as file:
        for line in lines:
            if line.startswith("#define HOSTNAME"):
                file.write(f"#define HOSTNAME            \"{hostname}\"\n")
            elif line.startswith("#define SIMA_NUM"):
                file.write(f"#define SIMA_NUM            {sima_num}\n")
            else:
                file.write(line)

def build_firmware():
    """Execute the PlatformIO build command."""
    result = os.system(platformio_command)
    if result != 0:
        raise RuntimeError("Build failed")

def move_firmware_output(hostname):
    """Move and rename the generated firmware file."""
    build_dir = os.path.join(".pio", "build")  # Base build directory

    # Search for the firmware file in the build directory
    firmware_path = None
    for root, _, files in os.walk(build_dir):
        if "firmware.bin" in files:
            firmware_path = os.path.join(root, "firmware.bin")
            break

    if not firmware_path or not os.path.exists(firmware_path):
        raise FileNotFoundError(f"Firmware binary not found in {build_dir}. Ensure the build was successful.")

    new_filename = f"{hostname}.bin"  # Rename the firmware file based on the hostname
    destination_path = os.path.join(output_dir, new_filename)
    print(f"Moving firmware from {firmware_path} to {destination_path}")
    shutil.move(firmware_path, destination_path)

# Main process
for config in sima_configs:
    hostname = config["hostname"]  # Get the hostname for the current configuration
    sima_num = config["sima_num"]  # Get the SIMA_NUM for the current configuration

    print(f"Building firmware for {hostname} with SIMA_NUM {sima_num}...")

    # Update the configuration file
    update_config_file(hostname, sima_num)

    # Build the firmware
    build_firmware()

    # Move and rename the firmware file
    move_firmware_output(hostname)

# Restore the original config.h file
update_config_file("DIT-SIMA-00", 00)

print("All firmware builds completed successfully.")
