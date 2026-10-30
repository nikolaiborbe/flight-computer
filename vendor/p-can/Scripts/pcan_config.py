#!/usr/bin/env python3

import sys
import struct
import subprocess
import shutil
import os

# The unique number used to confirm that a section is a P-CAN config
MAGIC_FIRST  = 0x5043414E # "PCAN"
MAGIC_SECOND = 0x434F4E46 # "CONF"

# Size of each magic number in bytes
MAGIC_SIZE = 4 # unsigned 32 bit integer

# Inverted values of the magic number
MAGIC_FIRST_INVERTED = ~MAGIC_FIRST & ((1 << MAGIC_SIZE * 8) - 1)
MAGIC_SECOND_INVERTED = ~MAGIC_SECOND & ((1 << MAGIC_SIZE * 8) - 1)

# The actual bytes values of the prefix and suffix
MAGIC_PREFIX = struct.pack("<II", MAGIC_FIRST, MAGIC_SECOND)
MAGIC_SUFFIX = struct.pack("<II", MAGIC_FIRST_INVERTED, MAGIC_SECOND_INVERTED)

# Sizes of the prefix and suffix of the config in bytes
PREFIX_SIZE = 2*MAGIC_SIZE + 4
SUFFIX_SIZE = 2*MAGIC_SIZE

# Temporary binary dump file path
TEMP_BIN_PATH = "./pcan_config_temp.bin"

# The address of the flash memory in the STM32
FLASH_ADDRESS = 0x8000000
FLASH_SIZE = 0x100000

def find_config_regions(binary: bytes) -> list[tuple[int, int, bytes]]:
    """
    Scan the provided binary data and returns a list of possible candidate regions for the config. 
    """
    regions = []

    offset = 0
    while (offset := binary.find(MAGIC_PREFIX, offset)) != -1:
        suffix_offset, = struct.unpack_from("<I", binary, offset + 2*MAGIC_SIZE)

        prefix_start = offset
        suffix_start = offset + suffix_offset

        data_start = prefix_start + PREFIX_SIZE
        data_size = suffix_start - data_start

        if (
            suffix_start + SUFFIX_SIZE <= len(binary) and 
            binary[suffix_start:suffix_start+SUFFIX_SIZE] == MAGIC_SUFFIX
        ):
            regions.append((
                offset + PREFIX_SIZE, 
                data_size, 
                binary[data_start:data_start+data_size],
            ))

        offset += 1

    return regions

def parse_int_or_die(value: str, minimum: int | None = None, maximum: int | None = None) -> int:
    try: 
        number = int(value)
    except ValueError:
        print(f"Input is not a valid integer value!")
        sys.exit(1)

    if (minimum is not None and number < minimum) or (maximum is not None and number > maximum):
        range_str = ""
        if minimum is not None and maximum is not None:
            range_str = f"[{minimum}, {maximum}]"
        elif minimum is not None:
            range_str = f"[{minimum}, ∞)"
        elif maximum is not None:
            range_str = f"(-∞, {maximum}]"
        print(f"Input must be in range {range_str}!")
        sys.exit(1)

    return number

def prompt_new_field_value(config: dict[str, int], key: str, minimum: int | None = None, maximum: int | None = None) -> None:
    value = input(f"{key} ({config[key]}): ").strip()
    if value:
        config[key] = parse_int_or_die(value, minimum, maximum)

def prompt_new_config(default: bytes) -> bytes:
    category, instance, _, _ = struct.unpack("<BBBB", default)

    config = {
        "Category": category,
        "Instance": instance,
    }

    print("Enter new values for config.")
    print("Leave blank for current value.")
    prompt_new_field_value(config, "Category", 0, 7)
    prompt_new_field_value(config, "Instance", 0, 7)

    return struct.pack("<BBBB", config["Category"], config["Instance"], 0, 0)

def read_stm32_binary(size: int, address: int | str = "0x8000000") -> bytes:
    """
    Reads `size` bytes from STM32 flash starting at `address` and returns it as bytes.
    """
    if isinstance(address, int):
        address = hex(address)

    if shutil.which("st-flash") is None:
        raise RuntimeError("st-flash not found in PATH")

    print(f"Reading {size} bytes from STM32 flash at {address}...")
        
    try:
        subprocess.run(
            ["st-flash", "read", TEMP_BIN_PATH, address, hex(size)],
            check=True,
            stderr=subprocess.PIPE,
        )

        with open(TEMP_BIN_PATH, "rb") as file:
            return file.read()
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to read STM32 flash: {e.stderr.decode(errors='replace')}")
    finally:
        try:
            os.unlink(TEMP_BIN_PATH)
        except FileNotFoundError:
            pass

def flash_stm32_binary(data: bytes, address: int | str = "0x8000000") -> None:
    """
    Flashes a bytes object `data` to STM32 flash starting at `address`.
    """
    if isinstance(address, int):
        address = hex(address)

    if shutil.which("st-flash") is None:
        raise RuntimeError("st-flash not found in PATH")

    print(f"Flashing {len(data)} bytes to STM32 flash at {address}...")
    try:
        with open(TEMP_BIN_PATH, "wb") as f:
            f.write(data)

        subprocess.run(
            ["st-flash", "write", TEMP_BIN_PATH, address],
            input=data,
            check=True,
            stderr=subprocess.PIPE,
        )
        print("Flashing completed successfully.")
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to flash STM32: {e.stderr.decode()}")
    finally:
        try:
            os.unlink(TEMP_BIN_PATH)
        except FileNotFoundError:
            pass

def main():
    binary_path = None

    if len(sys.argv) >= 2:
        binary_path = sys.argv[1]

    if binary_path:
        with open(binary_path, "rb") as f:
            binary = f.read()
    else:
        binary = read_stm32_binary(FLASH_SIZE, FLASH_ADDRESS)

    regions = find_config_regions(binary)

    if not regions:
        print("No config region found. :(")
        sys.exit(1)

        return

    if len(regions) > 1:
        print(f"Found {len(regions)} config region(s):")
        for index, (offset, length, _) in enumerate(regions):
            print(f"{index+1:>4}. Offset: 0x{offset:X}, Length: {length} bytes")

        print("Please select which to update.")
        selected_region = parse_int_or_die(input("Region index: ")) - 1
        if selected_region < 0 or selected_region >= len(regions):
            print("Entered regoin index does not exist!")
            sys.exit(1)
        region = regions[selected_region]
    else:
        region = regions[0]

    offset, length, current_data = region

    print(f"Updating config at offset 0x{offset:X}.")

    new_data = prompt_new_config(current_data)

    new_binary = binary[:offset] + new_data + binary[offset+length:]

    assert len(new_binary) == len(binary), "New binary must be same size as old one."

    if binary_path:
        with open(binary_path, "wb") as f:
            f.write(new_binary)
    else:
        flash_stm32_binary(new_binary, FLASH_ADDRESS)

    sys.exit(0)

if __name__ == "__main__":
    main()
