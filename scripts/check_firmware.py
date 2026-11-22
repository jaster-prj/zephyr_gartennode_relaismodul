#!/usr/bin/env python3
"""Check that a signed MCUboot image matches the linked application layout."""

import argparse
from pathlib import Path
import struct


def check_image(image, header_size, load_address, slot_size):
    if len(image) < 32:
        raise ValueError("truncated MCUboot header")
    magic, _, actual_header, _, body_size = struct.unpack_from("<IIHHI", image)
    if magic != 0x96F3B83D:
        raise ValueError("invalid MCUboot magic")
    if actual_header != header_size:
        raise ValueError("MCUboot header size does not match CONFIG_ROM_START_OFFSET")
    if len(image) > slot_size or header_size + body_size > len(image):
        raise ValueError("image does not fit the slot or is truncated")
    if body_size < 8:
        raise ValueError("missing application vector table")
    stack, reset = struct.unpack_from("<II", image, header_size)
    if not 0x20000000 < stack <= 0x20010000 or stack % 8:
        raise ValueError("invalid Gartennode initial stack pointer")
    start = load_address + header_size
    if not reset & 1 or not start <= (reset & ~1) < start + body_size:
        raise ValueError("reset vector does not point into the linked application")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("config", type=Path)
    args = parser.parse_args()
    config = dict(
        line.split("=", 1)
        for line in args.config.read_text().splitlines()
        if line.startswith("CONFIG_") and "=" in line
    )
    if config.get("CONFIG_MCUBOOT_BOOTLOADER_MODE_SWAP_SCRATCH") != "y":
        raise ValueError("installed bootloader requires swap using scratch")
    check_image(
        args.image.read_bytes(),
        int(config["CONFIG_ROM_START_OFFSET"], 0),
        int(config["CONFIG_FLASH_BASE_ADDRESS"], 0)
        + int(config["CONFIG_FLASH_LOAD_OFFSET"], 0),
        int(config["CONFIG_FLASH_LOAD_SIZE"], 0),
    )
    print("MCUboot image layout and vector table verified")


if __name__ == "__main__":
    main()
