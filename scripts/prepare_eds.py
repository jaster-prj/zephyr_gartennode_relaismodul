#!/usr/bin/env python3
"""Generate release EDS metadata using the signed firmware's version."""

import argparse
import ast
import configparser
from pathlib import Path


def prepare_eds(config, source, destination):
    prefix = "CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="
    lines = [line for line in config.read_text().splitlines()
             if line.startswith(prefix)]
    if len(lines) != 1:
        raise ValueError("signed firmware version missing from configuration")
    version = ast.literal_eval(lines[0][len(prefix):])
    if not isinstance(version, str) or not version:
        raise ValueError("signed firmware version is empty")
    eds = configparser.ConfigParser(interpolation=None)
    eds.optionxform = str
    eds.read_string(source.read_text())
    section = next((name for name in eds.sections() if name.upper() == "100A"), None)
    if section is None:
        raise ValueError("EDS manufacturer software version object is missing")
    eds.set(section, "DefaultValue", version)
    with destination.open("w") as output:
        eds.write(output, space_around_delimiters=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    prepare_eds(args.config, args.source, args.destination)


if __name__ == "__main__":
    main()
