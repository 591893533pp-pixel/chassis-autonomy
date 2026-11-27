#!/usr/bin/env python3
"""Check the built Linux SDK ABI without opening sensors or network sockets."""
import argparse
import pathlib
import re
import subprocess


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('library', nargs='?', type=pathlib.Path,
                        default=root / 'sensors/sdk-install/lib/liblivox_lidar_sdk_shared.so')
    args = parser.parse_args()
    header = (root / 'sensors/Livox-SDK2/include/livox_lidar_api.h').read_text()
    expected = set(re.findall(r'^(?:void|bool|uint16_t|livox_status)\s+(\w+)\s*\(',
                              header, re.MULTILINE))
    result = subprocess.check_output(
        ['nm', '-D', '--defined-only', '--format=posix', str(args.library)], text=True)
    actual = {line.split()[0] for line in result.splitlines() if line.strip()}
    missing, leaked = expected - actual, actual - expected
    if missing or leaked:
        raise SystemExit(f'SDK export failure: missing={sorted(missing)}, leaked={sorted(leaked)}')
    print(f'PASS: {len(expected)} public C functions; no private C++ exports')


if __name__ == '__main__':
    main()
