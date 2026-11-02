#!/usr/bin/env python3
"""
dump_live_opendrive.py — Dumps the OpenDRIVE data the running CARLA server is
actually using internally (world.get_map().to_opendrive()), and optionally
diffs it against a .xodr file on disk.

This is the way to check whether the live server's road network really
matches a given .xodr file, rather than assuming it does just because the
file on disk looks correct.

Usage:
    python dump_live_opendrive.py -o live_town15.xodr
    python dump_live_opendrive.py -o live_town15.xodr --diff-against Town15.xodr
"""

import argparse
import difflib

import carla


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1', help='CARLA server host (default 127.0.0.1)')
    ap.add_argument('--port', type=int, default=2000, help='CARLA server RPC port (default 2000)')
    ap.add_argument('--timeout', type=float, default=10.0, help='Client connection timeout in seconds (default 10)')
    ap.add_argument('-o', '--output', default='live_map.xodr', help='Where to save the dumped .xodr (default live_map.xodr)')
    ap.add_argument('--diff-against', default=None, help='Path to a .xodr file to diff the live one against')
    ap.add_argument('--diff-lines', type=int, default=200, help='Max diff lines to print (default 200)')
    args = ap.parse_args()

    print(f'Connecting to CARLA at {args.host}:{args.port} ...')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    cmap = world.get_map()
    print(f'Connected. Live map: {cmap.name}')

    live_opendrive = cmap.to_opendrive()
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write(live_opendrive)
    print(f'Wrote {len(live_opendrive)} bytes -> {args.output}')

    if args.diff_against:
        with open(args.diff_against, encoding='utf-8') as f:
            file_opendrive = f.read()

        if file_opendrive == live_opendrive:
            print(f'IDENTICAL: the live server\'s road network exactly matches {args.diff_against}')
            return

        print(f'DIFFERENT: the live server\'s road network does NOT match {args.diff_against}')
        print(f'Live: {len(live_opendrive)} bytes / {live_opendrive.count(chr(10))} lines')
        print(f'File: {len(file_opendrive)} bytes / {file_opendrive.count(chr(10))} lines')
        print()
        print(f'--- First {args.diff_lines} lines of diff (file -> live) ---')
        diff = difflib.unified_diff(
            file_opendrive.splitlines(keepends=True),
            live_opendrive.splitlines(keepends=True),
            fromfile=args.diff_against, tofile='<live server>',
        )
        for i, line in enumerate(diff):
            if i >= args.diff_lines:
                print('... (truncated, see the full diff by comparing the saved files yourself)')
                break
            print(line, end='')


if __name__ == '__main__':
    main()
