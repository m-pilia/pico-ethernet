#!/usr/bin/env -S uv run
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///

import argparse
import os
import shutil
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-interface", required=True)
    parser.add_argument("--peer-interface", required=True)
    parser.add_argument("--mac", default="ffff.ffff.ffff")
    parser.add_argument("--target-delay", default="200us")
    parser.add_argument("--peer-delay", default="200us")
    args = parser.parse_args()

    for tool in ("ethtool", "mausezahn"):
        if shutil.which(tool) is None:
            print(f"error: {tool} not found in PATH", file=sys.stderr)
            sys.exit(1)

    subprocess.run(["sudo", "-v"], check=True)

    subprocess.run(
        [
            "sudo", "ethtool", "-s", args.peer_interface,
            "speed", "10", "duplex", "half", "autoneg", "off",
        ],
        check=True,
    )

    procs = []
    try:
        procs.append(subprocess.Popen(
            ["sudo", "mausezahn", args.target_interface,
             "-c", "0", "-d", args.target_delay, "-b", args.mac, "-t", "ip"],
            preexec_fn=os.setpgrp,
        ))
        procs.append(subprocess.Popen(
            ["sudo", "mausezahn", args.peer_interface,
             "-c", "0", "-d", args.peer_delay, "-b", args.mac, "-t", "ip"],
            preexec_fn=os.setpgrp,
        ))
        for p in procs:
            p.wait()
    except KeyboardInterrupt:
        print()
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        for p in procs:
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for p in procs:
            p.wait()


if __name__ == "__main__":
    main()
