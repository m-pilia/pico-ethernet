#!/usr/bin/env -S uv run
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///

"""Flood the Pico and the peer with broadcast frames at the same time, and report
how both sides fared.

The peer either autonegotiates, advertising 10BASE-T half and full duplex, or is
forced to 10BASE-T half duplex (--peer-mode). In half duplex both stations contend
for the medium; in full duplex the floods run independently. Runs until Ctrl-C, or
for --duration seconds. Either way the Pico's device
counters and both hosts' kernel and driver counters are printed as deltas over the
run, and the peer's link mode is put back to what it was.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

from wire_counters import (
    DEVICE_COUNTERS,
    PEER_LINK_COUNTERS,
    PEER_MODES,
    PICO_LINK_COUNTERS,
    CounterProbe,
    capture_link,
    print_counter_delta,
    read_device_counters,
    read_ethtool_stats,
    read_link_counters,
    restore_link,
    set_peer_mode,
    sudo,
    wait_for_carrier,
    wait_for_iface,
    wait_for_link_duplex,
)


def start_flood(iface, delay, mac):
    # In a process group of its own, so the terminal's Ctrl-C reaches only this
    # script, which then stops the floods before reading the counters.
    return subprocess.Popen(
        ["sudo", "mausezahn", iface, "-c", "0", "-d", delay, "-b", mac, "-t", "ip"],
        preexec_fn=os.setpgrp,
    )


def wait_floods(procs, duration):
    """Returns once `duration` seconds have passed (never when None), a flood has
    exited, or Ctrl-C is pressed."""
    deadline = None if duration is None else time.monotonic() + duration
    try:
        while all(p.poll() is None for p in procs):
            if deadline is not None and time.monotonic() >= deadline:
                return
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def stop_floods(procs):
    for p in procs:
        if p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    for p in procs:
        p.wait()


def flood_and_report(args):
    # The device read detaches cdc_ncm, which recreates the Pico netdev and zeroes
    # its kernel counters: the host baselines have to follow it, and the host finals
    # precede it.
    device_baseline = read_device_counters()
    if not wait_for_iface(args.target_interface):
        print(f"error: {args.target_interface} did not reappear after counter read",
              file=sys.stderr)
        return 1
    if not wait_for_carrier(args.target_interface):
        print(f"error: {args.target_interface} has no carrier", file=sys.stderr)
        return 1

    pico = args.target_interface
    peer = args.peer_interface
    probes = [
        CounterProbe(f"Pico host interface {pico}",
                     lambda: read_link_counters(pico, netns=None), PICO_LINK_COUNTERS),
        CounterProbe(f"Pico host driver statistics (ethtool -S {pico})",
                     lambda: read_ethtool_stats(pico, netns=None)),
        CounterProbe(f"peer interface {peer}",
                     lambda: read_link_counters(peer, netns=None), PEER_LINK_COUNTERS),
        CounterProbe(f"peer driver statistics (ethtool -S {peer})",
                     lambda: read_ethtool_stats(peer, netns=None)),
    ]
    for probe in probes:
        probe.snapshot_baseline()

    procs = []
    try:
        procs.append(start_flood(pico, args.target_delay, args.mac))
        procs.append(start_flood(peer, args.peer_delay, args.mac))
        wait_floods(procs, args.duration)
    finally:
        # A second Ctrl-C must not cut the teardown short.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        stop_floods(procs)

    for probe in probes:
        probe.snapshot_final()
    if not wait_for_iface(pico):
        print(f"warning: {pico} not back before final read", file=sys.stderr)
    print_counter_delta("Pico device counters", device_baseline, read_device_counters(),
                        DEVICE_COUNTERS)
    for probe in probes:
        probe.print_delta()
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-interface", required=True)
    parser.add_argument("--peer-interface", required=True)
    parser.add_argument("--mac", default="ffff.ffff.ffff")
    parser.add_argument("--target-delay", default="200us")
    parser.add_argument("--peer-delay", default="200us")
    parser.add_argument("--peer-mode", choices=PEER_MODES, default="autoneg",
                        help="autoneg: the peer advertises 10BASE-T half and full duplex; "
                             "half: the peer is forced to 10BASE-T half duplex "
                             "(default %(default)s)")
    parser.add_argument("--duration", type=float, default=None,
                        help="seconds to flood for (default: until Ctrl-C)")
    args = parser.parse_args()

    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    for tool in ("ethtool", "mausezahn"):
        if shutil.which(tool) is None:
            print(f"error: {tool} not found in PATH", file=sys.stderr)
            return 1

    sudo(["-v"], check=True)

    saved_link = capture_link(args.peer_interface)
    set_peer_mode(args.peer_interface, args.peer_mode).check_returncode()
    try:
        duplex = wait_for_link_duplex(args.peer_interface)
        if duplex is None:
            print(f"error: {args.peer_interface} has no link", file=sys.stderr)
            return 1
        print(f"link 10/{duplex.capitalize()}")
        return flood_and_report(args)
    finally:
        restore_link(args.peer_interface, saved_link)


if __name__ == "__main__":
    sys.exit(main())
