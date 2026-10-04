# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martino Pilia

"""Bench helpers shared by the wire tools: snapshots of the Pico's device counters
and of both hosts' kernel and driver counters around a traffic run, and the peer
link mode a tool sets and has to put back."""

import json
import os
import subprocess
import sys
import time

# Labels as emitted by read_ethernet_stats.py (leading whitespace stripped by the
# parser). Covers the full TX/RX outcome set, so a failing run localizes the TX drops
# (xmit_error) and the RX per-error sub-counters.
DEVICE_COUNTERS = (
    # TX
    "xmit_ok",
    "xmit_error",
    "xmit_underrun",
    # Half-duplex transmit outcomes
    "xmit_deferred",
    "xmit_one_collision",
    "xmit_more_collisions",
    "xmit_max_collisions",
    "xmit_late_collisions",
    "link_down_dropped",
    "link_transitions",
    # RX aggregate + per-error sub-counters (Diagnostic in src/phy/phy_stats.h)
    "rcv_ok",
    "rcv_error",
    "runt",
    "giant",
    "bad_fcs",
    "carrier_glitch",
    "truncated",
    "pool_overflow",
    "host_backpressure",
)

# Kernel netdev ("<rx|tx>_<name>" from `ip -s -s -j link`) and root-qdisc counters.
# Frames the host drops before they reach the Pico show up in the qdisc (usbnet
# stops the queue under USB backpressure) or in tx_dropped.
PICO_LINK_COUNTERS = (
    "tx_packets",
    "tx_errors",
    "tx_dropped",
    "tx_fifo_errors",
    "rx_packets",
    "rx_errors",
    "rx_dropped",
    "qdisc_packets",
    "qdisc_drops",
    "qdisc_requeues",
    "qdisc_overlimits",
)
# The peer driver may fold frames its NIC flagged bad (CRC, alignment) into
# rx_errors without the per-kind breakdown.
PEER_LINK_COUNTERS = (
    "rx_packets",
    "rx_errors",
    "rx_dropped",
    "rx_missed_errors",
    "rx_over_errors",
    "rx_length_errors",
    "rx_crc_errors",
    "rx_frame_errors",
    "rx_fifo_errors",
    "tx_packets",
    "tx_errors",
    "tx_collisions",
)


def sudo(cmd, **kwargs):
    return subprocess.run(["sudo", *cmd], **kwargs)


def wait_for_iface(iface, timeout=15.0):
    end = time.time() + timeout
    while time.time() < end:
        if os.path.exists(f"/sys/class/net/{iface}"):
            return True
        time.sleep(0.3)
    return False


def wait_for_carrier(iface, timeout=10.0):
    """Whether the host sees carrier on `iface`, in the default namespace, within
    `timeout`. The Pico's link can come up seconds after the peer's (parallel
    detection), and until then the host drops what is sent through the Pico."""
    end = time.time() + timeout
    while time.time() < end:
        try:
            with open(f"/sys/class/net/{iface}/carrier") as f:
                if f.read().strip() == "1":
                    return True
        except OSError:
            pass  # EINVAL while the interface is administratively down
        time.sleep(0.1)
    return False


# `ethtool -s advertise` bits of the link modes a 10/100/1000BASE-T adapter lists.
ADVERTISE_BITS = {
    "10baseT/Half": 0x001,
    "10baseT/Full": 0x002,
    "100baseT/Half": 0x004,
    "100baseT/Full": 0x008,
    "1000baseT/Half": 0x010,
    "1000baseT/Full": 0x020,
}

# How --peer-mode sets the peer's link: autonegotiation advertising 10BASE-T half and
# full duplex only, or 10BASE-T half duplex forced with autonegotiation off.
PEER_MODES = {
    "autoneg": ["autoneg", "on", "advertise",
                hex(ADVERTISE_BITS["10baseT/Half"] | ADVERTISE_BITS["10baseT/Full"])],
    "half": ["speed", "10", "duplex", "half", "autoneg", "off"],
}


def read_link(iface, netns=None):
    """What `ethtool <iface>` reports of the link: speed, duplex, autoneg, the
    advertised modes as an `advertise` mask, and whether the link is detected."""
    out = sudo([*netns_prefix(netns), "ethtool", iface], capture_output=True, text=True)
    if out.returncode != 0:
        return None
    info = {}
    advertised = []
    in_advertised = False
    for line in out.stdout.splitlines():
        name, sep, value = line.strip().partition(":")
        if not sep:
            # The advertised link modes list continues over several lines.
            if in_advertised:
                advertised += name.split()
            continue
        in_advertised = name == "Advertised link modes"
        value = value.strip()
        if in_advertised:
            advertised += value.split()
        elif name == "Speed":
            digits = "".join(c for c in value if c.isdigit())
            if digits:
                info["speed"] = digits
        elif name == "Duplex":
            info["duplex"] = value.lower()
        elif name == "Auto-negotiation":
            info["autoneg"] = value.lower()
        elif name == "Link detected":
            info["detected"] = value == "yes"
    mask = sum(ADVERTISE_BITS.get(mode, 0) for mode in advertised)
    if mask:
        info["advertise"] = hex(mask)
    return info


def capture_link(iface):
    info = read_link(iface)
    return info if info and "autoneg" in info else None


def restore_link(iface, link):
    """Put back the link mode capture_link() recorded."""
    if link and link.get("autoneg") == "off" and "speed" in link and "duplex" in link:
        sudo(["ethtool", "-s", iface, "speed", link["speed"],
              "duplex", link["duplex"], "autoneg", "off"], capture_output=True)
    elif link and "advertise" in link:
        sudo(["ethtool", "-s", iface, "autoneg", "on", "advertise", link["advertise"]],
             capture_output=True)
    else:
        # Unreadable: at least undo a forced mode.
        sudo(["ethtool", "-s", iface, "autoneg", "on"], capture_output=True)


def set_peer_mode(iface, mode, netns=None):
    return sudo([*netns_prefix(netns), "ethtool", "-s", iface, *PEER_MODES[mode]],
                capture_output=True, text=True)


def wait_for_link_duplex(iface, netns=None, timeout=10.0):
    """The duplex ("half" or "full") the link resolves to once it is up, or None if
    it does not come up in time. Negotiation, or parallel detection on the Pico's
    side, takes a few seconds."""
    end = time.time() + timeout
    while time.time() < end:
        info = read_link(iface, netns)
        if info and info.get("detected") and info.get("duplex") in ("half", "full"):
            return info["duplex"]
        time.sleep(0.3)
    return None


def read_device_counters():
    reader = os.path.join(os.path.dirname(os.path.abspath(__file__)), "read_ethernet_stats.py")
    proc = sudo([reader], capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"warning: device-counter read failed: {proc.stderr.strip()}",
              file=sys.stderr)
        return None
    values = {}
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[-1].isdigit():
            label = parts[0]
            if label in DEVICE_COUNTERS:
                values[label] = int(parts[-1])
    return values


def netns_prefix(netns):
    return ["ip", "netns", "exec", netns] if netns else []


def read_link_counters(iface, netns):
    prefix = netns_prefix(netns)
    link = sudo([*prefix, "ip", "-s", "-s", "-j", "link", "show", "dev", iface],
                capture_output=True, text=True)
    if link.returncode != 0:
        return None
    try:
        counters = {f"{direction}_{name}": value
                    for direction, stats in json.loads(link.stdout)[0].get("stats64", {}).items()
                    for name, value in stats.items()}
        qdisc = sudo([*prefix, "tc", "-s", "-j", "qdisc", "show", "dev", iface],
                     capture_output=True, text=True)
        if qdisc.returncode == 0:
            # The root qdisc already aggregates its children (e.g. mq).
            for q in json.loads(qdisc.stdout):
                if q.get("root"):
                    for name in ("packets", "drops", "requeues", "overlimits"):
                        counters[f"qdisc_{name}"] = q.get(name, 0)
    except (json.JSONDecodeError, IndexError):
        return None
    return counters


def read_ethtool_stats(iface, netns):
    """Driver-specific counters; None when the driver exposes none."""
    out = sudo([*netns_prefix(netns), "ethtool", "-S", iface], capture_output=True, text=True)
    if out.returncode != 0:
        return None
    counters = {}
    for line in out.stdout.splitlines():
        name, sep, value = line.strip().rpartition(":")
        value = value.strip()
        if sep and value.lstrip("-").isdigit():
            counters[name.strip()] = int(value)
    return counters or None


class CounterProbe:
    """A counter source snapshotted around the traffic run. `keys` fixes the printed
    counters; None prints every counter that changed (for driver-specific names)."""

    def __init__(self, title, read, keys=None):
        self.title = title
        self.read = read
        self.keys = keys
        self.baseline = None
        self.final = None

    def snapshot_baseline(self):
        self.baseline = self.read()

    def snapshot_final(self):
        self.final = self.read()

    def print_delta(self):
        print_counter_delta(self.title, self.baseline, self.final, self.keys)


def print_counter_delta(title, baseline, final, keys=None):
    print(f"\n=== {title} (delta over the run) ===")
    if not baseline or not final:
        print("  unavailable")
        return
    if keys is None:
        keys = [k for k in final if k in baseline and final[k] != baseline[k]]
        if not keys:
            print("  no counter changed")
    width = max([22, *map(len, keys)])
    for key in keys:
        if key in baseline and key in final:
            print(f"  {key:<{width}} {final[key] - baseline[key]}")
