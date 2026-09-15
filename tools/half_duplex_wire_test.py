#!/usr/bin/env -S uv run --script
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martino Pilia

# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///

"""Line-legal wire test for the software-PHY 10BASE-T NIC.

The Pico NIC (device under test) and a USB 10/100 adapter (peer) are both on this
PC, joined by the copper link through the Pico's PHY. This tool drives crafted
Ethernet frames between them to check two things under a controllable, line-legal
load: that frames are not lost, and that their content is intact.

Because 10BASE-T half-duplex is a shared medium, --load is the *aggregate* offered
load on the one 10 Mbit/s budget; with --direction both it is split across the two
directions (--tx-share). The receiver reports kernel capture drops (PACKET_STATISTICS)
separately from wire loss, so a userspace-generator shortfall is visible rather than
mistaken for the PHY dropping frames.
"""

import argparse
import json
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import zlib

ETHERTYPE = 0x88B5
MAGIC = b"PWT1"
PICO_MAC_DEFAULT = "02:00:00:00:00:01"
NETNS = "picowire"
LINE_BPS = 10_000_000

STREAM_PEER_TO_PICO = 1
STREAM_PICO_TO_PEER = 2

HEADER_LEN = len(MAGIC) + 1 + 4 + 2  # magic + stream + seq + declared length
MIN_PAYLOAD = 46  # 14-byte L2 header + 46 => 60-byte frame => 64 on the wire with FCS
MAX_PAYLOAD = 1500
CRC_LEN = 4

SOL_PACKET = 263
PACKET_ADD_MEMBERSHIP = 1
PACKET_MR_PROMISC = 1
PACKET_STATISTICS = 6
RCVBUF_BYTES = 16 * 1024 * 1024

# Simple IMIX: seven small, four medium, one large, by payload length.
IMIX_PAYLOADS = [MIN_PAYLOAD] * 7 + [576] * 4 + [MAX_PAYLOAD]

# Labels as emitted by read_ethernet_stats.py (leading whitespace stripped by the
# parser). Covers the full TX/RX outcome set, so a failing run localizes the TX drops
# (xmit_error) and the RX per-error sub-counters.
DEVICE_COUNTERS = (
    # TX
    "xmit_ok",
    "xmit_error",
    "xmit_underrun",
    # RX aggregate + per-error sub-counters (RxDiagnostic in src/phy/phy_stats.h)
    "rcv_ok",
    "rcv_error",
    "bad_preamble",
    "runt",
    "giant",
    "bad_fcs",
    "carrier_glitch",
    "decode_error",
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


def parse_mac(text):
    parts = text.replace("-", ":").split(":")
    if len(parts) != 6:
        raise ValueError(f"invalid MAC: {text!r}")
    return bytes(int(p, 16) for p in parts)


def wire_bytes(payload_len):
    """Full on-wire footprint of a frame: L2 (padded to 60) + FCS + preamble/SFD + IFG."""
    l2 = max(14 + payload_len, 60)
    return l2 + 4 + 8 + 12


def build_payload(stream, seq, size):
    size = max(size, MIN_PAYLOAD)
    body = bytearray(size)
    struct.pack_into(">4sBIH", body, 0, MAGIC, stream, seq & 0xFFFFFFFF, size)
    for i in range(HEADER_LEN, size - CRC_LEN):
        body[i] = (seq + i) & 0xFF
    crc = zlib.crc32(bytes(body[: size - CRC_LEN])) & 0xFFFFFFFF
    struct.pack_into(">I", body, size - CRC_LEN, crc)
    return bytes(body)


def check_payload(payload):
    """Return (stream, seq) if this is one of our frames and its content is intact,
    ('corrupt', seq) if it is ours but damaged, or None if it is foreign traffic."""
    if len(payload) < HEADER_LEN + CRC_LEN or payload[: len(MAGIC)] != MAGIC:
        return None
    stream, seq, size = struct.unpack_from(">BIH", payload, len(MAGIC))
    if size != len(payload) or size < MIN_PAYLOAD:
        return ("corrupt", seq)
    want = zlib.crc32(payload[: size - CRC_LEN]) & 0xFFFFFFFF
    (got,) = struct.unpack_from(">I", payload, size - CRC_LEN)
    if want != got:
        return ("corrupt", seq)
    return (stream, seq)


def emit(event):
    sys.stdout.write(json.dumps(event) + "\n")
    sys.stdout.flush()


def open_socket(iface):
    sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETHERTYPE))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF_BYTES)
    sock.bind((iface, ETHERTYPE))
    ifindex = socket.if_nametoindex(iface)
    mreq = struct.pack("iHH8s", ifindex, PACKET_MR_PROMISC, 0, b"")
    sock.setsockopt(SOL_PACKET, PACKET_ADD_MEMBERSHIP, mreq)
    return sock


class Pacer:
    def __init__(self, load_pct):
        self.rate = LINE_BPS * load_pct / 100.0
        self.next = None

    def wait(self, bits):
        interval = bits / self.rate
        now = time.perf_counter()
        if self.next is None:
            self.next = now
        target = self.next
        remaining = target - now
        if remaining > 0.002:
            time.sleep(remaining - 0.001)
        while time.perf_counter() < target:
            pass
        self.next += interval
        now = time.perf_counter()
        if self.next < now - 0.05:  # fell far behind; resync rather than burst
            self.next = now


def size_picker(spec):
    if spec == "min":
        return lambda: MIN_PAYLOAD
    if spec == "max":
        return lambda: MAX_PAYLOAD
    if spec == "imix":
        import itertools

        cycle = itertools.cycle(IMIX_PAYLOADS)
        return lambda: next(cycle)
    if spec == "random":
        import random

        return lambda: random.randint(MIN_PAYLOAD, MAX_PAYLOAD)
    value = int(spec)
    if not MIN_PAYLOAD <= value <= MAX_PAYLOAD:
        raise ValueError(f"--size out of range [{MIN_PAYLOAD},{MAX_PAYLOAD}]: {value}")
    return lambda: value


def sender_loop(sock, stop, deadline, stream, src, dst, size_spec, load_pct, start_delay):
    header = dst + src + struct.pack(">H", ETHERTYPE)
    pick = size_picker(size_spec)
    pacer = Pacer(load_pct)
    time.sleep(start_delay)
    seq = 0
    wire_bits = 0
    start = time.perf_counter()
    while not stop.is_set() and time.time() < deadline:
        size = pick()
        sock.send(header + build_payload(stream, seq, size))
        seq += 1
        bits = wire_bytes(size) * 8
        wire_bits += bits
        pacer.wait(bits)
    emit({"type": "summary", "role": "sender", "stream": stream, "sent": seq,
          "wire_bits": wire_bits, "elapsed": time.perf_counter() - start})


def receiver_loop(sock, stop, deadline, stream, report_failures, grace):
    # Keep draining briefly after the sender stops so frames still in flight at the
    # deadline are not miscounted as lost.
    rx_deadline = deadline if deadline == float("inf") else deadline + grace
    sock.settimeout(0.2)
    received = corrupt = late = 0
    expected = 0
    first = last = None
    while not stop.is_set() and time.time() < rx_deadline:
        try:
            frame = sock.recv(2048)
        except socket.timeout:
            continue
        result = check_payload(frame[14:])
        if result is None:
            continue
        kind, seq = result
        if kind == "corrupt":
            corrupt += 1
            if report_failures:
                emit({"type": "failure", "kind": "corrupt", "stream": stream, "seq": seq})
            continue
        if kind != stream:
            continue
        received += 1
        first = seq if first is None else first
        last = seq
        if seq >= expected:
            if seq > expected and report_failures:
                emit({"type": "failure", "kind": "loss", "stream": stream,
                      "from": expected, "to": seq - 1, "count": seq - expected})
            expected = seq + 1
        else:
            late += 1
    _tp_packets, tp_drops = struct.unpack("II", sock.getsockopt(SOL_PACKET, PACKET_STATISTICS, 8))
    emit({"type": "summary", "role": "receiver", "stream": stream,
          "received": received, "corrupt": corrupt, "late": late,
          "first": first, "last": last, "host_drops": tp_drops})


def worker_main(argv):
    p = argparse.ArgumentParser(prog="wire test worker")
    p.add_argument("--iface", required=True)
    p.add_argument("--send-stream", type=int, default=0)
    p.add_argument("--send-src")
    p.add_argument("--send-dst")
    p.add_argument("--recv-stream", type=int, default=0)
    p.add_argument("--size", default="min")
    p.add_argument("--load", type=float, default=90.0)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--live", action="store_true")
    p.add_argument("--start-delay", type=float, default=0.5)
    p.add_argument("--drain-grace", type=float, default=1.0)
    p.add_argument("--report-failures", action="store_true")
    args = p.parse_args(argv)

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    deadline = float("inf") if args.live else time.time() + args.duration + args.start_delay

    threads = []
    socks = []
    if args.recv_stream:
        sock = open_socket(args.iface)
        socks.append(sock)
        threads.append(threading.Thread(
            target=receiver_loop,
            args=(sock, stop, deadline, args.recv_stream,
                  args.report_failures or args.live, args.drain_grace),
        ))
    if args.send_stream:
        sock = open_socket(args.iface)
        socks.append(sock)
        threads.append(threading.Thread(
            target=sender_loop,
            args=(sock, stop, deadline, args.send_stream, parse_mac(args.send_src),
                  parse_mac(args.send_dst), args.size, args.load, args.start_delay),
        ))
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    for sock in socks:
        sock.close()


def sudo(cmd, **kwargs):
    return subprocess.run(["sudo", *cmd], **kwargs)


def read_peer_mac(iface):
    out = sudo(
        ["ip", "netns", "exec", NETNS, "cat", f"/sys/class/net/{iface}/address"],
        check=True, capture_output=True, text=True,
    )
    return out.stdout.strip()


def wait_for_iface(iface, timeout=15.0):
    end = time.time() + timeout
    while time.time() < end:
        if os.path.exists(f"/sys/class/net/{iface}"):
            return True
        time.sleep(0.3)
    return False


OFFLOAD_FEATURES = {
    "gro": "generic-receive-offload",
    "gso": "generic-segmentation-offload",
    "tso": "tcp-segmentation-offload",
    "lro": "large-receive-offload",
}


def capture_link(iface):
    out = sudo(["ethtool", iface], capture_output=True, text=True)
    if out.returncode != 0:
        return None
    info = {}
    for line in out.stdout.splitlines():
        line = line.strip()
        if line.startswith("Speed:"):
            digits = "".join(c for c in line.split(":", 1)[1] if c.isdigit())
            if digits:
                info["speed"] = digits
        elif line.startswith("Duplex:"):
            info["duplex"] = line.split(":", 1)[1].strip().lower()
        elif line.startswith("Auto-negotiation:"):
            info["autoneg"] = line.split(":", 1)[1].strip().lower()
    return info if "autoneg" in info else None


def capture_offloads(iface):
    out = sudo(["ethtool", "-k", iface], capture_output=True, text=True)
    if out.returncode != 0:
        return {}
    long_to_short = {v: k for k, v in OFFLOAD_FEATURES.items()}
    state = {}
    for line in out.stdout.splitlines():
        name, sep, rest = line.strip().partition(":")
        if not sep or name not in long_to_short:
            continue
        rest = rest.strip()
        if "fixed" in rest:  # not changeable, so setup could not have altered it
            continue
        value = rest.split()[0] if rest else ""
        if value in ("on", "off"):
            state[long_to_short[name]] = value
    return state


def capture_admin_up(iface):
    out = sudo(["ip", "-o", "link", "show", iface], capture_output=True, text=True)
    if out.returncode != 0:
        return True
    start, end = out.stdout.find("<"), out.stdout.find(">")
    flags = out.stdout[start + 1:end].split(",") if 0 <= start < end else []
    return "UP" in flags


def capture_state(peer, pico):
    """Snapshot the settings this tool will change, so teardown can restore them.
    The peer's link mode is managed (forced 10baseT/Half); the Pico's is not."""
    return {
        "peer": {"iface": peer, "manage_link": True, "link": capture_link(peer),
                 "offloads": capture_offloads(peer), "up": capture_admin_up(peer)},
        "pico": {"iface": pico, "manage_link": False, "link": None,
                 "offloads": capture_offloads(pico), "up": capture_admin_up(pico)},
    }


def restore_state(s):
    for short, value in s["offloads"].items():
        sudo(["ethtool", "-K", s["iface"], short, value], capture_output=True)
    if s["manage_link"]:
        link = s["link"]
        if link and link.get("autoneg") == "off" and "speed" in link and "duplex" in link:
            sudo(["ethtool", "-s", s["iface"], "speed", link["speed"],
                  "duplex", link["duplex"], "autoneg", "off"], capture_output=True)
        else:
            # Original was autoneg-on (or unreadable): undo the forced 10baseT/Half.
            sudo(["ethtool", "-s", s["iface"], "autoneg", "on"], capture_output=True)
    sudo(["ip", "link", "set", s["iface"], "up" if s["up"] else "down"], capture_output=True)


def setup_environment(peer, pico):
    sudo(["ip", "netns", "add", NETNS], check=True)
    sudo(["ip", "link", "set", peer, "netns", NETNS], check=True)
    sudo(["ip", "netns", "exec", NETNS, "ip", "link", "set", peer, "up"], check=True)

    forced = sudo(
        ["ip", "netns", "exec", NETNS, "ethtool", "-s", peer,
         "speed", "10", "duplex", "half", "autoneg", "off"],
        capture_output=True, text=True,
    )
    if forced.returncode != 0:
        print(f"warning: could not force {peer} to 10baseT/Half "
              f"({forced.stderr.strip()}); line legality is not hardware-enforced",
              file=sys.stderr)

    sudo(["ip", "link", "set", pico, "up"], check=True)
    for prefix in (["ip", "netns", "exec", NETNS], []):
        iface = peer if prefix else pico
        sudo([*prefix, "ethtool", "-K", iface,
              "gro", "off", "gso", "off", "tso", "off", "lro", "off"],
             capture_output=True)


def teardown_environment(saved):
    # Deleting the namespace returns the peer adapter to the default namespace
    # (brought down by the kernel); restore the settings we changed on both ifaces.
    sudo(["ip", "netns", "del", NETNS], capture_output=True)
    if not saved:
        return
    peer = saved["peer"]["iface"]
    if wait_for_iface(peer):
        restore_state(saved["peer"])
    else:
        print(f"warning: {peer} did not return to the default namespace; "
              "its settings were not restored", file=sys.stderr)
    restore_state(saved["pico"])


def read_device_counters(script_dir):
    reader = os.path.join(script_dir, "read_ethernet_stats.py")
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


def netns_prefix(in_netns):
    return ["ip", "netns", "exec", NETNS] if in_netns else []


def read_link_counters(iface, in_netns):
    prefix = netns_prefix(in_netns)
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


def read_ethtool_stats(iface, in_netns):
    """Driver-specific counters; None when the driver exposes none."""
    out = sudo([*netns_prefix(in_netns), "ethtool", "-S", iface], capture_output=True, text=True)
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


class WorkerHandle:
    def __init__(self, popen, on_event):
        self.popen = popen
        self.summaries = []
        self.reader = threading.Thread(target=self._pump, args=(on_event,), daemon=True)
        self.reader.start()

    def _pump(self, on_event):
        for raw in self.popen.stdout:
            raw = raw.strip()
            if not raw:
                continue
            try:
                event = json.loads(raw)
            except json.JSONDecodeError:
                print(raw, file=sys.stderr)
                continue
            if event.get("type") == "summary":
                self.summaries.append(event)
            on_event(event)


def spawn_worker(in_netns, worker_args, on_event):
    launcher = ["sudo"]
    if in_netns:
        launcher += ["ip", "netns", "exec", NETNS]
    cmd = [*launcher, sys.executable, os.path.abspath(__file__), "worker", *worker_args]
    popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1)
    return WorkerHandle(popen, on_event)


def plan_workers(direction, peer, pico, peer_mac, pico_mac):
    """Return (peer_spec, pico_spec) as dicts of worker roles for the direction."""
    peer_spec = {"iface": peer, "in_netns": True}
    pico_spec = {"iface": pico, "in_netns": False}
    if direction in ("rx", "both"):
        peer_spec["send"] = (STREAM_PEER_TO_PICO, peer_mac, pico_mac)
        pico_spec["recv"] = STREAM_PEER_TO_PICO
    if direction in ("tx", "both"):
        pico_spec["send"] = (STREAM_PICO_TO_PEER, pico_mac, peer_mac)
        peer_spec["recv"] = STREAM_PICO_TO_PEER
    return peer_spec, pico_spec


def worker_argv(spec, size, stream_loads, duration, live):
    argv = ["--iface", spec["iface"]]
    if "send" in spec:
        stream, src, dst = spec["send"]
        argv += ["--send-stream", str(stream), "--send-src", src, "--send-dst", dst,
                 "--size", size, "--load", str(stream_loads[stream])]
    if "recv" in spec:
        argv += ["--recv-stream", str(spec["recv"])]
    if live:
        argv.append("--live")
    else:
        argv += ["--duration", str(duration)]
    return argv


def direction_loads(direction, load, tx_share):
    """Split the aggregate offered load across the active directions. The half-duplex
    medium is shared, so bidirectional traffic draws from one budget."""
    loads = {}
    if direction in ("rx", "both"):
        loads[STREAM_PEER_TO_PICO] = load * (1 - tx_share) if direction == "both" else load
    if direction in ("tx", "both"):
        loads[STREAM_PICO_TO_PEER] = load * tx_share if direction == "both" else load
    return loads


def run_step(specs, size, stream_loads, duration, live):
    printed_live = {"count": 0}

    def on_event(event):
        if event.get("type") != "failure":
            return
        if event["kind"] == "loss":
            print(f"  ! LOSS  stream {event['stream']} seq {event['from']}..{event['to']} "
                  f"({event['count']} frame(s))")
        else:
            print(f"  ! CORRUPT stream {event['stream']} seq {event['seq']}")
        printed_live["count"] += 1

    handles = []
    for spec in specs:
        argv = worker_argv(spec, size, stream_loads, duration, live)
        handles.append(spawn_worker(spec["in_netns"], argv, on_event))

    try:
        for h in handles:
            h.popen.wait()
    except KeyboardInterrupt:
        for h in handles:
            h.popen.send_signal(signal.SIGINT)
        for h in handles:
            h.popen.wait()
    for h in handles:
        h.reader.join(timeout=2.0)

    summaries = [s for h in handles for s in h.summaries]
    return aggregate(summaries)


def aggregate(summaries):
    senders = {s["stream"]: s for s in summaries if s["role"] == "sender"}
    receivers = {s["stream"]: s for s in summaries if s["role"] == "receiver"}
    per_stream = {}
    for stream in sorted(set(senders) | set(receivers)):
        tx = senders.get(stream, {})
        sent = tx.get("sent", 0)
        elapsed = tx.get("elapsed", 0.0)
        offered_mbps = (tx.get("wire_bits", 0) / elapsed / 1e6) if elapsed else 0.0
        fps = (sent / elapsed) if elapsed else 0.0
        rx = receivers.get(stream, {})
        received = rx.get("received", 0)
        late = rx.get("late", 0)
        corrupt = rx.get("corrupt", 0)
        host_drops = rx.get("host_drops", 0)
        unique = received - late
        lost = max(sent - unique, 0)
        loss_pct = (100.0 * lost / sent) if sent else 0.0
        per_stream[stream] = {
            "sent": sent, "received": received, "unique": unique, "late": late,
            "corrupt": corrupt, "lost": lost, "loss_pct": loss_pct,
            "offered_mbps": offered_mbps, "fps": fps, "host_drops": host_drops,
        }
    return per_stream


STREAM_NAMES = {STREAM_PEER_TO_PICO: "peer->Pico", STREAM_PICO_TO_PEER: "Pico->peer"}


def print_step_result(per_stream):
    for stream, s in per_stream.items():
        note = "  [host-limited: results contaminated]" if s["host_drops"] else ""
        print(f"  {STREAM_NAMES[stream]:<11} sent={s['sent']:<8} recv={s['received']:<8} "
              f"lost={s['lost']:<7} ({s['loss_pct']:.3f}%)  corrupt={s['corrupt']}  "
              f"late={s['late']}  offered={s['offered_mbps']:.2f}Mb/s ({s['fps']:.0f}fps)  "
              f"hostdrop={s['host_drops']}{note}")


def step_passes(per_stream, loss_threshold):
    return all(s["loss_pct"] <= loss_threshold and s["corrupt"] == 0
               for s in per_stream.values())


def run_main(argv):
    p = argparse.ArgumentParser(
        prog=os.path.basename(__file__),
        description="Line-legal wire test for the Pico software-PHY 10BASE-T NIC.",
    )
    p.add_argument("--pico-interface", required=True,
                   help="host network interface for the Pico NIC (CDC-NCM)")
    p.add_argument("--peer-interface", required=True,
                   help="host network interface for the USB 10/100 peer adapter")
    p.add_argument("--pico-mac", default=PICO_MAC_DEFAULT)
    p.add_argument("--direction", choices=("rx", "tx", "both"), default="rx",
                   help="rx=peer->Pico, tx=Pico->peer, both=simultaneous")
    p.add_argument("--size", default="max",
                   help="min|max|imix|random|<bytes> (payload length; default %(default)s)")
    p.add_argument("--load", type=float, default=90.0,
                   help="aggregate offered load as percent of the shared 10 Mbit/s "
                   "half-duplex medium (split across directions when both; default: %(default).2f)")
    p.add_argument("--tx-share", type=float, default=0.5,
                   help="fraction of the aggregate budget given to Pico->peer when "
                        "--direction both (default %(default).2f)")
    p.add_argument("--duration", type=float, default=10.0,
                   help="seconds per run/step (ignored with --live)")
    p.add_argument("--sweep", nargs="?", const="10,25,50,75,90,100", default=None,
                   help="sweep offered load over a comma list (default 10,25,50,75,90,100)")
    p.add_argument("--loss-threshold", type=float, default=0.0,
                   help="max loss percent for a step to pass (default %(default).2f)")
    p.add_argument("--live", action="store_true",
                   help="run until interrupted, printing only on failure")
    p.add_argument("--device-stats", action="store_true",
                   help="snapshot the Pico's counters before/after (detaches cdc_ncm)")
    p.add_argument("--peer-stats", action="store_true",
                   help="snapshot the peer's kernel counters and `ethtool -S` driver "
                        "statistics before/after (not every driver exposes the latter)")
    args = p.parse_args(argv)

    if args.load <= 0 or args.load > 100:
        p.error("--load must be in (0, 100]")
    if not 0.0 <= args.tx_share <= 1.0:
        p.error("--tx-share must be in [0, 1]")
    if args.sweep and args.live:
        p.error("--sweep and --live are mutually exclusive")

    if shutil.which("ethtool") is None:
        print("error: ethtool not found in PATH", file=sys.stderr)
        return 1

    parse_mac(args.pico_mac)  # validate format before spawning workers
    script_dir = os.path.dirname(os.path.abspath(__file__))

    print("Caching sudo credentials (needed for namespace/socket setup)...")
    sudo(["-v"], check=True)

    baseline = None
    if args.device_stats:
        baseline = read_device_counters(script_dir)
        if not wait_for_iface(args.pico_interface):
            print(f"error: {args.pico_interface} did not reappear after counter read",
                  file=sys.stderr)
            return 1

    pico = args.pico_interface
    peer = args.peer_interface
    probes = [
        CounterProbe(f"Pico host interface {pico}",
                     lambda: read_link_counters(pico, in_netns=False), PICO_LINK_COUNTERS),
        # cdc_ncm reports NTBs sent (tx_ntbs; tx_packets / tx_ntbs = datagrams per
        # NTB) and why each NTB was closed (tx_reason_*).
        CounterProbe(f"Pico host driver statistics (ethtool -S {pico})",
                     lambda: read_ethtool_stats(pico, in_netns=False)),
    ]
    if args.peer_stats:
        probes += [
            CounterProbe(f"peer interface {peer}",
                         lambda: read_link_counters(peer, in_netns=True), PEER_LINK_COUNTERS),
            CounterProbe(f"peer driver statistics (ethtool -S {peer})",
                         lambda: read_ethtool_stats(peer, in_netns=True)),
        ]

    overall_ok = True
    saved = None
    try:
        # Clear any namespace left by an aborted run so the peer is back in the
        # default namespace before we snapshot the settings we are about to change.
        sudo(["ip", "netns", "del", NETNS], capture_output=True)
        wait_for_iface(args.peer_interface)
        saved = capture_state(args.peer_interface, args.pico_interface)
        setup_environment(args.peer_interface, args.pico_interface)
        if not wait_for_iface(args.pico_interface):
            print(f"error: {args.pico_interface} not present", file=sys.stderr)
            return 1
        for probe in probes:
            probe.snapshot_baseline()
        peer_mac = read_peer_mac(args.peer_interface)
        print(f"peer MAC {peer_mac}  ->  Pico MAC {args.pico_mac}   direction={args.direction}")

        peer_spec, pico_spec = plan_workers(
            args.direction, args.peer_interface, args.pico_interface,
            peer_mac, args.pico_mac,
        )
        specs = [peer_spec, pico_spec]

        loads = [float(x) for x in args.sweep.split(",")] if args.sweep else [args.load]
        for load in loads:
            stream_loads = direction_loads(args.direction, load, args.tx_share)
            label = "live" if args.live else f"{load:g}% aggregate, {args.duration:g}s"
            print(f"\n=== load: {label} ===")
            if args.live:
                print("(running until Ctrl-C; failures print below)")
            per_stream = run_step(specs, args.size, stream_loads, args.duration, args.live)
            print_step_result(per_stream)
            if not step_passes(per_stream, args.loss_threshold):
                overall_ok = False
    finally:
        # Before teardown (the peer leaves the namespace and drops link) and before
        # the final device read, whose cdc_ncm detach recreates the Pico netdev and
        # zeroes its counters.
        for probe in probes:
            probe.snapshot_final()
        teardown_environment(saved)

    if args.device_stats:
        if not wait_for_iface(args.pico_interface):
            print(f"warning: {args.pico_interface} not back before final read",
                  file=sys.stderr)
        final = read_device_counters(script_dir)
        print_counter_delta("Pico device counters", baseline, final, DEVICE_COUNTERS)
    for probe in probes:
        probe.print_delta()

    print(f"\nRESULT: {'PASS' if overall_ok else 'FAIL'}")
    return 0 if overall_ok else 1


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


def main():
    argv = sys.argv[1:]
    if argv and argv[0] == "worker":
        worker_main(argv[1:])
        return 0
    return run_main(argv)


if __name__ == "__main__":
    sys.exit(main())
