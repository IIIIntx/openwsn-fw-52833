"""Receive four-RX IQ reports from multi_collector over UART.

UART frame (big-endian lengths):
    55 AA | version | rx_id | tx1_seq | tx2_seq | flags |
    tx1_iq_count | tx2_iq_count | data_len:u16 | data | xor_checksum

Complete data is 704 bytes: TX1 IQ first (352), then TX2 IQ (352).
Every accepted frame is appended to a JSONL file. IQ bytes are stored as
base64 so the original uint32 sample representation is preserved exactly.
"""

from __future__ import annotations

import argparse
import base64
import json
from datetime import datetime
from pathlib import Path
from time import monotonic

import serial


# Edit this define to match the nRF52840DK virtual COM port. The --port option
# can still override it for a particular run.
COM_PORT = "COM9"
BAUD_RATE = 115200

SYNC = b"\x55\xaa"
STARTUP_TEST = b"test\r\n"
VERSION = 1
HEADER_LEN = 11
IQ_BYTES_PER_TX = 352
IQ_BYTES_TOTAL = 704
FLAG_TX1 = 0x01
FLAG_TX2 = 0x02
FLAG_COMPLETE = 0x04
KNOWN_FLAGS = FLAG_TX1 | FLAG_TX2 | FLAG_COMPLETE
RX_NODE_COUNT = 4
ROUND_STATUS_TIMEOUT_S = 3.0


def extract_frames(buffer: bytearray):
    """Yield checksum-valid frames while retaining an incomplete tail."""
    while True:
        start = buffer.find(SYNC)
        if start < 0:
            if buffer[-1:] == SYNC[:1]:
                del buffer[:-1]
            else:
                buffer.clear()
            return
        if start:
            del buffer[:start]
        if len(buffer) < HEADER_LEN:
            return

        data_len = (buffer[9] << 8) | buffer[10]
        flags = buffer[6]
        header_valid = (
            buffer[2] == VERSION
            and 1 <= buffer[3] <= RX_NODE_COUNT
            and not (flags & ~KNOWN_FLAGS)
            and data_len in (0, IQ_BYTES_TOTAL)
            and bool(flags & FLAG_COMPLETE) == (data_len == IQ_BYTES_TOTAL)
        )
        if not header_valid:
            del buffer[0]
            continue

        frame_len = HEADER_LEN + data_len + 1
        if len(buffer) < frame_len:
            return

        frame = bytes(buffer[:frame_len])
        del buffer[:frame_len]
        checksum = 0
        for byte in frame[2:-1]:
            checksum ^= byte
        if checksum == frame[-1]:
            yield frame


def decode_frame(frame: bytes) -> dict:
    data_len = (frame[9] << 8) | frame[10]
    data = frame[HEADER_LEN : HEADER_LEN + data_len]
    flags = frame[6]
    return {
        "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "rx_id": frame[3],
        "tx1_seq": frame[4],
        "tx2_seq": frame[5],
        "flags": flags,
        "tx1_received": bool(flags & FLAG_TX1),
        "tx2_received": bool(flags & FLAG_TX2),
        "iq_complete": bool(flags & FLAG_COMPLETE) and data_len == IQ_BYTES_TOTAL,
        "tx1_iq_count": frame[7],
        "tx2_iq_count": frame[8],
        "tx1_iq_raw_b64": base64.b64encode(data[:IQ_BYTES_PER_TX]).decode()
        if data_len
        else "",
        "tx2_iq_raw_b64": base64.b64encode(data[IQ_BYTES_PER_TX:]).decode()
        if data_len
        else "",
    }


def print_status(record: dict) -> None:
    tx1 = "OK" if record["tx1_received"] else "MISSING"
    tx2 = "OK" if record["tx2_received"] else "MISSING"
    iq = "IQ COMPLETE" if record["iq_complete"] else "NO IQ DATA"
    print(
        f'RX{record["rx_id"]} '
        f'TX1(seq={record["tx1_seq"]:03d}, iq={record["tx1_iq_count"]:03d}) {tx1} | '
        f'TX2(seq={record["tx2_seq"]:03d}, iq={record["tx2_iq_count"]:03d}) {tx2} | '
        f"{iq}",
        flush=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Receive multi_collector IQ frames")
    parser.add_argument("--port", default=COM_PORT, help=f"Collector serial port (default: {COM_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("collector_iq_capture.jsonl"),
        help="Append-only JSONL output file",
    )
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    serial_buffer = bytearray()
    startup_probe = bytearray()
    startup_seen = False
    round_status: dict[int, tuple[float, dict[int, dict]]] = {}

    print(f"Listening on {args.port} at {args.baud} baud")
    print(f"Saving to {args.output.resolve()}")

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser, args.output.open(
        "a", encoding="utf-8"
    ) as output:
        try:
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                serial_buffer.extend(chunk)

                if not startup_seen:
                    startup_probe.extend(chunk)
                    if STARTUP_TEST in startup_probe:
                        startup_seen = True
                        print("UART startup test received: test", flush=True)
                    elif len(startup_probe) > 64:
                        del startup_probe[: -(len(STARTUP_TEST) - 1)]

                for frame in extract_frames(serial_buffer):
                    record = decode_frame(frame)
                    output.write(json.dumps(record, separators=(",", ":")) + "\n")
                    output.flush()
                    print_status(record)

                    round_seq = (
                        record["tx1_seq"]
                        if record["tx1_received"]
                        else record["tx2_seq"]
                    )
                    now = monotonic()
                    for stale_seq, (created_at, _) in list(round_status.items()):
                        if now - created_at > ROUND_STATUS_TIMEOUT_S:
                            del round_status[stale_seq]

                    _, nodes = round_status.setdefault(round_seq, (now, {}))
                    nodes[record["rx_id"]] = record
                    if len(nodes) == RX_NODE_COUNT:
                        missing = []
                        for rx_id in range(1, 5):
                            node = nodes[rx_id]
                            if not node["tx1_received"]:
                                missing.append(f"RX{rx_id}:TX1")
                            if not node["tx2_received"]:
                                missing.append(f"RX{rx_id}:TX2")
                        print(
                            f"ROUND {round_seq:03d}: "
                            + (", ".join(missing) if missing else "all RX/TX packets present"),
                            flush=True,
                        )
                        del round_status[round_seq]
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
