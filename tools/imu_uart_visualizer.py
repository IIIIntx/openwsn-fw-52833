#!/usr/bin/env python3
"""Receive BMI270 UART frames and plot acceleration in real time."""

from __future__ import annotations

import argparse
import collections
import struct
import time

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial
from serial.tools import list_ports


FRAME_LEN = 14
HEADER = b"\xaa\x55"


def list_serial_ports() -> None:
    for port in list_ports.comports():
        print(f"{port.device}\t{port.description}")


def parse_frame(frame: bytes) -> dict[str, int] | None:
    if len(frame) != FRAME_LEN or frame[:2] != HEADER:
        return None

    checksum = 0
    for value in frame[:-1]:
        checksum ^= value
    if checksum != frame[-1]:
        return None

    seq, who, diag, status, internal, ax, ay, az = struct.unpack("<BBBBBhhh", frame[2:13])
    return {
        "seq": seq,
        "who": who,
        "diag": diag,
        "status": status,
        "internal": internal,
        "ax": ax,
        "ay": ay,
        "az": az,
    }


def read_frame(ser: serial.Serial) -> dict[str, int] | None:
    while True:
        first = ser.read(1)
        if not first:
            return None
        if first != HEADER[:1]:
            continue
        second = ser.read(1)
        if second != HEADER[1:]:
            continue
        payload = ser.read(FRAME_LEN - 2)
        if len(payload) != FRAME_LEN - 2:
            return None
        parsed = parse_frame(HEADER + payload)
        if parsed is not None:
            return parsed


def run_visualizer(port: str, baud: int, window: int) -> None:
    xs = collections.deque(maxlen=window)
    ax_values = collections.deque(maxlen=window)
    ay_values = collections.deque(maxlen=window)
    az_values = collections.deque(maxlen=window)
    last_print = 0.0
    start = time.monotonic()

    ser = serial.Serial(port, baudrate=baud, timeout=0.05)

    fig, axis = plt.subplots()
    line_x, = axis.plot([], [], label="acc_x")
    line_y, = axis.plot([], [], label="acc_y")
    line_z, = axis.plot([], [], label="acc_z")
    axis.set_title("BMI270 acceleration")
    axis.set_xlabel("time (s)")
    axis.set_ylabel("raw int16")
    axis.grid(True)
    axis.legend(loc="upper right")

    def update(_frame: int):
        nonlocal last_print

        parsed = read_frame(ser)
        if parsed is None:
            return line_x, line_y, line_z

        now = time.monotonic() - start
        xs.append(now)
        ax_values.append(parsed["ax"])
        ay_values.append(parsed["ay"])
        az_values.append(parsed["az"])

        line_x.set_data(xs, ax_values)
        line_y.set_data(xs, ay_values)
        line_z.set_data(xs, az_values)

        if xs:
            axis.set_xlim(max(0, xs[-1] - window), max(window, xs[-1]))
        all_values = list(ax_values) + list(ay_values) + list(az_values)
        if all_values:
            low = min(all_values)
            high = max(all_values)
            margin = max(100, (high - low) // 10)
            axis.set_ylim(low - margin, high + margin)

        if now - last_print > 1.0:
            last_print = now
            print(
                "seq={seq:3d} who=0x{who:02x} diag=0x{diag:02x} "
                "status=0x{status:02x} internal=0x{internal:02x} "
                "acc=({ax:6d},{ay:6d},{az:6d})".format(**parsed)
            )

        return line_x, line_y, line_z

    animation.FuncAnimation(fig, update, interval=20, blit=False)
    plt.show()
    ser.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", help="Serial port, for example COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=int, default=120, help="Plot window in seconds")
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return
    if not args.port:
        parser.error("port is required unless --list is used")

    run_visualizer(args.port, args.baud, args.window)


if __name__ == "__main__":
    main()
