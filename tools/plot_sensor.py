#!/usr/bin/env python3
"""Live plot of SensorData (AccelerationData) streamed over UDP."""

import socket
import struct
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

UDP_PORT = 12289  # 0x3001
# AccelerationData: bool valid (1B) + 3B padding + float acc_x,y,z
SENSOR_FMT = "<B3xfff"  # little-endian, matches ARM Cortex-M struct layout
SENSOR_SIZE = struct.calcsize(SENSOR_FMT)  # 16 bytes

MAX_POINTS = 500  # rolling window

# Buffers
t0 = None
times = deque(maxlen=MAX_POINTS)
ax_data = deque(maxlen=MAX_POINTS)
ay_data = deque(maxlen=MAX_POINTS)
az_data = deque(maxlen=MAX_POINTS)

# UDP socket (non-blocking so animation loop doesn't stall)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("", UDP_PORT))
sock.setblocking(False)

print(f"Listening on UDP port {UDP_PORT} for {SENSOR_SIZE}-byte SensorData packets...")
print("Waiting for packets... (Ctrl+C to quit)")

# Plot setup
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
fig.suptitle("Live Acceleration (g)")
labels = ["acc_x", "acc_y", "acc_z"]
colors = ["#e74c3c", "#2ecc71", "#3498db"]
lines = []
for ax, label, color in zip(axes, labels, colors):
    (line,) = ax.plot([], [], color=color, linewidth=1.2)
    ax.set_ylabel(f"{label} (g)")
    ax.grid(True, alpha=0.3)
    lines.append(line)
axes[-1].set_xlabel("Time (s)")
fig.tight_layout()


def update(_frame):
    global t0

    # Drain all queued packets
    while True:
        try:
            data, _addr = sock.recvfrom(256)
        except BlockingIOError:
            break

        print(f"  Packet from {_addr}: {len(data)} bytes: {data.hex(' ')}")

        if len(data) < SENSOR_SIZE:
            # Try parsing as a single float if we get 4 bytes
            if len(data) >= 4:
                (val,) = struct.unpack("<f", data[:4])
                print(f"    As float: {val:.4f}")
            continue

        valid, acc_x, acc_y, acc_z = struct.unpack(SENSOR_FMT, data[:SENSOR_SIZE])
        print(f"    Parsed: valid={valid} acc=({acc_x:.3f}, {acc_y:.3f}, {acc_z:.3f})")
        if not valid:
            continue

        now = time.monotonic()
        if t0 is None:
            t0 = now

        times.append(now - t0)
        ax_data.append(acc_x)
        ay_data.append(acc_y)
        az_data.append(acc_z)

    # Update plot
    if times:
        t_list = list(times)
        for line, series in zip(lines, [ax_data, ay_data, az_data]):
            line.set_data(t_list, list(series))
        for ax in axes:
            ax.set_xlim(t_list[0], t_list[-1] + 0.01)
            ax.relim()
            ax.autoscale_view(scalex=False)

    return lines


ani = animation.FuncAnimation(fig, update, interval=10, blit=True, cache_frame_data=False)
plt.show()
sock.close()
