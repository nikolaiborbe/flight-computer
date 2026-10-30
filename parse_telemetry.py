#!/usr/bin/env python3
"""Parse a flight-computer telemetry.bin log into CSV.

The log is a stream of length-delimited entries written by FileLogger
(vendor/common/Src/Storage/file_logger.cpp). Every entry is:

    LogEntryHeader (24 bytes, packed) | payload (`size` bytes)

The entry `id` is the source's UDP broadcast port, which selects the payload
struct. This file contains three streams that together cover the fields of
interest:

    id 3001  SensorData  -> gnss, (bme), ms, icm, mmc, stim, pressure
    id 6967  BMS_Data    -> charger, (battery, rail)
    id 3002  KalmanData  -> kalman

The requested `SensorSnapshot` layout (gnss/icm/mmc/ms/pressure/stim/charger/
kalman, entry id 1) is not present in this capture, so we reconstruct the same
columns by decoding the three streams above and merging them on time: one row
per SensorData sample, with the most recent charger/kalman values forward-filled.

All sub-structs are `#pragma pack(1)`; ULONG (the icm/ms/bme timestamps) is
32-bit on the Cortex-M7 target.

Usage:
    python3 parse_telemetry.py telemetry.bin -o telemetry.csv
    python3 parse_telemetry.py telemetry.bin -o telemetry.csv --plot telemetry_last5min.png
    python3 parse_telemetry.py telemetry.bin --split
"""

import argparse
import csv
import math
import os
import struct
import sys

# --- LogEntryHeader ---------------------------------------------------------
HEADER_FMT = "<HHHHQQ"                        # magic, checksum, id, size, reserved0, timestamp_us
HEADER_SIZE = struct.calcsize(HEADER_FMT)     # 24
MAGIC = 0xBEEF

# Entry ids (UDP ports) present in the capture.
ID_SENSOR = 3001
ID_KALMAN = 3002
ID_BMS = 6967

# --- Sub-struct formats (packed, little-endian) -----------------------------
GNSS_FMT = "<BBdddddIIiii"   # data_valid,numSV, lat,lon,hMSL,gSpeed,headMot, hAcc,vAcc, velN,velE,velD
BME_FMT = "<BfffI"           # valid, temperature, pressure, humidity, timestamp
MS_FMT = "<BffI"             # valid, temperature, pressure, timestamp
ICM_FMT = "<fffffffI"        # temperature_C, gyro xyz, acc xyz (m/s^2), timestamp
MMC_FMT = "<fff"             # mag xyz (gauss)
STIM_FMT = "<ffffff"         # acc xyz (m/s^2), gyro xyz
PRESSURE_FMT = "<ff"         # flotation, separation

# SensorData (id 3001): gnss, bme, ms, icm, mmc, stim, pressure  (168 bytes)
SENSOR_FMT = "<" + "".join(f[1:] for f in
                           (GNSS_FMT, BME_FMT, MS_FMT, ICM_FMT, MMC_FMT, STIM_FMT, PRESSURE_FMT))
SENSOR_SIZE = struct.calcsize(SENSOR_FMT)

# ChargerData (start of BMS_Data, id 6967): only the charger portion is decoded.
CHARGER_FMT = "<dddddd??BB"  # vac,vbat,vsys,iac,ibat,temperature, charging,power_good,faults,state
CHARGER_SIZE = struct.calcsize(CHARGER_FMT)

# KalmanData (id 3002)
KALMAN_FMT = "<dd"           # altitude, velocity
KALMAN_SIZE = struct.calcsize(KALMAN_FMT)

SENSOR_FIELDS = [
    "gnss_data_valid", "gnss_numSV", "gnss_lat", "gnss_lon", "gnss_hMSL",
    "gnss_gSpeed", "gnss_headMot", "gnss_hAcc", "gnss_vAcc",
    "gnss_velN", "gnss_velE", "gnss_velD",
    "bme_valid", "bme_temperature", "bme_pressure", "bme_humidity", "bme_timestamp",
    "ms_valid", "ms_temperature", "ms_pressure", "ms_timestamp",
    "icm_temperature_C", "icm_gyro_x", "icm_gyro_y", "icm_gyro_z",
    "icm_acc_x_m_per_ss", "icm_acc_y_m_per_ss", "icm_acc_z_m_per_ss", "icm_timestamp",
    "mmc_mag_x_gauss", "mmc_mag_y_gauss", "mmc_mag_z_gauss",
    "stim_acc_x_m_per_ss", "stim_acc_y_m_per_ss", "stim_acc_z_m_per_ss",
    "stim_gyro_x", "stim_gyro_y", "stim_gyro_z",
    "pressure_flotation", "pressure_separation",
]
CHARGER_FIELDS = [
    "charger_vac", "charger_vbat", "charger_vsys", "charger_iac", "charger_ibat",
    "charger_temperature", "charger_charging", "charger_power_good",
    "charger_faults", "charger_state",
]
KALMAN_FIELDS = ["kalman_altitude", "kalman_velocity", "kalman_acceleration_m_per_ss"]
RAW_KALMAN_FIELDS = ["kalman_altitude", "kalman_velocity"]

TIME_CANDIDATES = [
    ("timestamp_us", 1_000_000.0),
    ("ms_timestamp", 1_000.0),
    ("bme_timestamp", 1_000.0),
    ("icm_timestamp", 1_000.0),
]
MAX_REASONABLE_SAMPLE_GAP_S = 5.0

CHARGING_STATUS = {
    0: "NotCharging", 1: "TrickleCharge", 2: "PreCharge", 3: "FastCharge",
    4: "TaperCharge", 5: "Reserved", 6: "TopOffTimerCharge", 7: "ChargeTermination",
}


def crc16_ccitt_false(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def entry_checksum(header_bytes, payload):
    # Covers the header past the `checksum` field, then the payload
    # (matches log_entry_checksum in file_logger.cpp).
    crc = crc16_ccitt_false(header_bytes[4:])  # skip magic(2) + checksum(2)
    return crc16_ccitt_false(payload, crc)


def iter_entries(buf, verify=True):
    """Yield (entry_id, timestamp_us, payload) for each valid entry."""
    n = len(buf)
    i = 0
    while i + HEADER_SIZE <= n:
        if buf[i] != (MAGIC & 0xFF) or buf[i + 1] != (MAGIC >> 8):
            i += 1
            continue
        header_bytes = buf[i:i + HEADER_SIZE]
        _magic, checksum, entry_id, size, _reserved, ts_us = struct.unpack(HEADER_FMT, header_bytes)
        end = i + HEADER_SIZE + size
        if end > n:
            break
        payload = buf[i + HEADER_SIZE:end]
        if verify and entry_checksum(header_bytes, payload) != checksum:
            i += 1  # false magic match or corruption; resync
            continue
        yield entry_id, ts_us, payload
        i = end


def decode_charger(values, raw_state):
    values = list(values)
    if not raw_state:
        values[-1] = CHARGING_STATUS.get(values[-1], values[-1])
    return values


def to_float(value):
    if value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def choose_time_source(rows):
    best = None
    for field, scale in TIME_CANDIDATES:
        values = []
        for row in rows:
            value = to_float(row.get(field, ""))
            if value is not None and value > 0:
                values.append(value / scale)
        if len(values) < 2:
            continue

        deltas = [
            b - a
            for a, b in zip(values, values[1:])
            if 0 < b - a <= MAX_REASONABLE_SAMPLE_GAP_S
        ]
        if not deltas:
            continue

        deltas.sort()
        median_delta = deltas[len(deltas) // 2]
        estimated_duration = median_delta * (len(rows) - 1)
        coverage = len(values) / len(rows)
        score = coverage * estimated_duration

        if best is None or score > best[4]:
            best = (field, scale, median_delta, estimated_duration, score)

    return best


def add_derived_time_and_acceleration(rows):
    time_source = choose_time_source(rows)
    previous_raw_time = None
    previous_time_s = None
    last_velocity = None
    last_time = None

    for idx, row in enumerate(rows):
        if time_source is not None:
            field, scale, median_delta, _duration, _score = time_source
            value = to_float(row.get(field, ""))
            raw_time_s = value / scale if value is not None and value > 0 else None
            time_s = None
            if raw_time_s is not None:
                if previous_raw_time is None:
                    time_s = raw_time_s
                else:
                    raw_dt = raw_time_s - previous_raw_time
                    if 0 <= raw_dt <= MAX_REASONABLE_SAMPLE_GAP_S:
                        time_s = previous_time_s + raw_dt

            if time_s is None and previous_time_s is not None:
                time_s = previous_time_s + median_delta

            if raw_time_s is not None:
                previous_raw_time = raw_time_s
        else:
            # Last-resort sample index keeps plotting deterministic for captures
            # where firmware did not stamp any logged stream yet.
            time_s = float(idx)

        row["time_s"] = "" if time_s is None else time_s
        if time_s is not None:
            previous_time_s = time_s

        velocity = to_float(row.get("kalman_velocity", ""))
        acceleration = None
        if velocity is not None and last_velocity is not None and time_s is not None and last_time is not None:
            dt = time_s - last_time
            if dt > 0:
                acceleration = (velocity - last_velocity) / dt

        row["kalman_acceleration_m_per_ss"] = "" if acceleration is None else acceleration

        if velocity is not None and time_s is not None:
            last_velocity = velocity
            last_time = time_s

    return time_source[0] if time_source is not None else "sample_index"


def write_merged(buf, out_path, verify, raw_state):
    header = ["time_s", "timestamp_us"] + SENSOR_FIELDS + CHARGER_FIELDS + KALMAN_FIELDS
    charger = [""] * len(CHARGER_FIELDS)
    kalman = [""] * len(RAW_KALMAN_FIELDS)
    rows = []
    base_fields = ["timestamp_us"] + SENSOR_FIELDS + CHARGER_FIELDS + RAW_KALMAN_FIELDS
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for entry_id, ts_us, payload in iter_entries(buf, verify):
            if entry_id == ID_BMS and len(payload) >= CHARGER_SIZE:
                charger = decode_charger(struct.unpack(CHARGER_FMT, payload[:CHARGER_SIZE]), raw_state)
            elif entry_id == ID_KALMAN and len(payload) == KALMAN_SIZE:
                kalman = list(struct.unpack(KALMAN_FMT, payload))
            elif entry_id == ID_SENSOR and len(payload) == SENSOR_SIZE:
                sensor = struct.unpack(SENSOR_FMT, payload)
                rows.append(dict(zip(base_fields, [ts_us] + list(sensor) + charger + kalman)))

        time_source = add_derived_time_and_acceleration(rows)
        for row in rows:
            w.writerow([row.get(field, "") for field in header])
    print(f"Merged {len(rows)} rows -> {out_path} (time source: {time_source})", file=sys.stderr)


def write_split(buf, base, verify, raw_state):
    streams = {
        ID_SENSOR: (SENSOR_FMT, SENSOR_SIZE, SENSOR_FIELDS, "sensor", None),
        ID_BMS: (CHARGER_FMT, CHARGER_SIZE, CHARGER_FIELDS, "charger", "prefix"),
        ID_KALMAN: (KALMAN_FMT, KALMAN_SIZE, RAW_KALMAN_FIELDS, "kalman", None),
    }
    writers, files = {}, {}
    counts = {k: 0 for k in streams}
    root, _ = os.path.splitext(base)
    for eid, (_fmt, _sz, fields, name, _mode) in streams.items():
        fp = open(f"{root}_{name}.csv", "w", newline="")
        files[eid] = fp
        writers[eid] = csv.writer(fp)
        writers[eid].writerow(["timestamp_us"] + fields)
    try:
        for entry_id, ts_us, payload in iter_entries(buf, verify):
            s = streams.get(entry_id)
            if not s:
                continue
            fmt, sz, _fields, _name, mode = s
            if len(payload) < sz:
                continue
            values = struct.unpack(fmt, payload[:sz])
            if mode == "prefix":  # BMS: decode only the leading ChargerData
                values = decode_charger(values, raw_state)
            writers[entry_id].writerow([ts_us] + list(values))
            counts[entry_id] += 1
    finally:
        for fp in files.values():
            fp.close()
    for eid, (_f, _s, _fl, name, _m) in streams.items():
        print(f"  {name}: {counts[eid]} rows -> {root}_{name}.csv", file=sys.stderr)


def load_csv_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def numeric_series(rows, field):
    points = []
    for row in rows:
        x = to_float(row.get("time_s", ""))
        y = to_float(row.get(field, ""))
        if x is None or y is None:
            continue
        points.append((x, y))
    points.sort(key=lambda point: point[0])
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return xs, ys


def trim_last_window(rows, minutes):
    times = [to_float(row.get("time_s", "")) for row in rows]
    valid_times = [t for t in times if t is not None]
    if not valid_times:
        return rows

    end = max(valid_times)
    start = end - minutes * 60.0
    trimmed = [row for row in rows if (to_float(row.get("time_s", "")) or -math.inf) >= start]
    return trimmed or rows


def plot_csv(csv_path, output_path, minutes=5.0, show=False):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("missing matplotlib; install it or use the tools/.venv Python") from exc

    rows = trim_last_window(load_csv_rows(csv_path), minutes)
    if not rows:
        raise SystemExit(f"no rows to plot in {csv_path}")

    fields_by_axis = [
        ("Kalman altitude", ["kalman_altitude"], "m"),
        ("Kalman velocity", ["kalman_velocity"], "m/s"),
        (
            "Acceleration",
            [
                "kalman_acceleration_m_per_ss",
                "icm_acc_x_m_per_ss",
                "icm_acc_y_m_per_ss",
                "icm_acc_z_m_per_ss",
            ],
            "m/s^2",
        ),
        ("MS5611 barometer", ["ms_pressure", "ms_temperature"], "mbar / degC"),
        ("Magnetometer", ["mmc_mag_x_gauss", "mmc_mag_y_gauss", "mmc_mag_z_gauss"], "gauss"),
    ]

    fig, axes = plt.subplots(len(fields_by_axis), 1, sharex=True, figsize=(12, 10), constrained_layout=True)
    for ax, (title, fields, ylabel) in zip(axes, fields_by_axis):
        plotted = False
        for field in fields:
            xs, ys = numeric_series(rows, field)
            if not xs:
                continue
            t0 = xs[0]
            ax.plot([(x - t0) / 60.0 for x in xs], ys, label=field)
            plotted = True
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        if plotted:
            ax.legend(loc="best", fontsize="small")
        else:
            ax.text(0.5, 0.5, "no data", transform=ax.transAxes, ha="center", va="center")

    axes[-1].set_xlabel(f"minutes since start of plotted {minutes:g} min window")
    fig.suptitle(f"Flight computer telemetry: last {minutes:g} minutes")

    if output_path:
        fig.savefig(output_path, dpi=160)
        print(f"Plot -> {output_path}", file=sys.stderr)
    if show:
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", nargs="?", default="telemetry.bin", help="input .bin file")
    ap.add_argument("-o", "--output", default="telemetry.csv", help="output .csv (merged mode)")
    ap.add_argument("--split", action="store_true",
                    help="write one CSV per stream instead of a merged table")
    ap.add_argument("--no-verify", action="store_true", help="skip CRC-16 verification")
    ap.add_argument("--raw-state", action="store_true",
                    help="keep charger_state as an integer instead of a name")
    ap.add_argument("--plot", nargs="?", const="telemetry_last5min.png",
                    help="also plot the merged CSV to this image path")
    ap.add_argument("--plot-only", action="store_true",
                    help="treat input as an existing CSV and only make the plot")
    ap.add_argument("--window-minutes", type=float, default=5.0,
                    help="minutes of data to include in the plot")
    ap.add_argument("--show", action="store_true", help="show the plot interactively")
    args = ap.parse_args()

    if args.plot_only:
        plot_csv(args.input, args.plot, args.window_minutes, args.show)
        return

    if args.split:
        with open(args.input, "rb") as f:
            buf = f.read()
        write_split(buf, args.output, not args.no_verify, args.raw_state)
    else:
        with open(args.input, "rb") as f:
            buf = f.read()
        write_merged(buf, args.output, not args.no_verify, args.raw_state)
        if args.plot or args.show:
            plot_csv(args.output, args.plot, args.window_minutes, args.show)


if __name__ == "__main__":
    main()
