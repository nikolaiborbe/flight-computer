import math
import queue
import re
import socket
import struct
import sys
import threading
import time

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import pandas as pd

# ----- settings -----------------------------------------------------------
IP_ADDRESS = "10.19.0.110"  # pre-fill a default IP here if you like
DEFAULT_PORT = 25567        # CommandManager::kLocalPort (control channel, UDP)
PACKET_DELAY = 0.1          # seconds to wait between sensor-data packets

COMMAND_ID_FSM_STATE = 0x10        # Commands::FSMStateCommand (command_objects.hpp "#id=0x10")
COMMAND_ID_TEST_SENSOR_DATA = 0x20  # TestStateCommandService kCommandID (test_state_command_service.cpp)
COMMAND_DELAY = 1.0      # seconds between FSM state command packets while arming

# Commands::StateCommands (command_objects.hpp) -- NOT the same enum as STATE_NAME
# (Data/GroundStation.hpp)/STATE_NAMES below. The command byte is an *event* fed into
# FsmStateCommandService::handle() (fsm_state_command_service.cpp), which the FSM
# then maps to a state transition per state_initilizer.cpp's event tables.
STATE_COMMAND_IDLE = 0            # -> EVENT::IDLE (any armed state falls back to IDLE)
STATE_COMMAND_EXPECT_LIFTOFF = 3  # -> EVENT::EXPECT_LIFTOFF (IDLE -> READY, only from IDLE)

KALMAN_PORT = 3002  # Telemetry::GroundStation::KalmanData broadcast (Networking/services/kalman_service.cpp)
STATUS_PORT = 3003  # Telemetry::GroundStation::SystemStatus broadcast, carries current_state (status_service.cpp)
LISTENER_DRAIN_TIME = 1.0  # seconds to keep listening after the last packet is sent

# Standard-atmosphere defaults used to invert Utils::pressure_to_altitude()
# (App/Src/Utilities/math.cpp) so raw_altitude (meters) can be turned back into
# the pressure readings the firmware's sensor structs actually carry.
SEA_LEVEL_PRESSURE_HPA = 1013.25
SEA_LEVEL_TEMPERATURE_C = 15.0
TEMPERATURE_LAPSE_RATE_K_PER_M = 0.0065
BAROMETRIC_EXPONENT = 0.1902631
KELVIN_OFFSET = 273.15
# --------------------------------------------------------------------------

# Order must match the STATE_NAME enum in App/Inc/Data/GroundStation.hpp.
STATE_NAMES = [
    "NONE", "IDLE", "READY", "FILLING", "SEPARATION_FILLING", "FLOTATION_FILLING",
    "THRUSTING", "COASTING", "SEPARATION", "FLOTATION", "SHUTDOWN",
    "TEST_SEPARATION", "TEST_FLOTATION",
]

# ----- wire formats (App/Inc/Data/GroundStation.hpp, all structs are #pragma pack(1)) -------
_GNSS_FMT = "<BBdddddIIiii"  # data_valid, numSV, lat, lon, hMSL, gSpeed, headMot, hAcc, vAcc, velN, velE, velD
_BME280_FMT = "<BfffI"       # valid, temperature(C), pressure(Pa), humidity, timestamp
_MS5611_FMT = "<BffI"        # valid, temperature(C), pressure(mbar), timestamp
_ICM_FMT = "<fffffff"        # acc_x,acc_y,acc_z (m/s^2), gyro_x,gyro_y,gyro_z, temperature_C
_STIM_FMT = "<ffffff"        # acc_x,acc_y,acc_z (m/s^2), gyro_x,gyro_y,gyro_z
_PRESSURE_FMT = "<ff"        # flotation_pressure, separation_pressure

SENSOR_DATA_SIZE = sum(
    struct.calcsize(fmt) for fmt in (_GNSS_FMT, _BME280_FMT, _MS5611_FMT, _ICM_FMT, _STIM_FMT, _PRESSURE_FMT)
)

_KALMAN_FMT = "<dd"  # altitude, velocity
_KALMAN_SIZE = struct.calcsize(_KALMAN_FMT)

# ActuatorStatus(4 bool) + SensorStatus(7 bool) + HardwareStatus(7 bool). The
# trailing STATE_NAME field is a scoped enum with no fixed underlying type, so
# instead of hard-coding its width we just take however many bytes are left in
# the packet (see unpack_status()).
_STATUS_BOOL_FMT = "<4?7?7?"
_STATUS_BOOL_SIZE = struct.calcsize(_STATUS_BOOL_FMT)


def is_valid_ip(ip):
    """Basic IPv4 sanity check (four 0-255 octets)."""
    parts = ip.split(".")
    if len(parts) != 4:
        return False
    for part in parts:
        if not part.isdigit() or not 0 <= int(part) <= 255:
            return False
    return True


def get_ip():
    """Confirm the pre-set IP with Y/n, or ask for a new one."""
    if IP_ADDRESS:
        while True:
            answer = input(f"Current ip address is: {IP_ADDRESS} [Y/n] ").strip().lower()
            if answer in ("", "y", "yes"):
                return IP_ADDRESS          # keep the current IP
            if answer in ("n", "no"):
                break                      # fall through to enter a new one
            print("  Please answer y or n.")
    while True:
        ip = input("Enter the target IP address: ").strip()
        if is_valid_ip(ip):
            return ip
        print("  That is not a valid IPv4 address. Try again.")


def get_port():
    """Ask for the control port, defaulting to DEFAULT_PORT on empty input."""
    while True:
        raw = input(f"Enter the port [{DEFAULT_PORT}]: ").strip()
        if raw == "":
            return DEFAULT_PORT
        if raw.isdigit() and 1 <= int(raw) <= 65535:
            return int(raw)
        print("  Port must be a whole number between 1 and 65535.")


MILLIG_TO_MS2 = 9.80665 / 1000.0

# (column-name pattern, transform to m/s^2 in the firmware's convention: positive,
# ~+9.8 at rest, exceeding +3*G during thrust -- see App/Src/Threads/Fsm/States/ready.cpp)
# checked in order, first match wins.
ALT_COLUMNS = [
    (re.compile(r"^raw_altitude$"), lambda s: s),
    (re.compile(r"^BARO\d*altitude$"), lambda s: s),
    (re.compile(r"^baro_alt_m$"), lambda s: s),  # already meters
]
ACCEL_COLUMNS = [
    (re.compile(r"^fsm_accel$"), lambda s: s),
    (re.compile(r"^IMU\d*accel_y$"), lambda s: s),  # Excel fixtures: already ~9.8 m/s^2 at rest
    # FC_DATA_2025.csv: milli-g, and this is the axis that actually reads the
    # thrust event (iim_accel_y is small/noisy throughout); sign is flipped
    # because this axis reads ~-991 mg at rest, ~-6000 mg during thrust, the
    # opposite of the "positive, rising past +3*G" convention above.
    (re.compile(r"^iim_accel_x$"), lambda s: -s * MILLIG_TO_MS2),
]


def _match_column(df, specs):
    for pattern, transform in specs:
        col = next((c for c in df.columns if pattern.match(c)), None)
        if col is not None:
            return col, transform
    return None, None


def load_data():
    """Ask for a .csv or .xlsx path and load it into a DataFrame with raw_altitude/fsm_accel columns.

    Accepts the canonical raw_altitude/fsm_accel columns, the per-sensor
    Excel columns (BARO<n>altitude, IMU<n>accel_y), or the flight-log CSV
    columns (baro_alt_m, iim_accel_x) -- always the raw sensor input (not
    baro_pa/kf_alt/kf_vel) so the Kalman filter's own output can be compared
    against what was actually sent. All other columns (including kf_alt/
    kf_vel, if present) are preserved for reference.
    """
    while True:
        path = input("Path to the sensor data file to send (.csv or .xlsx): ").strip().strip('"')
        try:
            df = pd.read_csv(path) if path.lower().endswith(".csv") else pd.read_excel(path)
        except FileNotFoundError:
            print("  File not found. Check the path and try again.")
            continue
        except Exception as exc:
            print(f"  Could not read that file: {exc}")
            continue

        alt_col, alt_transform = _match_column(df, ALT_COLUMNS)
        accel_col, accel_transform = _match_column(df, ACCEL_COLUMNS)
        if alt_col is None or accel_col is None:
            print(
                "  Could not find altitude/accel columns. Expected raw_altitude/fsm_accel, "
                "BARO<n>altitude/IMU<n>accel_y, or baro_alt_m/iim_accel_x. Try again."
            )
            continue

        df = df.copy()
        df["raw_altitude"] = alt_transform(df[alt_col])
        df["fsm_accel"] = accel_transform(df[accel_col])
        return df


def get_send_row_count(total_rows, default=2000):
    """Ask how many rows (evenly spaced across the full file) to stream.

    Flight-log CSVs can be ~100 Hz over many minutes; sending every row at
    PACKET_DELAY would take hours, so this downsamples evenly across the
    whole file (instead of truncating to the first N rows) so pad, thrust,
    coast, and landing all still show up in a shorter run.
    """
    while True:
        raw = input(f"Rows to send, evenly spaced across all {total_rows} (blank = {min(default, total_rows)}): ").strip()
        if raw == "":
            return min(default, total_rows)
        if raw.isdigit() and int(raw) > 0:
            return min(int(raw), total_rows)
        print("  Enter a positive whole number, or leave blank for the default.")


FLIGHT_WINDOW_ALT_M = 20.0   # sustained altitude that marks "definitely flying"
FLIGHT_WINDOW_PAD_S = 5.0    # pad time to keep before the detected liftoff


def prepare_replay(df):
    """Pick which rows to stream and at what stride.

    The firmware's Kalman thread integrates at a fixed dt (KALMAN_THREAD_HZ),
    so the replay is only physically meaningful if data time advances at the
    same rate as wall time: one packet every PACKET_DELAY carrying samples
    PACKET_DELAY apart. Evenly striding N rows out of the whole file compresses
    time (a full flight replayed 6x too fast overshoots apogee ~13% and scales
    velocity by the compression factor -- verified by simulating
    kalmanfilter.c against FC_DATA_2025.csv both ways).

    So when the file has a time_s column, the default is: trim to the flight
    window (a few seconds of pad, then liftoff through landing) and stride so
    replay time == real time. The even-stride row-count prompt remains as the
    fallback, for files without timestamps or for a deliberate quick pass.
    """
    total_rows = len(df)
    if "time_s" in df.columns:
        dt = df["time_s"].diff().median()
        if pd.notna(dt) and dt > 0:
            stride = max(1, round(PACKET_DELAY / dt))
            start = 0
            if "raw_altitude" in df.columns:
                sustained = df["raw_altitude"].rolling(20).min() > FLIGHT_WINDOW_ALT_M
                if sustained.any():
                    start = max(0, int(sustained.idxmax()) - int(FLIGHT_WINDOW_PAD_S / dt))
            windowed = df.iloc[start::stride].reset_index(drop=True)
            duration_s = len(windowed) * PACKET_DELAY
            answer = input(
                f"Replay flight window in real time ({len(windowed)} packets, ~{duration_s:.0f}s)? [Y/n] "
            ).strip().lower()
            if answer in ("", "y", "yes"):
                return windowed
            print("  Falling back to even-stride replay (note: compresses time, distorts velocity).")

    send_count = get_send_row_count(total_rows)
    step = max(1, total_rows // send_count)
    return df.iloc[::step].reset_index(drop=True)


def send_command(sock, addr, command_id, payload=b""):
    """Send one UDP command packet: [command_id][payload...] (command_manager.cpp framing)."""
    sock.sendto(bytes([command_id]) + payload, addr)


def altitude_to_pressure_hpa(altitude_m, starting_temperature_c=SEA_LEVEL_TEMPERATURE_C,
                              starting_pressure_hpa=SEA_LEVEL_PRESSURE_HPA):
    """Inverse of Utils::pressure_to_altitude (App/Src/Utilities/math.cpp)."""
    starting_temperature_k = starting_temperature_c + KELVIN_OFFSET
    base = 1.0 - (altitude_m * TEMPERATURE_LAPSE_RATE_K_PER_M) / starting_temperature_k
    base = max(base, 0.0)  # guard against altitudes above the model's validity range
    return starting_pressure_hpa * (base ** (1.0 / BAROMETRIC_EXPONENT))


def pack_sensor_data(altitude_m, accel_z, timestamp_ms):
    """Builds one Telemetry::GroundStation::SensorData packet (Data/GroundStation.hpp) from a data row.

    Only the fields the firmware currently reads are given real values:
    icm_data.acc_m_per_ss.acc_z_m_per_ss (Fsm/States/ready.cpp liftoff detection)
    and the ms/bme pressure readings (raw_altitude run back through the
    barometric formula). GNSS and STIM377H have no equivalent column and are
    zeroed / marked invalid.
    """
    pressure_hpa = altitude_to_pressure_hpa(altitude_m)
    ts = int(timestamp_ms) & 0xFFFFFFFF

    gnss = struct.pack(_GNSS_FMT, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0)
    bme = struct.pack(_BME280_FMT, 1, SEA_LEVEL_TEMPERATURE_C, pressure_hpa * 100.0, 0.0, ts)
    ms = struct.pack(_MS5611_FMT, 1, SEA_LEVEL_TEMPERATURE_C, pressure_hpa, ts)
    icm = struct.pack(_ICM_FMT, 0.0, 0.0, accel_z, 0.0, 0.0, 0.0, 0.0)
    stim = struct.pack(_STIM_FMT, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    pressure = struct.pack(_PRESSURE_FMT, 0.0, 0.0)

    packet = gnss + bme + ms + icm + stim + pressure
    assert len(packet) == SENSOR_DATA_SIZE
    return packet


def unpack_kalman(data):
    if len(data) < _KALMAN_SIZE:
        return None
    altitude, velocity = struct.unpack_from(_KALMAN_FMT, data)
    return {"altitude": altitude, "velocity": velocity}


def unpack_status(data):
    if len(data) <= _STATUS_BOOL_SIZE:
        return None
    state_bytes = data[_STATUS_BOOL_SIZE:]
    state_id = int.from_bytes(state_bytes, "little")
    state_name = STATE_NAMES[state_id] if 0 <= state_id < len(STATE_NAMES) else f"UNKNOWN({state_id})"
    return {"state_id": state_id, "state_name": state_name}


def listen_udp(port, stop_event, out_queue, unpack_fn, label):
    """Binds a UDP port and pushes (wall_clock_time, record) onto out_queue until stopped."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("", port))
    except OSError as exc:
        print(f"  Could not listen for {label} telemetry on UDP :{port}: {exc}")
        return
    sock.settimeout(0.5)

    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break

        record = unpack_fn(data)
        if record is not None:
            out_queue.put((time.time(), record))

    sock.close()


def wait_for_state(status_queue, wanted_state, timeout_s=3.0):
    """Waits for the device to report an FSM state, up to timeout_s.

    Returns (last_state_or_None, consumed) where consumed holds the raw
    (wall_time, record) tuples pulled off the queue, so the caller can fold
    them back into the final status record list for plotting.
    """
    consumed = []
    last_state = None
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            ts, record = status_queue.get(timeout=0.2)
        except queue.Empty:
            continue
        consumed.append((ts, record))
        last_state = record["state_name"]
        if last_state == wanted_state:
            break
    return last_state, consumed


def drain(q, t0):
    """Empties a listener queue into a time-sorted list of records with relative time 't'."""
    records = []
    while True:
        try:
            ts, record = q.get_nowait()
        except queue.Empty:
            break
        record = dict(record)
        record["t"] = ts - t0
        records.append(record)
    records.sort(key=lambda r: r["t"])
    return records


# ----- plotting -------------------------------------------------------------
# Palette values from the dataviz skill's reference palette (categorical hues,
# assigned in fixed order -- never cycled by rank -- plus chart chrome/ink).
CATEGORICAL_PALETTE = [
    "#2a78d6", "#1baf7a", "#eda100", "#008300", "#4a3aa7", "#e34948", "#e87ba4", "#eb6834",
]
INK = "#0b0b0b"
SECONDARY_INK = "#52514e"
MUTED = "#898781"
GRIDLINE = "#e1e0d9"
BASELINE = "#c3c2b7"
SURFACE = "#fcfcfb"


def build_state_segments(status_records, end_t):
    """Turns a list of {'t', 'state_name'} samples into contiguous (start, end, state) runs."""
    segments = []
    prev_state = None
    seg_start = None
    for record in status_records:
        if record["state_name"] != prev_state:
            if prev_state is not None:
                segments.append((seg_start, record["t"], prev_state))
            prev_state = record["state_name"]
            seg_start = record["t"]
    if prev_state is not None:
        segments.append((seg_start, end_t, prev_state))
    return segments


RAW_ALTITUDE_COLOR = CATEGORICAL_PALETTE[0]
RAW_ACCEL_COLOR = CATEGORICAL_PALETTE[2]
LOGGED_KALMAN_COLOR = CATEGORICAL_PALETTE[4]


def plot_results(kalman_records, status_records, sent_df=None, logged_kalman_df=None):
    if not kalman_records:
        print("No Kalman telemetry was captured; skipping plot.")
        return

    kdf = pd.DataFrame(kalman_records).sort_values("t")
    end_t = kdf["t"].max()
    if sent_df is not None and not sent_df.empty:
        end_t = max(end_t, sent_df["t"].max())
    if logged_kalman_df is not None and not logged_kalman_df.empty:
        end_t = max(end_t, logged_kalman_df["t"].max())
    segments = build_state_segments(status_records, end_t)

    # Color assigned by the state's fixed position in STATE_NAMES (identity),
    # never by order of appearance in this run.
    observed_states = [s for s in STATE_NAMES if any(seg[2] == s for seg in segments)]
    color_by_state = {s: CATEGORICAL_PALETTE[i % len(CATEGORICAL_PALETTE)] for i, s in enumerate(observed_states)}

    fig, (ax_alt, ax_vel) = plt.subplots(2, 1, sharex=True, figsize=(11, 7), facecolor=SURFACE)

    for ax, col, ylabel in ((ax_alt, "altitude", "Altitude (m)"), (ax_vel, "velocity", "Velocity (m/s)")):
        ax.set_facecolor(SURFACE)
        for start, end, state in segments:
            ax.axvspan(start, end, color=color_by_state.get(state, MUTED), alpha=0.18, linewidth=0)
        ax.plot(kdf["t"], kdf[col], color=INK, linewidth=2, label="Kalman " + col)
        if logged_kalman_df is not None and not logged_kalman_df.empty:
            ax.plot(
                logged_kalman_df["t"], logged_kalman_df[col], color=LOGGED_KALMAN_COLOR,
                linewidth=1.3, linestyle=":", label="Flight-log Kalman " + col,
            )
        ax.set_ylabel(ylabel, color=SECONDARY_INK)
        ax.grid(True, color=GRIDLINE, linewidth=0.8)
        for spine in ax.spines.values():
            spine.set_color(BASELINE)
        ax.tick_params(colors=MUTED)

    ax_accel = None
    if sent_df is not None and not sent_df.empty:
        ax_alt.plot(
            sent_df["t"], sent_df["raw_altitude"], color=RAW_ALTITUDE_COLOR,
            linewidth=1.3, linestyle="--", label="Raw altitude (sent)",
        )
        ax_accel = ax_vel.twinx()
        ax_accel.plot(
            sent_df["t"], sent_df["fsm_accel"], color=RAW_ACCEL_COLOR,
            linewidth=1.3, linestyle="--", label="Raw accel_z (sent)",
        )
        ax_accel.set_ylabel("Accel input (m/s²)", color=RAW_ACCEL_COLOR)
        ax_accel.tick_params(colors=RAW_ACCEL_COLOR)
        ax_accel.spines["right"].set_color(RAW_ACCEL_COLOR)

    for ax in (ax_alt, ax_vel):
        handles, labels = ax.get_legend_handles_labels()
        if ax is ax_vel and ax_accel is not None:
            twin_handles, twin_labels = ax_accel.get_legend_handles_labels()
            handles, labels = handles + twin_handles, labels + twin_labels
        ax.legend(handles, labels, loc="upper left", frameon=False, labelcolor=SECONDARY_INK, fontsize=9)

    ax_vel.set_xlabel("Time since stream start (s)", color=SECONDARY_INK)
    fig.suptitle("Kalman-filtered altitude & velocity vs. raw sensor input & FSM state", color=INK)

    if color_by_state:
        handles = [mpatches.Patch(color=c, alpha=0.4, label=s) for s, c in color_by_state.items()]
        fig.legend(
            handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.0),
            ncol=min(len(handles), 4), frameon=False, labelcolor=SECONDARY_INK,
        )
    else:
        print("No status/state telemetry was captured; plotting altitude & velocity without state shading.")

    fig.tight_layout(rect=(0, 0.1, 1, 1))
    plt.show()


def main():
    ip = get_ip()
    port = get_port()
    df = load_data()
    total_rows = len(df)
    df = prepare_replay(df)

    has_logged_kalman = {"kf_alt", "kf_vel"}.issubset(df.columns)
    print(f"\nLoaded {len(df)} rows to send (of {total_rows} in the file).")

    control_addr = (ip, port)
    control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    stop_event = threading.Event()
    kalman_queue = queue.Queue()
    status_queue = queue.Queue()

    kalman_thread = threading.Thread(
        target=listen_udp, args=(KALMAN_PORT, stop_event, kalman_queue, unpack_kalman, "Kalman"), daemon=True
    )
    status_thread = threading.Thread(
        target=listen_udp, args=(STATUS_PORT, stop_event, status_queue, unpack_status, "Status"), daemon=True
    )
    kalman_thread.start()
    status_thread.start()
    print(f"Listening for Kalman telemetry on UDP :{KALMAN_PORT} and status/state telemetry on UDP :{STATUS_PORT}\n")

    t0 = time.time()

    # --- arm for the test -------------------------------------------------
    # Mirror the FSM's own startup path (App/Src/Threads/Fsm/fsm.cpp:
    # FIRST_STATE = IDLE, then state_initilizer.cpp's IDLE -> READY transition
    # on EVENT::EXPECT_LIFTOFF) instead of jumping straight to a mid-flight state.
    try:
        for label, command_value in (("IDLE", STATE_COMMAND_IDLE), ("READY (EXPECT_LIFTOFF)", STATE_COMMAND_EXPECT_LIFTOFF)):
            send_command(control_sock, control_addr, COMMAND_ID_FSM_STATE, bytes([command_value]))
            print(f"Sent FSM state command: {label}")
            time.sleep(COMMAND_DELAY)
    except OSError as exc:
        print(f"Connection problem while arming: {exc}")
        stop_event.set()
        control_sock.close()
        sys.exit(1)

    # Confirm the device actually reached READY before streaming. If a previous
    # run left the FSM mid-flight-chain or in SHUTDOWN, neither IDLE nor
    # EXPECT_LIFTOFF can leave those states (state_initilizer.cpp has no such
    # transitions) and only a power-cycle recovers -- without this check the
    # stream would run to completion with the state never changing.
    confirmed_state, prestream_status = wait_for_state(status_queue, "READY")
    if confirmed_state is None:
        print("WARNING: no status telemetry on UDP :3003 -- cannot confirm the FSM state. Continuing anyway.")
    elif confirmed_state != "READY":
        print(f"WARNING: device reports state {confirmed_state}, not READY.")
        print(f"         IDLE/EXPECT_LIFTOFF cannot leave {confirmed_state}; power-cycle the board and rerun.")
    else:
        print("Device confirmed READY.")
    # ----------------------------------------------------------------------

    # send one row == one SensorData packet
    print("\nStreaming data...")
    sent = 0
    sent_records = []
    logged_kalman_records = []
    try:
        for _, row in df.iterrows():
            altitude = float(row["raw_altitude"])
            accel = float(row["fsm_accel"])
            elapsed_ms = (time.time() - t0) * 1000.0
            t = elapsed_ms / 1000.0

            payload = pack_sensor_data(altitude, accel, elapsed_ms)
            send_command(control_sock, control_addr, COMMAND_ID_TEST_SENSOR_DATA, payload)

            sent_records.append({"t": t, "raw_altitude": altitude, "fsm_accel": accel})
            if has_logged_kalman:
                logged_kalman_records.append({"t": t, "altitude": float(row["kf_alt"]), "velocity": float(row["kf_vel"])})
            sent += 1
            print(f"Sent {sent}/{len(df)}: altitude={altitude:.2f} m, accel_z={accel:.2f} m/s^2")

            time.sleep(PACKET_DELAY)
    except OSError as exc:
        print(f"\nConnection problem after {sent} packet(s): {exc}")
    finally:
        print(f"\nDone streaming. Sent {sent} packet(s). Waiting {LISTENER_DRAIN_TIME}s for trailing telemetry...")
        time.sleep(LISTENER_DRAIN_TIME)
        stop_event.set()
        kalman_thread.join(timeout=2.0)
        status_thread.join(timeout=2.0)
        control_sock.close()

    kalman_records = drain(kalman_queue, t0)
    status_records = [dict(record, t=ts - t0) for ts, record in prestream_status] + drain(status_queue, t0)
    status_records.sort(key=lambda r: r["t"])
    print(f"Captured {len(kalman_records)} Kalman sample(s) and {len(status_records)} status sample(s).")

    sent_df = pd.DataFrame(sent_records)
    logged_kalman_df = pd.DataFrame(logged_kalman_records) if logged_kalman_records else None
    plot_results(kalman_records, status_records, sent_df, logged_kalman_df)


if __name__ == "__main__":
    main()
