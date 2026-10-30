#!/usr/bin/env python3
"""
Parser for telemetry.bin logs produced by FileLogger (Storage/file_logger.*).

The file is a flat stream of back-to-back entries. Each entry is a packed
24-byte header immediately followed by `size` bytes of payload:

    struct __attribute__((packed)) LogEntryHeader {
        uint16_t magic;         // 0xBEEF sync word, marks the start of a header
        uint16_t checksum;      // CRC-16/CCITT-FALSE, see below
        uint16_t id;            // entry type
        uint16_t size;          // payload length in bytes
        uint64_t reserved0;     // reserved, currently always 0
        uint64_t timestamp_us;  // microseconds since boot
    };
    uint8_t data[size];         // payload, immediately after the header

Everything is little-endian (Cortex-M7 / STM32H7 default).

The checksum covers every header byte *after* the checksum field (id, size,
reserved0, timestamp_us) followed by the payload, with poly 0x1021 and
init 0xFFFF (no reflection, no xorout). This matches log_entry_checksum() in
the firmware.

Because entries are size-chained with no other framing, a single corrupt byte
would normally desync the whole rest of the file. To stay robust we lean on the
two fields the firmware gives us:

  * `magic` lets us cheaply *find* candidate header starts, and
  * `checksum` lets us *confirm* a candidate really is a header.

On any bad entry we scan forward to the next 0xBEEF, and only re-lock once a
run of consecutive entries all pass their CRC. The skipped gap is reported as a
warning instead of aborting the parse.

For each entry id, the payloads are written to `<output_folder>/<id>.bin`, each
payload prefixed with its 8-byte little-endian timestamp_us and stored
sequentially.

Usage:
    python3 telemetry_parser.py telemetry.bin <output_folder>
"""

import binascii
import os
import struct
import sys
from collections import namedtuple

try:
    from tqdm import tqdm
except ImportError:
    def tqdm(iterable=None, total=None, **kwargs):
        if iterable is not None:
            return iterable

        class _NullProgressBar:
            def update(self, n=1):
                pass

            def close(self):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *exc_info):
                pass

        return _NullProgressBar()

# ---- On-disk format. Must stay in sync with LogEntryHeader in the firmware. ---
MAGIC = 0xBEEF
HEADER_FORMAT = "<HHHHQQ"  # magic, checksum, id, size, reserved0, timestamp_us
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 24
MAGIC_BYTES = struct.pack("<H", MAGIC)  # little-endian 0xBEEF -> b"\xef\xbe"

# The checksum covers the header starting at `id` (right after `checksum`).
CRC_COVER_START = 4
MAX_LOG_DATA_SIZE = 512  # must match max_log_data_size in firmware

# Per-packet framing prepended to every payload written to <id>.bin: the
# entry's timestamp_us as a little-endian uint64.
TIMESTAMP_FORMAT = "<Q"

# How many consecutive CRC-valid entries a candidate must yield before we trust
# it as a resync point. A single CRC-16 match has a ~1/65536 chance of being a
# coincidence; requiring two chained matches drops that to ~1/4e9.
RESYNC_CONFIRM_ENTRIES = 2

# Cap on how many per-desync warnings we print (a badly mangled file could
# otherwise produce thousands).
MAX_WARNINGS_SHOWN = 40


Entry = namedtuple("Entry", "id payload timestamp end")


def entry_crc(mv, offset, end):
    """CRC-16/CCITT-FALSE over the covered header bytes followed by the payload."""
    crc = binascii.crc_hqx(mv[offset + CRC_COVER_START: offset + HEADER_SIZE], 0xFFFF)
    crc = binascii.crc_hqx(mv[offset + HEADER_SIZE: end], crc)
    return crc


def try_parse_entry(data, mv, offset):
    """
    Try to read one valid, CRC-verified entry that starts exactly at `offset`.

    Returns an Entry on success, or None if there is no structurally valid,
    checksum-matching entry at that position.
    """
    if offset + HEADER_SIZE > len(data):
        return None

    magic, checksum, entry_id, size, _reserved0, timestamp = struct.unpack_from(
        HEADER_FORMAT, data, offset
    )

    if magic != MAGIC or size > MAX_LOG_DATA_SIZE:
        return None

    end = offset + HEADER_SIZE + size
    if end > len(data):
        return None

    if entry_crc(mv, offset, end) != checksum:
        return None

    return Entry(entry_id, mv[offset + HEADER_SIZE: end], timestamp, end)


def confirms_resync(data, mv, offset):
    """True if `offset` yields a run of consecutive CRC-valid entries (or ends cleanly)."""
    cursor = offset
    for confirmed in range(RESYNC_CONFIRM_ENTRIES):
        entry = try_parse_entry(data, mv, cursor)
        if entry is None:
            # Ran out early: only trust it if we already validated at least one
            # entry and landed exactly on end-of-file.
            return confirmed >= 1 and cursor == len(data)
        cursor = entry.end
    return True


def find_resync(data, mv, start):
    """Scan forward from `start` for the next confirmable header. None if there is none."""
    search = start
    while True:
        idx = data.find(MAGIC_BYTES, search)
        if idx < 0:
            return None
        if confirms_resync(data, mv, idx):
            return idx
        search = idx + 1


def open_stream(streams, out_dir, entry_id):
    stream = streams.get(entry_id)
    if stream is None:
        fh = open(os.path.join(out_dir, f"telemetry_{entry_id}.bin"), "wb")
        stream = {"fh": fh, "count": 0, "min": None, "max": None, "total": 0}
        streams[entry_id] = stream
    return stream


def parse_telemetry(data, out_dir):
    """
    Parse `data`, writing per-id streams into `out_dir`.

    Returns a stats dict describing what was recovered and skipped.
    """
    mv = memoryview(data)
    total = len(data)

    streams = {}
    warnings = []
    resync_events = 0
    skipped_bytes = 0
    entry_count = 0

    offset = 0
    with tqdm(total=total, unit="B", unit_scale=True, desc="Parsing entries") as pbar:
        while offset < total:
            entry = try_parse_entry(data, mv, offset)
            if entry is not None:
                stream = open_stream(streams, out_dir, entry.id)
                stream["fh"].write(struct.pack(TIMESTAMP_FORMAT, entry.timestamp // 1000))
                stream["fh"].write(entry.payload)
                n = len(entry.payload)
                stream["count"] += 1
                stream["total"] += n
                stream["min"] = n if stream["min"] is None else min(stream["min"], n)
                stream["max"] = n if stream["max"] is None else max(stream["max"], n)
                entry_count += 1
                pbar.update(entry.end - offset)
                offset = entry.end
                continue

            # Desync: hunt for the next confirmable header.
            resync = find_resync(data, mv, offset + 1)
            if resync is None:
                remaining = total - offset
                warnings.append(
                    f"Desync at offset {offset}: no further valid entries found; "
                    f"discarding {remaining} trailing byte(s)."
                )
                skipped_bytes += remaining
                pbar.update(remaining)
                break

            gap = resync - offset
            skipped_bytes += gap
            resync_events += 1
            warnings.append(
                f"Desync at offset {offset}: skipped {gap} byte(s), resynced at offset {resync}."
            )
            pbar.update(gap)
            offset = resync

    for stream in tqdm(streams.values(), total=len(streams), desc="Closing output streams", unit="stream"):
        stream["fh"].close()

    return {
        "streams": streams,
        "warnings": warnings,
        "resync_events": resync_events,
        "skipped_bytes": skipped_bytes,
        "entry_count": entry_count,
        "total_bytes": total,
    }


def print_report(stats):
    streams = stats["streams"]

    print(f"{'id':>6} | {'count':>7} | {'min':>6} | {'max':>6} | {'avg':>7} | {'total bytes':>11}")
    print("-" * 55)

    payload_bytes = 0
    for entry_id, s in sorted(streams.items()):
        avg = s["total"] / s["count"] if s["count"] else 0
        payload_bytes += s["total"]
        print(
            f"{entry_id:>6} | {s['count']:>7} | {s['min']:>6} | {s['max']:>6} | "
            f"{avg:>7.1f} | {s['total']:>11}"
        )

    print("-" * 55)
    print(
        f"{len(streams)} unique id(s), {stats['entry_count']} entries total, "
        f"{payload_bytes} payload bytes total"
    )

    recovered = stats["total_bytes"] - stats["skipped_bytes"]
    pct = (100.0 * recovered / stats["total_bytes"]) if stats["total_bytes"] else 100.0
    print(
        f"\nParsed {recovered} / {stats['total_bytes']} byte(s) ({pct:.2f}%); "
        f"{stats['resync_events']} resync event(s), {stats['skipped_bytes']} byte(s) skipped."
    )

    warnings = stats["warnings"]
    if warnings:
        print("\nWarnings:")
        for w in warnings[:MAX_WARNINGS_SHOWN]:
            print(f"  - {w}")
        if len(warnings) > MAX_WARNINGS_SHOWN:
            print(f"  ... and {len(warnings) - MAX_WARNINGS_SHOWN} more.")
    else:
        print("\nNo corruption detected; parsed cleanly.")


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <telemetry.bin> <output_folder>")
        sys.exit(1)

    path = sys.argv[1]
    out_dir = sys.argv[2]

    with open(path, "rb") as f:
        data = f.read()

    os.makedirs(out_dir, exist_ok=True)
    stats = parse_telemetry(data, out_dir)
    print_report(stats)


if __name__ == "__main__":
    main()
