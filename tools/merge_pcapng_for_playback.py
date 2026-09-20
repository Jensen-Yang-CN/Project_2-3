#!/usr/bin/env python3
"""Merge selected pcapng captures into one chronologically playable .pcap file.

The application opens files through libpcap, so the output deliberately keeps
the .pcap suffix even though its container is pcapng.  Only Enhanced Packet
Blocks are copied; packet bytes, capture timestamps, and original lengths are
preserved.  A fresh Ethernet interface description is written once.
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path


SHB = 0x0A0D0D0A
IDB = 0x00000001
EPB = 0x00000006
PCAPNG_BOM_LE = b"\x4d\x3c\x2b\x1a"
PCAPNG_BOM_BE = b"\x1a\x2b\x3c\x4d"


def read_block(stream, endian: str, first: bool = False):
    header = stream.read(8)
    if not header:
        return None
    if len(header) != 8:
        raise ValueError("truncated pcapng block header")
    block_type = struct.unpack_from(endian + "I", header, 0)[0]
    total_length = struct.unpack_from(endian + "I", header, 4)[0]
    if total_length < 12 or total_length % 4 != 0:
        raise ValueError(f"invalid pcapng block length: {total_length}")
    body_and_tail = stream.read(total_length - 8)
    if len(body_and_tail) != total_length - 8:
        raise ValueError("truncated pcapng block")
    body = body_and_tail[:-4]
    trailing_length = struct.unpack_from(endian + "I", body_and_tail, len(body))[0]
    if trailing_length != total_length:
        raise ValueError("pcapng block length trailer mismatch")
    if first and block_type != SHB:
        raise ValueError("input is not a pcapng Section Header Block")
    return block_type, body


def iter_enhanced_packets(path: Path):
    with path.open("rb") as stream:
        endian = "<"
        first = True
        interface_count = 0
        link_type = None
        snaplen = None
        while True:
            block = read_block(stream, endian, first=first)
            if block is None:
                return link_type, snaplen
            block_type, body = block
            if first:
                if body[0:4] == PCAPNG_BOM_LE:
                    endian = "<"
                elif body[0:4] == PCAPNG_BOM_BE:
                    endian = ">"
                else:
                    raise ValueError("invalid pcapng byte-order magic")
                first = False
                continue
            if block_type == IDB:
                if len(body) < 8:
                    raise ValueError("truncated interface description block")
                current_link_type = struct.unpack_from(endian + "H", body, 0)[0]
                current_snaplen = struct.unpack_from(endian + "I", body, 4)[0]
                if interface_count == 0:
                    link_type = current_link_type
                    snaplen = current_snaplen
                elif (current_link_type, current_snaplen) != (link_type, snaplen):
                    raise ValueError("input contains incompatible interfaces")
                interface_count += 1
                continue
            if block_type != EPB:
                continue
            if len(body) < 20:
                raise ValueError("truncated enhanced packet block")
            interface_id, timestamp_high, timestamp_low, captured_length, original_length = struct.unpack_from(
                endian + "IIIII", body, 0
            )
            if interface_id != 0:
                raise ValueError("only interface 0 is supported")
            packet_start = 20
            packet_end = packet_start + captured_length
            if packet_end > len(body):
                raise ValueError("enhanced packet length exceeds block")
            packet = body[packet_start:packet_end]
            yield timestamp_high, timestamp_low, captured_length, original_length, packet


def inspect_interface(path: Path) -> tuple[int, int]:
    with path.open("rb") as stream:
        endian = "<"
        first = True
        while True:
            block = read_block(stream, endian, first=first)
            if block is None:
                break
            block_type, body = block
            if first:
                if body[0:4] == PCAPNG_BOM_LE:
                    endian = "<"
                elif body[0:4] == PCAPNG_BOM_BE:
                    endian = ">"
                else:
                    raise ValueError("invalid pcapng byte-order magic")
                first = False
                continue
            if block_type == IDB:
                if len(body) < 8:
                    raise ValueError("truncated interface description block")
                return (
                    struct.unpack_from(endian + "H", body, 0)[0],
                    struct.unpack_from(endian + "I", body, 4)[0],
                )
    raise ValueError(f"input has no interface description block: {path}")


def write_block(stream, block_type: int, body: bytes) -> None:
    padded_length = (len(body) + 3) & ~3
    total_length = 12 + padded_length
    stream.write(struct.pack("<II", block_type, total_length))
    stream.write(body)
    stream.write(b"\x00" * (padded_length - len(body)))
    stream.write(struct.pack("<I", total_length))


def write_section_header(stream) -> None:
    body = struct.pack("<IHHq", 0x1A2B3C4D, 1, 0, -1)
    write_block(stream, SHB, body)


def write_interface_description(stream, link_type: int, snaplen: int) -> None:
    write_block(stream, IDB, struct.pack("<HHI", link_type, 0, snaplen))


def write_packet(stream, packet) -> None:
    timestamp_high, timestamp_low, captured_length, original_length, data = packet
    body = struct.pack(
        "<IIIII",
        0,
        timestamp_high,
        timestamp_low,
        captured_length,
        original_length,
    ) + data
    write_block(stream, EPB, body)


def merge(inputs: list[Path], output: Path, manifest_path: Path) -> dict[str, object]:
    if len(inputs) < 2:
        raise ValueError("at least two input PCAP files are required")
    if any(not path.is_file() for path in inputs):
        missing = [str(path) for path in inputs if not path.is_file()]
        raise FileNotFoundError("missing input: " + ", ".join(missing))

    output.parent.mkdir(parents=True, exist_ok=True)
    packet_count = 0
    first_timestamp = None
    last_timestamp = None
    source_stats = []
    link_type, snaplen = inspect_interface(inputs[0])
    with output.open("wb") as destination:
        write_section_header(destination)
        write_interface_description(destination, link_type, snaplen)
        for input_path in inputs:
            source_packet_count = 0
            source_first = None
            source_last = None
            source_link_type = None
            source_snaplen = None
            for packet in iter_enhanced_packets(input_path):
                timestamp_high, timestamp_low = packet[0], packet[1]
                timestamp_ticks = (timestamp_high << 32) | timestamp_low
                timestamp_seconds = timestamp_ticks / 1_000_000.0
                if last_timestamp is not None and timestamp_seconds < last_timestamp:
                    raise ValueError(
                        "input files are not in chronological order: "
                        f"{input_path} starts at {timestamp_seconds}, "
                        f"previous packet is {last_timestamp}"
                    )
                if source_first is None:
                    source_first = timestamp_seconds
                source_last = timestamp_seconds
                if first_timestamp is None:
                    first_timestamp = timestamp_seconds
                last_timestamp = timestamp_seconds
                write_packet(destination, packet)
                packet_count += 1
                source_packet_count += 1
            source_stats.append(
                {
                    "path": str(input_path),
                    "packet_count": source_packet_count,
                    "first_timestamp": source_first,
                    "last_timestamp": source_last,
                }
            )
            if source_packet_count == 0:
                raise ValueError(f"input contains no Enhanced Packet Blocks: {input_path}")

    manifest = {
        "output": str(output),
        "container": "pcapng",
        "extension_reason": "The current PCAP dialog filters .pcap; libpcap detects the pcapng container.",
        "inputs": [str(path) for path in inputs],
        "source_stats": source_stats,
        "packet_count": packet_count,
        "first_timestamp": first_timestamp,
        "last_timestamp": last_timestamp,
        "georeference": "D:/xwechat_files/wxid_1vufciky1ykg32_8fe8/msg/file/2026-09/georeference(1).json",
        "note": "This file preserves the dynamic packet timeline. It does not itself apply ENU/geographic transforms; the supplied georeference is recorded as metadata and is used by the generated slammap.",
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(merge(args.input, args.output, args.manifest), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
