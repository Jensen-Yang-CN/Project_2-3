#!/usr/bin/env python3
"""Convert the supplied binary ENU PCD into the application's .slammap format.

The source PCD is already in the local ENU frame described by the supplied
georeference JSON.  The JSON's map_to_enu matrix is therefore recorded as
metadata but deliberately not applied a second time.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np


MAGIC = b"USVSLM2"
FORMAT_MAJOR = 2
FORMAT_MINOR = 1
COORDINATE_FRAME_ENU = 1

SOURCE_DTYPE = np.dtype(
    [
        ("source_id", "<f4"),
        ("intensity", "<f4"),
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("padding", "u1", (4,)),
    ]
)
XYZ_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
TARGET_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "u1")]
)


def read_pcd_header(path: Path) -> tuple[dict[str, str], int]:
    header: dict[str, str] = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header ended before DATA")
            text = line.decode("ascii", errors="strict").strip()
            if text and not text.startswith("#"):
                parts = text.split(maxsplit=1)
                header[parts[0].upper()] = parts[1] if len(parts) > 1 else ""
            if text.upper().startswith("DATA"):
                return header, stream.tell()


def parse_pcd(path: Path) -> tuple[dict[str, str], int, int, np.dtype, bool]:
    header, data_offset = read_pcd_header(path)
    if header.get("DATA", "").lower() != "binary":
        raise ValueError("only binary PCD is supported")
    fields = header.get("FIELDS", "").split()
    sizes = [int(value) for value in header.get("SIZE", "").split()]
    counts = [int(value) for value in header.get("COUNT", "").split()]
    if fields == ["source_id", "intensity", "x", "y", "z", "_"]:
        if sizes != [4, 4, 4, 4, 4, 1] or counts != [1, 1, 1, 1, 1, 4]:
            raise ValueError("unexpected extended PCD field layout")
        source_dtype = SOURCE_DTYPE
        has_intensity = True
    elif fields == ["x", "y", "z"]:
        if sizes != [4, 4, 4] or counts != [1, 1, 1]:
            raise ValueError("unexpected XYZ PCD field layout")
        source_dtype = XYZ_DTYPE
        has_intensity = False
    else:
        raise ValueError(f"unexpected PCD fields: {fields}")
    points = int(header["POINTS"])
    record_bytes = sum(size * count for size, count in zip(sizes, counts))
    if record_bytes != source_dtype.itemsize:
        raise ValueError("PCD record size does not match the converter")
    expected_size = data_offset + points * record_bytes
    actual_size = path.stat().st_size
    if expected_size != actual_size:
        raise ValueError(
            f"PCD size mismatch: expected {expected_size}, got {actual_size}"
        )
    return header, data_offset, points, source_dtype, has_intensity


def finite_source_chunk(chunk: np.ndarray) -> None:
    coordinates = np.column_stack((chunk["x"], chunk["y"], chunk["z"]))
    if not np.isfinite(coordinates).all():
        raise ValueError("source PCD contains non-finite point values")
    if "intensity" in chunk.dtype.names and not np.isfinite(chunk["intensity"]).all():
        raise ValueError("source PCD contains non-finite intensity values")


def write_slammap(
    pcd_path: Path,
    georef_path: Path,
    output_path: Path,
    chunks: int,
    read_batch: int,
    dataset_root: Path | None = None,
) -> dict[str, object]:
    header, data_offset, point_count, source_dtype, has_intensity = parse_pcd(pcd_path)
    if chunks <= 0:
        raise ValueError("chunks must be positive")
    if chunks > 10000:
        raise ValueError("chunks exceeds the application's keyframe limit")

    georef = json.loads(georef_path.read_text(encoding="utf-8"))
    required = (
        "map_anchor_timestamp",
        "origin_latitude_deg",
        "origin_longitude_deg",
        "origin_altitude_m",
    )
    missing = [key for key in required if key not in georef]
    if missing:
        raise ValueError(f"georeference JSON is missing: {missing}")

    anchor_timestamp = float(georef["map_anchor_timestamp"])
    anchor_latitude = float(georef["origin_latitude_deg"])
    anchor_longitude = float(georef["origin_longitude_deg"])
    anchor_altitude = float(georef["origin_altitude_m"])
    if not all(
        math.isfinite(value)
        for value in (
            anchor_timestamp,
            anchor_latitude,
            anchor_longitude,
            anchor_altitude,
        )
    ):
        raise ValueError("georeference anchor contains non-finite values")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    points_per_chunk = math.ceil(point_count / chunks)
    keyframe_counts: list[int] = []
    target_dtype = TARGET_DTYPE
    with pcd_path.open("rb") as source, output_path.open("wb") as target:
        target.write(MAGIC)
        target.write(
            struct.pack(
                "<HHBBddddI",
                FORMAT_MAJOR,
                FORMAT_MINOR,
                COORDINATE_FRAME_ENU,
                1,
                anchor_timestamp,
                anchor_latitude,
                anchor_longitude,
                anchor_altitude,
                chunks,
            )
        )
        source.seek(data_offset)
        remaining = point_count
        for keyframe_id in range(chunks):
            count = min(points_per_chunk, remaining)
            if count <= 0:
                break
            keyframe_counts.append(count)
            # Synthetic chunks are map-space data, not time-ordered poses.
            target.write(
                struct.pack(
                    "<QddddddddI",
                    keyframe_id,
                    anchor_timestamp,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    count,
                )
            )
            # No per-keyframe GNSS/INS pose exists in the merged PCD.
            target.write(struct.pack("<Bddddddddd", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))

            left = count
            while left:
                batch = min(left, read_batch)
                source_chunk = np.fromfile(source, dtype=source_dtype, count=batch)
                if source_chunk.size != batch:
                    raise ValueError("PCD ended before the declared point count")
                finite_source_chunk(source_chunk)
                converted = np.empty(batch, dtype=target_dtype)
                converted["x"] = source_chunk["x"]
                converted["y"] = source_chunk["y"]
                converted["z"] = source_chunk["z"]
                if has_intensity:
                    converted["intensity"] = np.clip(
                        np.rint(source_chunk["intensity"]), 0, 255
                    ).astype(np.uint8)
                else:
                    converted["intensity"] = 0
                converted.tofile(target)
                left -= batch
            remaining -= count

    if remaining != 0 or len(keyframe_counts) != chunks:
        raise ValueError("failed to write all PCD points")

    pcap_inventory: list[dict[str, object]] = []
    if dataset_root is not None:
        for batch_directory in sorted(dataset_root.iterdir()):
            if not batch_directory.is_dir():
                continue
            if "检测" not in batch_directory.name:
                continue
            for pcap_path in sorted(batch_directory.glob("*.pcap")):
                pcap_inventory.append(
                    {
                        "batch_directory": batch_directory.name,
                        "relative_path": str(pcap_path.relative_to(dataset_root)),
                        "size_bytes": pcap_path.stat().st_size,
                    }
                )

    return {
        "source_pcd": str(pcd_path),
        "georeference_path": str(georef_path),
        "dataset_root": str(dataset_root) if dataset_root is not None else None,
        "pcap_source_inventory": pcap_inventory,
        "pcap_selection_note": (
            "The merged PCD has no per-point timestamp or source identifier "
            "(source_id is zero for every point), so the inventory is recorded "
            "for traceability only; no PCAP boundary is inferred."
        ),
        "output_slammap": str(output_path),
        "source_header": header,
        "source_points": point_count,
        "keyframe_count": len(keyframe_counts),
        "keyframe_point_counts": keyframe_counts,
        "coordinate_frame": "WGS84 local tangent plane ENU",
        "units": "metres",
        "map_to_enu_applied": False,
        "map_to_enu_reason": (
            "The PCD filename and coordinate-conversion note identify the PCD "
            "as ENU already; applying this matrix again would double-transform it."
        ),
        "keyframe_split_reason": (
            "The application limits one keyframe to 5,000,000 points. Chunks "
            "are capacity chunks and do not represent PCAP boundaries or time."
        ),
        "anchor": {
            "timestamp": anchor_timestamp,
            "latitude_deg": anchor_latitude,
            "longitude_deg": anchor_longitude,
            "altitude_m": anchor_altitude,
        },
        "georeference": georef,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--georeference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path)
    parser.add_argument("--chunks", type=int, default=2)
    parser.add_argument("--read-batch", type=int, default=250_000)
    args = parser.parse_args()

    manifest = write_slammap(
        args.pcd,
        args.georeference,
        args.output,
        args.chunks,
        args.read_batch,
        args.dataset_root,
    )
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
