#!/usr/bin/env python3
"""Verify that Gazebo soil-column poses follow excavation height updates."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


CELL_SIZE_M = 0.05
PILE_CENTER_M = 7.0
PILE_HEIGHT_M = 1.8
ANGLE_OF_REPOSE_DEG = 34.0
COLUMN_HEIGHT_M = 1.8


def parse_poses(text: str) -> dict[int, float]:
    poses: dict[int, float] = {}
    current_index: int | None = None
    for line in text.splitlines():
        name_match = re.search(r"Name: soil_column_(\d+)", line)
        if name_match:
            current_index = int(name_match.group(1))
            continue
        if current_index is None:
            continue
        pose_match = re.match(
            r"\s*\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]",
            line,
        )
        if pose_match:
            poses[current_index] = float(pose_match.group(3))
            current_index = None
    return poses


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("pose_log", type=Path)
    parser.add_argument("expectation_log", type=Path)
    args = parser.parse_args()

    poses = parse_poses(args.pose_log.read_text(encoding="utf-8"))
    expectation = args.expectation_log.read_text(encoding="utf-8").split()
    if len(expectation) != 2:
        raise RuntimeError("invalid soil proxy expectation file")
    index = int(expectation[0])
    expected_z = float(expectation[1])
    if index not in poses:
        raise RuntimeError(f"Gazebo did not return soil column {index}")
    error = abs(poses[index] - expected_z)
    if error > 0.002:
        raise RuntimeError(
            f"soil column {index} differs from TerrainState by {error:.6f} m"
        )

    print(
        "PASS soil visual proxy update: "
        f"column={index} expected_z={expected_z:.6f}m "
        f"actual_z={poses[index]:.6f}m error={error:.6f}m"
    )


if __name__ == "__main__":
    main()
