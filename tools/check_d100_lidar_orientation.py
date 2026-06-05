#!/usr/bin/env python3
"""Validate D100 chin/tail LiDAR X-axis directions in URDF files."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


URDF_FILES = [
    Path("zyurdf-20260522-1./urdf/zyurdf-20260522-1.urdf"),
    Path("src/descriptions/GON_Future/d100_description/urdf/robot.urdf"),
    Path("install/d100_description/share/d100_description/urdf/robot.urdf"),
]

EXPECTED_Z_SIGN = {
    "radar_f_joint": -1.0,
    "radar_r_joint": -1.0,
}


def rotation_x_axis_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rz_ry_rx = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    # Match the X-axis direction observed in robot_state_publisher /tf_static.
    return (rz_ry_rx[0][0], rz_ry_rx[0][1], rz_ry_rx[0][2])


def joint_rpy(root: ET.Element, joint_name: str) -> tuple[float, float, float]:
    for joint in root.findall("joint"):
        if joint.get("name") != joint_name:
            continue
        origin = joint.find("origin")
        if origin is None:
            raise ValueError(f"{joint_name} has no origin element")
        rpy_text = origin.get("rpy")
        if rpy_text is None:
            raise ValueError(f"{joint_name} origin has no rpy attribute")
        values = tuple(float(part) for part in rpy_text.split())
        if len(values) != 3:
            raise ValueError(f"{joint_name} rpy should have 3 numbers, got {rpy_text!r}")
        return values
    raise ValueError(f"{joint_name} not found")


def check_file(path: Path) -> list[str]:
    root = ET.parse(path).getroot()
    errors = []
    for joint_name, expected_sign in EXPECTED_Z_SIGN.items():
        rpy = joint_rpy(root, joint_name)
        x_axis = rotation_x_axis_from_rpy(*rpy)
        z_value = x_axis[2]
        direction = "up" if z_value > 0 else "down"
        print(
            f"{path}: {joint_name} rpy={rpy} x_axis_z={z_value:.6f} ({direction})"
        )
        if expected_sign > 0 and z_value <= 0:
            errors.append(f"{path}: {joint_name} X axis should point upward")
        if expected_sign < 0 and z_value >= 0:
            errors.append(f"{path}: {joint_name} X axis should point downward")
    return errors


def main() -> int:
    all_errors = []
    for path in URDF_FILES:
        all_errors.extend(check_file(path))
    if all_errors:
        print("\nFAIL:")
        for error in all_errors:
            print(f"- {error}")
        return 1
    print("\nPASS: chin and tail X axes point downward in all checked URDFs.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
