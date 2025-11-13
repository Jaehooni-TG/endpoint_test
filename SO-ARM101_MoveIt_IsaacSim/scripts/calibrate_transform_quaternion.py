#!/usr/bin/env python3
"""
Compute transform_quaternion for websocket_pose_bridge_node.

Given a pair of orientations of the same physical pose expressed in two frames:
  - q_web  = (w, x, y, z) from the web client
  - q_base = (w, x, y, z) desired in the ROS base/planning frame

The transform quaternion T that maps web -> base is:
  T = q_base ⊗ conj(q_web)

You can then pass T to the bridge as the 'transform_quaternion' parameter
in [w, x, y, z] order.

Usage examples:
  python3 scripts/calibrate_transform_quaternion.py --qweb  1 0 0 0 --qbase 0.7071 0.7071 0 0
  python3 scripts/calibrate_transform_quaternion.py --qweb  "1,0,0,0" --qbase "0.7071,0.7071,0,0"

This prints T in both [w,x,y,z] and [x,y,z,w] forms and a ready-to-copy
ROS 2 CLI snippet.
"""

from __future__ import annotations

import argparse
from typing import Iterable, Tuple

from tf_transformations import quaternion_conjugate, quaternion_multiply


def parse_quat(arg: str | Iterable[float]) -> Tuple[float, float, float, float]:
    if isinstance(arg, str):
        parts = [p.strip() for p in arg.replace("[", "").replace("]", "").split(",")]
        if len(parts) == 1:
            # space-separated
            parts = arg.split()
        vals = [float(v) for v in parts if v != ""]
    else:
        vals = [float(v) for v in arg]
    if len(vals) != 4:
        raise argparse.ArgumentTypeError("Quaternion must have exactly 4 values (w x y z).")
    return (vals[0], vals[1], vals[2], vals[3])


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate transform_quaternion (web -> base)")
    parser.add_argument("--qweb", required=True, type=parse_quat, help="Web quaternion (w x y z)")
    parser.add_argument("--qbase", required=True, type=parse_quat, help="Base quaternion (w x y z)")
    args = parser.parse_args()

    q_web_wxyz = args.qweb
    q_base_wxyz = args.qbase

    # tf_transformations uses (x, y, z, w) ordering
    q_web_xyzw = (q_web_wxyz[1], q_web_wxyz[2], q_web_wxyz[3], q_web_wxyz[0])
    q_base_xyzw = (q_base_wxyz[1], q_base_wxyz[2], q_base_wxyz[3], q_base_wxyz[0])

    # T = q_base ⊗ conj(q_web)
    T_xyzw = quaternion_multiply(q_base_xyzw, quaternion_conjugate(q_web_xyzw))
    T_wxyz = (T_xyzw[3], T_xyzw[0], T_xyzw[1], T_xyzw[2])

    print("Computed transform_quaternion:")
    print(f"  wxyz: {T_wxyz}")
    print(f"  xyzw: {T_xyzw}")
    print()
    print("ROS 2 param snippet:")
    print("  -p transform_quaternion:=\"[%.6f, %.6f, %.6f, %.6f]\"" % T_wxyz)


if __name__ == "__main__":
    main()

