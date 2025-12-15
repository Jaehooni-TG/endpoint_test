#!/usr/bin/env python3
"""Minimal Feetech STS3215 driver for SO-ARM101.

This module provides just enough functionality to:
  - open a Feetech serial bus using ``scservo_sdk``
  - send absolute position commands to each joint
  - read back present positions

It is intentionally much simpler than LeRobot's full motor stack and does not
depend on torch, datasets, accelerate, etc.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping

import serial  # type: ignore[import-not-found]

try:
    import scservo_sdk as scs  # type: ignore[import-not-found]
except Exception as exc:  # pragma: no cover - runtime guard
    raise RuntimeError(
        "The 'feetech-servo-sdk' package (providing scservo_sdk) is required to talk to "
        "SO-ARM101 motors. Install it with:\n"
        "  python3 -m pip install --user feetech-servo-sdk\n"
    ) from exc

logger = logging.getLogger(__name__)


@dataclass
class MotorCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


@dataclass
class MotorMapping:
    """Precomputed mapping parameters for a single motor."""

    center: float  # raw encoder value corresponding to 0deg
    counts_per_deg: float  # how many encoder counts per degree (symmetric)


class So101FeetechArm:
    """Very small helper around scservo_sdk for SO-ARM101.

    The mapping from joint angles[deg] to raw encoder counts is kept deliberately
    simple and relies only on the calibration JSON:
      - each motor uses its own [range_min, range_max] window
      - 0deg is mapped to the mid-point of that window
      - +/- max_abs_deg is mapped to the min/max of the window
    This is sufficient for teleop and can be tuned later if needed.
    """

    # STS3215 control table addresses (12‑bit position encoder)
    ADDR_TORQUE_ENABLE = 40
    ADDR_GOAL_POSITION = 42
    ADDR_PRESENT_POSITION = 56
    ADDR_LOCK = 55

    def __init__(
        self,
        port: str,
        calibration_file: Path,
        max_abs_deg: float = 120.0,
    ) -> None:
        self._port_name = port
        self._baudrate = 1_000_000
        self._protocol_version = 0
        self._max_abs_deg = float(max_abs_deg)

        self._calib: Dict[str, MotorCalibration] = self._load_calibration(calibration_file)
        # Fixed SO‑ARM101 motor ordering
        self._joint_to_motor = {
            "Rotation": "shoulder_pan",
            "Pitch": "shoulder_lift",
            "Elbow": "elbow_flex",
            "Wrist_Pitch": "wrist_flex",
            "Wrist_Roll": "wrist_roll",
            "Jaw": "gripper",
        }

        self._port_handler: scs.PortHandler | None = None
        self._packet_handler: scs.PacketHandler | None = None
        # Precomputed encoder↔angle mapping per motor (populated on demand)
        self._mapping: Dict[str, MotorMapping] = {}

    @staticmethod
    def _load_calibration(path: Path) -> Dict[str, MotorCalibration]:
        if not path.is_file():
            raise FileNotFoundError(f"Calibration file not found: {path}")
        data = json.loads(path.read_text())
        calib: Dict[str, MotorCalibration] = {}
        for name, entry in data.items():
            calib[name] = MotorCalibration(
                id=int(entry["id"]),
                drive_mode=int(entry.get("drive_mode", 0)),
                homing_offset=int(entry.get("homing_offset", 0)),
                range_min=int(entry["range_min"]),
                range_max=int(entry["range_max"]),
            )
        return calib

    # ------------------------------------------------------------------ #
    # Connection management
    # ------------------------------------------------------------------ #

    @property
    def is_connected(self) -> bool:
        return bool(self._port_handler and self._port_handler.is_open)

    def connect(self, enable_torque: bool = True) -> None:
        if self.is_connected:
            return

        self._port_handler = scs.PortHandler(self._port_name)
        self._packet_handler = scs.PacketHandler(self._protocol_version)

        if not self._port_handler.openPort():
            raise ConnectionError(f"Failed to open Feetech port '{self._port_name}'")
        if not self._port_handler.setBaudRate(self._baudrate):
            raise ConnectionError(
                f"Failed to set baudrate {self._baudrate} on port '{self._port_name}'"
            )

        # Optionally enable torque on all motors so they actually move when commanded.
        # In \"state-only\" / hand-guiding mode, callers can pass enable_torque=False
        # to leave the servos free while still allowing position reads.
        if enable_torque:
            for motor in self._calib.values():
                try:
                    # Torque_Enable = 1, Lock = 1
                    self._write1b(motor.id, self.ADDR_TORQUE_ENABLE, 1)
                    self._write1b(motor.id, self.ADDR_LOCK, 1)
                except Exception as exc:
                    logger.warning(
                        "Failed to enable torque on motor id=%d: %s", motor.id, exc
                    )

        logger.info(
            "Connected Feetech bus on %s (baud=%d)", self._port_name, self._baudrate
        )

    def disconnect(self, disable_torque: bool = True) -> None:
        if not self.is_connected:
            return
        assert self._port_handler is not None
        assert self._packet_handler is not None

        if disable_torque:
            for motor in self._calib.values():
                try:
                    self._write1b(motor.id, self.ADDR_TORQUE_ENABLE, 0)
                except Exception:
                    pass

        self._port_handler.closePort()
        logger.info("Disconnected Feetech bus on %s", self._port_name)

    # ------------------------------------------------------------------ #
    # Torque helpers
    # ------------------------------------------------------------------ #

    def disable_all_torque(self) -> None:
        """Explicitly disable torque on every motor (best-effort)."""
        if not self.is_connected:
            raise RuntimeError("Feetech arm is not connected")

        for motor in self._calib.values():
            try:
                self._write1b(motor.id, self.ADDR_TORQUE_ENABLE, 0)
                self._write1b(motor.id, self.ADDR_LOCK, 0)
            except Exception as exc:  # pragma: no cover - hardware-dependent
                logger.warning(
                    "Failed to disable torque on motor id=%d: %s",
                    motor.id,
                    exc,
                )

    # ------------------------------------------------------------------ #
    # Low-level helpers
    # ------------------------------------------------------------------ #

    def _write2b(self, motor_id: int, address: int, value: int) -> None:
        assert self._port_handler is not None and self._packet_handler is not None
        value = int(value)
        # scservo_sdk write*TxRx functions return (comm_result, error)
        comm_result, error = self._packet_handler.write2ByteTxRx(
            self._port_handler, motor_id, address, value
        )
        if comm_result != 0 or error != 0:
            raise RuntimeError(
                f"Feetech write2ByteTxRx failed (id={motor_id}, addr={address}, "
                f"value={value}, comm={comm_result}, error={error})"
            )

    def _write1b(self, motor_id: int, address: int, value: int) -> None:
        assert self._port_handler is not None and self._packet_handler is not None
        value = int(value)
        comm_result, error = self._packet_handler.write1ByteTxRx(
            self._port_handler, motor_id, address, value
        )
        if comm_result != 0 or error != 0:
            raise RuntimeError(
                f"Feetech write1ByteTxRx failed (id={motor_id}, addr={address}, "
                f"value={value}, comm={comm_result}, error={error})"
            )

    def _read2b(self, motor_id: int, address: int) -> int:
        assert self._port_handler is not None and self._packet_handler is not None
        value, comm_result, error = self._packet_handler.read2ByteTxRx(
            self._port_handler, motor_id, address
        )
        if comm_result != 0 or error != 0:
            raise RuntimeError(
                f"Feetech read2ByteTxRx failed (id={motor_id}, addr={address}, "
                f"comm={comm_result}, error={error})"
            )
        return int(value)

    # ------------------------------------------------------------------ #
    # Angle ↔ encoder conversion
    # ------------------------------------------------------------------ #

    def _get_mapping(self, motor_name: str) -> MotorMapping:
        """Compute or retrieve mapping parameters for a motor.

        The goal is to reproduce LeRobot's ``MotorNormMode.DEGREES`` mapping
        for Feetech motors, but without pulling in the whole stack.

        From LeRobot (see ``MotorsBus._normalize/_unnormalize`` and
        ``FeetechMotorsBus.model_resolution_table``):

          - Encoder resolution is 4096 counts per full turn.
          - ``range_min`` / ``range_max`` are limits in encoder counts
            (same units as ``Present_Position``).
          - Degrees are defined such that 0deg corresponds to the mid‑point
            ``mid = (range_min + range_max) / 2``.
          - Conversion is::

                deg  = (raw - mid) * 360 / (max_res - 1)
                raw  = mid + deg * (max_res - 1) / 360

        where ``max_res = 4096`` for STS3215.

        We keep the same convention here so that our joint angles match what
        LeRobot would report when using ``norm_mode=DEGREES``.
        """
        if motor_name in self._mapping:
            return self._mapping[motor_name]

        calib = self._calib[motor_name]
        raw_min = float(calib.range_min)
        raw_max = float(calib.range_max)
        if raw_max <= raw_min:
            # Fallback: use full 12‑bit span if calibration is invalid
            raw_min, raw_max = 0.0, 4095.0

        # LeRobot's convention: mid‑point of the (possibly trimmed) range is
        # the 0‑degree reference.
        center = (raw_min + raw_max) / 2.0
        # STS3215 resolution: 4096 counts / revolution.
        max_res = 4096.0
        # Counts per degree is independent of the particular min/max limits.
        # The limits only decide where 0deg sits (via ``center``).
        counts_per_deg = (max_res - 1.0) / 360.0

        mapping = MotorMapping(center=center, counts_per_deg=counts_per_deg)
        self._mapping[motor_name] = mapping
        return mapping

    def _deg_to_raw(self, motor_name: str, deg: float) -> int:
        """Map an angle in degrees to a raw encoder count for a motor."""
        calib = self._calib[motor_name]
        mapping = self._get_mapping(motor_name)

        # Clamp requested angle to configured range
        max_abs = self._max_abs_deg
        d = max(-max_abs, min(max_abs, float(deg)))

        raw_f = mapping.center + d * mapping.counts_per_deg
        raw_i = int(round(raw_f))
        return max(calib.range_min, min(calib.range_max, raw_i))

    def _raw_to_deg(self, motor_name: str, raw: int) -> float:
        calib = self._calib[motor_name]
        mapping = self._get_mapping(motor_name)
        raw_f = float(raw)
        # Clamp to calibrated interval to avoid huge angles on bad reads.
        raw_f = max(float(calib.range_min), min(float(calib.range_max), raw_f))
        return (raw_f - mapping.center) / mapping.counts_per_deg

    # ------------------------------------------------------------------ #
    # High-level API used by the ROS bridge
    # ------------------------------------------------------------------ #

    def send_action(self, joint_deg: Mapping[str, float]) -> None:
        """Command joints to target angles in degrees."""
        if not self.is_connected:
            raise RuntimeError("Feetech arm is not connected")

        for joint_name, deg in joint_deg.items():
            motor_name = self._joint_to_motor.get(joint_name)
            if motor_name is None or motor_name not in self._calib:
                continue
            motor_id = self._calib[motor_name].id
            raw = self._deg_to_raw(motor_name, deg)
            try:
                self._write2b(motor_id, self.ADDR_GOAL_POSITION, raw)
            except Exception as exc:
                logger.warning(
                    "Failed to send position to motor '%s' (id=%d): %s",
                    motor_name,
                    motor_id,
                    exc,
                )

    def get_observation(self) -> Dict[str, float]:
        """Read present joint angles in degrees."""
        if not self.is_connected:
            raise RuntimeError("Feetech arm is not connected")

        joint_deg: Dict[str, float] = {}
        for joint_name, motor_name in self._joint_to_motor.items():
            if motor_name not in self._calib:
                continue
            motor_id = self._calib[motor_name].id
            try:
                raw = self._read2b(motor_id, self.ADDR_PRESENT_POSITION)
            except Exception as exc:
                logger.warning(
                    "Failed to read position from motor '%s' (id=%d): %s",
                    motor_name,
                    motor_id,
                    exc,
                )
                continue
            joint_deg[joint_name] = self._raw_to_deg(motor_name, raw)

        return joint_deg
