"""
motion_utils.py – HRC2026 Task 1 Role 3
========================================
Core utilities for pick-place motion primitive:
  - Pose / Waypoint dataclasses
  - check_preconditions()          : validate ActionPlan before execution
  - generate_pick_place_waypoints(): build full waypoint sequence
  - make_pose()                    : construct Pose from position + yaw/quaternion
  - apply_retry_adjustment()       : adjust params for retry attempt
  - is_within_workspace()          : workspace bounds check

Pure Python – no Isaac runtime required. Runnable in dry-run mode.
All uncertain APIs marked TODO(BASELINE_VERIFY).
"""

from __future__ import annotations

import logging
import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

try:
    import yaml
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False


# ---------------------------------------------------------------------------
# Default params
# ---------------------------------------------------------------------------
DEFAULT_PARAMS: dict = {
    # Waypoint heights (m)
    "pre_grasp_height_m":           0.12,
    "lift_height_m":                0.17,   # từ config: lift_height: 0.17
    "pre_place_height_m":           0.17,
    "grasp_z_offset_m":             0.005,
    "place_z_offset_m":             0.05,
    "retreat_height_m":             0.12,

    # Speed scales
    "speed_scale_nominal":          1.0,
    "speed_scale_approach":         0.6,
    "speed_scale_transfer":         0.8,
    "speed_scale_place":            0.5,

    # Gripper (m, steps)
    "open_width_m":                 -0.0215,
    "close_width_m":                0.01,
    "min_grasp_width_m":            0.01,
    "max_grasp_width_m":            0.08,
    "open_wait_steps":              10,
    "close_wait_steps":             15,

    # Workspace limits – robot tại [0.7,-0.2,0.9], scatter x:[0.50,0.80] y:[0.10,0.30] z~1.04
    "workspace_x":                  [0.45, 0.85],
    "workspace_y":                  [0.05, 0.35],
    "workspace_z":                  [0.90, 1.20],

    # Retry
    "max_retry":                    1,
    "retry_pre_grasp_height_add_m": 0.03,
    "retry_close_wait_add_steps":   5,
    "retry_lift_height_add_m":      0.05,
    "retry_speed_multiplier":       0.5,

    # Timeout per step (s)
    "timeout_pre_grasp_s":          3.0,
    "timeout_grasp_s":              2.0,
    "timeout_lift_s":               2.0,
    "timeout_transfer_s":           4.0,
    "timeout_place_s":              2.0,
    "timeout_retreat_s":            2.0,
    "primitive_timeout_s":          30.0,

    # Min confidence
    "min_confidence":               0.70,
}



# ---------------------------------------------------------------------------
# YAML loader
# ---------------------------------------------------------------------------

def load_params(yaml_path: str) -> dict:
    """Load task YAML config and merge with DEFAULT_PARAMS."""
    params = dict(DEFAULT_PARAMS)
    if not os.path.isfile(yaml_path):
        logger.info(f"[motion_utils] {yaml_path} not found – using DEFAULT_PARAMS.")
        return params
    if not _HAS_YAML:
        logger.warning("[motion_utils] PyYAML not installed; using DEFAULT_PARAMS.")
        return params
    with open(yaml_path, "r") as f:
        loaded = yaml.safe_load(f) or {}
    params.update(loaded)
    logger.info(f"[motion_utils] Loaded params from {yaml_path}")
    return params


# ---------------------------------------------------------------------------
# Pose / Waypoint dataclasses
# ---------------------------------------------------------------------------

@dataclass
class Pose:
    """6-DOF pose: position [x,y,z] + quaternion [qx,qy,qz,qw]."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=np.float64)

    def quaternion(self) -> np.ndarray:
        return np.array([self.qx, self.qy, self.qz, self.qw], dtype=np.float64)

    def as_xyzrpy(self) -> List[float]:
        """Convert to [x, y, z, roll, pitch, yaw] for IK interface."""
        roll, pitch, yaw = _quat_to_rpy(self.qx, self.qy, self.qz, self.qw)
        return [self.x, self.y, self.z, roll, pitch, yaw]


@dataclass
class Waypoint:
    """Single waypoint in the pick-place sequence."""
    step_name: str
    pose: Pose
    speed_scale: float = 1.0
    timestamp: float = field(default_factory=time.time)

    def to_csv_row(self) -> dict:
        return {
            "step_name": self.step_name,
            "x": self.pose.x,
            "y": self.pose.y,
            "z": self.pose.z,
            "qx": self.pose.qx,
            "qy": self.pose.qy,
            "qz": self.pose.qz,
            "qw": self.pose.qw,
            "speed_scale": self.speed_scale,
            "timestamp": self.timestamp,
        }


@dataclass
class PreconditionResult:
    passed: bool
    failure_reason: Optional[str] = None
    details: str = ""


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Quaternion → roll-pitch-yaw (ZYX convention)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Build quaternion from yaw-only rotation (z-axis)."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def make_pose(
    position: List[float],
    quaternion_xyzw: Optional[List[float]] = None,
    yaw_rad: float = 0.0,
) -> Pose:
    """Construct a Pose from position and optional quaternion or yaw."""
    x, y, z = float(position[0]), float(position[1]), float(position[2])
    if quaternion_xyzw is not None:
        qx, qy, qz, qw = [float(v) for v in quaternion_xyzw]
    else:
        qx, qy, qz, qw = _yaw_to_quat(yaw_rad)
    return Pose(x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw)


# ---------------------------------------------------------------------------
# Workspace check
# ---------------------------------------------------------------------------

def is_within_workspace(position: list, params: dict) -> bool:
    """Check if [x, y, z] is within workspace bounds."""
    if not position or len(position) != 3:
        return False
    x, y, z = position
    wx = params.get("workspace_x", [0.25, 0.70])
    wy = params.get("workspace_y", [-0.35, 0.35])
    wz = params.get("workspace_z", [0.70, 1.15])
    return wx[0] <= x <= wx[1] and wy[0] <= y <= wy[1] and wz[0] <= z <= wz[1]


# ---------------------------------------------------------------------------
# Precondition checks
# ---------------------------------------------------------------------------

def check_preconditions(
    action_plan: dict,
    params: Optional[dict] = None,
    gripper_state: str = "open",
    ik_solver: Any = None,
    dry_run: bool = False,
) -> PreconditionResult:
    """Validate all preconditions before executing pick-place.

    Args:
        action_plan: Dict with object_pose_base, bin_pose_base, grasp_hint, confidence.
        params: Motion params dict (uses DEFAULT_PARAMS if None).
        gripper_state: Current gripper state ("open" / "closed").
        ik_solver: Optional IK solver for reachability check.
        dry_run: If True, skip IK reachability check.

    Returns:
        PreconditionResult with passed flag and failure_reason.
    """
    if params is None:
        params = DEFAULT_PARAMS
    ts = time.time()

    required_keys = ["object_pose_base", "bin_pose_base", "grasp_hint", "confidence"]
    for key in required_keys:
        if key not in action_plan:
            logger.error(f"[{ts:.3f}] Precondition FAIL: missing key '{key}'")
            return PreconditionResult(False, "BAD_ACTION_PLAN", f"Missing key: {key}")

    confidence = float(action_plan.get("confidence", 0.0))
    min_conf = float(params.get("min_confidence", 0.70))
    if confidence < min_conf:
        msg = f"confidence={confidence:.3f} < threshold={min_conf}"
        logger.error(f"[{ts:.3f}] Precondition FAIL: {msg}")
        return PreconditionResult(False, "BAD_ACTION_PLAN", msg)

    obj_pos = action_plan["object_pose_base"].get("position_m", [])
    if len(obj_pos) != 3:
        return PreconditionResult(False, "BAD_ACTION_PLAN", "object position_m must be length 3")
    try:
        if not all(isinstance(v, (int, float)) and -1e6 < v < 1e6 for v in obj_pos):
            return PreconditionResult(False, "BAD_ACTION_PLAN", "object position not finite")
    except Exception:
        return PreconditionResult(False, "BAD_ACTION_PLAN", "object position not finite")

    if not is_within_workspace(obj_pos, params):
        msg = f"object position {obj_pos} outside workspace"
        logger.error(f"[{ts:.3f}] Precondition FAIL: {msg}")
        return PreconditionResult(False, "OBJECT_OUT_OF_WORKSPACE", msg)

    bin_pos = action_plan["bin_pose_base"].get("position_m", [])
    if len(bin_pos) != 3:
        return PreconditionResult(False, "BAD_ACTION_PLAN", "bin position_m must be length 3")
    if not is_within_workspace(bin_pos, params):
        msg = f"bin position {bin_pos} outside workspace"
        logger.error(f"[{ts:.3f}] Precondition FAIL: {msg}")
        return PreconditionResult(False, "BIN_OUT_OF_WORKSPACE", msg)

    grasp_width = float(action_plan["grasp_hint"].get("grasp_width_m", 0.0))
    if grasp_width <= 0:
        return PreconditionResult(False, "BAD_ACTION_PLAN", f"invalid grasp_width_m={grasp_width}")

    if gripper_state != "open":
        msg = f"gripper_state='{gripper_state}', expected 'open'"
        logger.error(f"[{ts:.3f}] Precondition FAIL: {msg}")
        return PreconditionResult(False, "GRIPPER_OPEN_FAIL", msg)

    # IK reachability (skip in dry_run or if solver unavailable)
    if not dry_run and ik_solver is not None:
        pre_grasp_h = float(params.get("pre_grasp_height_m", 0.12))
        pre_grasp_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + pre_grasp_h]
        yaw = float(action_plan["grasp_hint"].get("yaw_rad", 0.0))
        pre_grasp_pose = make_pose(pre_grasp_pos, yaw_rad=yaw)
        # TODO(BASELINE_VERIFY): DualArmIK does not expose standalone reachability check.
        # Using solve_dual_arm as proxy.
        try:
            xyzrpy = pre_grasp_pose.as_xyzrpy()
            result = ik_solver.solve_dual_arm(right_target_xyzrpy=xyzrpy)
            if not result.get("right_success", False):
                msg = f"IK not reachable for pre_grasp at {pre_grasp_pos}"
                logger.error(f"[{ts:.3f}] Precondition FAIL: {msg}")
                return PreconditionResult(False, "IK_FAIL_PRE_GRASP", msg)
        except Exception as exc:
            logger.warning(f"[{ts:.3f}] IK reachability check exception: {exc} – skipping")
    else:
        logger.info(f"[{ts:.3f}] IK reachability check skipped (dry_run={dry_run})")

    logger.info(f"[{ts:.3f}] All preconditions PASSED")
    return PreconditionResult(True)


# ---------------------------------------------------------------------------
# Retry adjustment
# ---------------------------------------------------------------------------

def apply_retry_adjustment(params: dict, failure_reason: str) -> dict:
    """Return a copy of params with retry adjustments applied.

    Strategy (Module 6 §retry):
      - IK fail / collision on descend → raise pre_grasp height
      - Grasp not confirmed / drop during lift → more close_wait, higher lift
      - Collision during transfer → raise lift and pre_place height
      - Place fail → raise place z offset
    """
    p = dict(params)
    spd_mult = float(p.get("retry_speed_multiplier", 0.5))

    if failure_reason in ("IK_FAIL_GRASP", "COLLISION_ON_DESCEND"):
        p["pre_grasp_height_m"] = (
            params.get("pre_grasp_height_m", 0.12)
            + params.get("retry_pre_grasp_height_add_m", 0.03)
        )
        p["grasp_z_offset_m"] = params.get("grasp_z_offset_m", 0.005) + 0.01

    elif failure_reason in ("GRASP_NOT_CONFIRMED", "DROP_DURING_LIFT"):
        p["close_wait_steps"] = (
            params.get("close_wait_steps", 15)
            + params.get("retry_close_wait_add_steps", 5)
        )
        p["lift_height_m"] = params.get("lift_height_m", 0.15) + 0.02

    elif failure_reason == "COLLISION_TRANSFER":
        add = params.get("retry_lift_height_add_m", 0.05)
        p["lift_height_m"] = params.get("lift_height_m", 0.15) + add
        p["pre_place_height_m"] = params.get("pre_place_height_m", 0.15) + add

    elif failure_reason == "PLACE_FAIL":
        p["place_z_offset_m"] = params.get("place_z_offset_m", 0.05) + 0.01

    # Always halve speed on retry
    for key in ("speed_scale_nominal", "speed_scale_approach", "speed_scale_transfer", "speed_scale_place"):
        if key in p:
            p[key] = float(p[key]) * spd_mult

    return p


# ---------------------------------------------------------------------------
# Waypoint generation
# ---------------------------------------------------------------------------

def generate_pick_place_waypoints(
    action_plan: dict,
    params: Optional[dict] = None,
    attempt: int = 0,
) -> List[Waypoint]:
    """Generate the full pick-place waypoint sequence.

    Args:
        action_plan: Validated ActionPlan dict.
        params: Motion params (uses DEFAULT_PARAMS if None).
        attempt: 0 = nominal, 1 = retry (adjusted params should be passed in).

    Returns:
        Ordered list of Waypoints:
        pre_grasp → grasp → lift → pre_place → place → retreat
    """
    if params is None:
        params = DEFAULT_PARAMS

    obj_pos = action_plan["object_pose_base"]["position_m"]
    bin_pos = action_plan["bin_pose_base"]["position_m"]
    yaw = float(action_plan["grasp_hint"].get("yaw_rad", 0.0))

    pre_grasp_h = float(params.get("pre_grasp_height_m", 0.12))
    grasp_z_off = float(params.get("grasp_z_offset_m", 0.005))
    lift_h = float(params.get("lift_height_m", 0.15))
    place_z_off = float(params.get("place_z_offset_m", 0.05))
    retreat_h = float(params.get("retreat_height_m", 0.12))

    spd_nominal = float(params.get("speed_scale_nominal", 1.0))
    spd_approach = float(params.get("speed_scale_approach", 0.6))
    spd_transfer = float(params.get("speed_scale_transfer", 0.8))
    spd_place = float(params.get("speed_scale_place", 0.5))

    qx, qy, qz, qw = _yaw_to_quat(yaw)
    ts_base = time.time()
    waypoints: List[Waypoint] = []

    # 1. pre_grasp – above object
    waypoints.append(Waypoint(
        step_name="pre_grasp",
        pose=Pose(x=obj_pos[0], y=obj_pos[1], z=obj_pos[2] + pre_grasp_h,
                  qx=qx, qy=qy, qz=qz, qw=qw),
        speed_scale=spd_nominal,
        timestamp=ts_base,
    ))

    # 2. grasp – at object centroid
    waypoints.append(Waypoint(
        step_name="grasp",
        pose=Pose(x=obj_pos[0], y=obj_pos[1], z=obj_pos[2] + grasp_z_off,
                  qx=qx, qy=qy, qz=qz, qw=qw),
        speed_scale=spd_approach,
        timestamp=ts_base,
    ))

    # 3. lift – straight up
    lift_z = obj_pos[2] + grasp_z_off + lift_h
    waypoints.append(Waypoint(
        step_name="lift",
        pose=Pose(x=obj_pos[0], y=obj_pos[1], z=lift_z,
                  qx=qx, qy=qy, qz=qz, qw=qw),
        speed_scale=0.7 * (float(params.get("retry_speed_multiplier", 1.0)) if attempt > 0 else 1.0),
        timestamp=ts_base,
    ))

    # 4. pre_place – above bin at lift height
    waypoints.append(Waypoint(
        step_name="pre_place",
        pose=Pose(x=bin_pos[0], y=bin_pos[1], z=lift_z,
                  qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        speed_scale=spd_transfer,
        timestamp=ts_base,
    ))

    # 5. place – descend to bin
    waypoints.append(Waypoint(
        step_name="place",
        pose=Pose(x=bin_pos[0], y=bin_pos[1], z=bin_pos[2] + place_z_off,
                  qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        speed_scale=spd_place,
        timestamp=ts_base,
    ))

    # 6. retreat – lift from bin
    waypoints.append(Waypoint(
        step_name="retreat",
        pose=Pose(x=bin_pos[0], y=bin_pos[1], z=bin_pos[2] + place_z_off + retreat_h,
                  qx=0.0, qy=0.0, qz=0.0, qw=1.0),
        speed_scale=spd_nominal,
        timestamp=ts_base,
    ))

    logger.info(
        f"[motion_utils] Generated {len(waypoints)} waypoints "
        f"(attempt={attempt}, pre_grasp_h={pre_grasp_h:.3f}m)"
    )
    return waypoints

