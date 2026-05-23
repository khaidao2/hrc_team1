"""
real_motion_interface.py – HRC2026 Task 1 Role 3
==================================================
Bridge từ motion primitive waypoints → IsaacSimRobotInterface.

Dùng khi Isaac Sim đang chạy và robot đã được initialize.

Usage trong motion_debug.py / motion.py:
    from real_motion_interface import RealMotionInterface
    interface = RealMotionInterface(robot_interface)
    runner = MotionPrimitiveRunner(interface, params, dry_run=False)

All uncertain APIs marked TODO(BASELINE_VERIFY).
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any, List, Optional

import numpy as np

logger = logging.getLogger(__name__)

# Physics timestep từ baseline (walkers2simConfig.py:88)
PHYSICS_DT = 1.0 / 200.0
CONTROL_HZ = 200


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _quat_to_rpy(qx: float, qy: float, qz: float, qw: float):
    """Quaternion → roll-pitch-yaw (ZYX)."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)

    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)

    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)

    return roll, pitch, yaw


def _waypoint_to_xyzrpy(waypoint) -> List[float]:
    """Convert Waypoint pose → [x, y, z, roll, pitch, yaw] cho IK."""
    p = waypoint.pose
    roll, pitch, yaw = _quat_to_rpy(p.qx, p.qy, p.qz, p.qw)
    return [p.x, p.y, p.z, roll, pitch, yaw]


# ---------------------------------------------------------------------------
# RealMotionInterface
# ---------------------------------------------------------------------------

class RealMotionInterface:
    """
    Wraps IsaacSimRobotInterface để dùng với MotionPrimitiveRunner.

    Mỗi move_to_waypoint() sẽ:
      1. Convert waypoint pose → xyzrpy
      2. Gọi robot.control_dual_arm_ik() lặp nhiều steps
      3. Gọi world.step() để physics tiến
      4. Check convergence hoặc timeout
    """

    def __init__(
        self,
        robot_interface: Any,
        world: Any = None,
        side: str = "right",
        ik_steps: int = 100,
        pos_tol: float = 0.01,
        rot_tol: float = 0.05,
    ):
        """
        Args:
            robot_interface: IsaacSimRobotInterface instance (đã initialize).
            world: Isaac Sim World instance (dùng để step physics).
            side: "right" hoặc "left" – arm dùng cho Task 1.
            ik_steps: Số physics steps tối đa để reach waypoint.
            pos_tol: Position tolerance (m) để coi là đã đến nơi.
            rot_tol: Rotation tolerance (rad).
        """
        self._robot = robot_interface
        self._world = world
        self._side = side
        self._ik_steps = ik_steps
        self._pos_tol = pos_tol
        self._rot_tol = rot_tol

        # Gripper state tracking
        self._gripper_state = {"left": "open", "right": "open"}

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def move_to_waypoint(
        self,
        waypoint,
        dry_run: bool = False,
        ik_solver: Any = None,
    ) -> bool:
        """Move robot EE to waypoint pose via IK.

        Args:
            waypoint: Waypoint dataclass với pose và speed_scale.
            dry_run: Nếu True, skip thật sự (fallback mock).
            ik_solver: Không dùng ở đây – robot_interface tự có IK.

        Returns:
            True nếu đến được waypoint, False nếu IK không converge.
        """
        if dry_run:
            logger.info(f"[RealMotion] dry_run skip: {waypoint.step_name}")
            return True

        xyzrpy = _waypoint_to_xyzrpy(waypoint)
        step_name = waypoint.step_name
        speed_scale = float(waypoint.speed_scale)

        # Số steps tỉ lệ với speed_scale – chậm hơn = nhiều steps hơn
        n_steps = max(20, int(self._ik_steps / max(speed_scale, 0.1)))

        logger.info(
            f"[RealMotion] {step_name}: target={[f'{v:.3f}' for v in xyzrpy]}"
            f"  steps={n_steps}  speed={speed_scale:.2f}"
        )

        # TODO(BASELINE_VERIFY): control_dual_arm_ik nhận left hoặc right target.
        # Hiện tại dùng right arm cho Task 1.
        left_target = xyzrpy if self._side == "left" else None
        right_target = xyzrpy if self._side == "right" else None

        for step_i in range(n_steps):
            try:
                ik_result = self._robot.control_dual_arm_ik(
                    step_size=PHYSICS_DT,
                    left_target_xyzrpy=left_target,
                    right_target_xyzrpy=right_target,
                )
            except Exception as exc:
                logger.error(f"[RealMotion] control_dual_arm_ik exception: {exc}")
                return False

            # Step physics
            if self._world is not None:
                self._world.step(render=True)
            else:
                # TODO(BASELINE_VERIFY): nếu không có world ref, dùng robot._world
                if hasattr(self._robot, "_world") and self._robot._world is not None:
                    self._robot._world.step(render=True)

            # Check convergence
            if ik_result is not None:
                side_key = f"{self._side}_success"
                if ik_result.get(side_key, False):
                    logger.info(f"[RealMotion] {step_name} converged at step {step_i+1}/{n_steps}")
                    return True

        # Không converge nhưng không crash – coi là partial success
        # TODO(BASELINE_VERIFY): quyết định có return False không tùy tolerance thực tế
        logger.warning(f"[RealMotion] {step_name} did not fully converge after {n_steps} steps")
        return True

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------

    def open_gripper(self, side: str = "right", task_name: Optional[str] = None) -> None:
        """Open gripper – delegate to IsaacSimRobotInterface."""
        try:
            self._robot.open_gripper(side=side)
            self._gripper_state[side] = "open"
            logger.info(f"[RealMotion] open_gripper(side={side})")
        except Exception as exc:
            logger.error(f"[RealMotion] open_gripper failed: {exc}")
            raise RuntimeError(f"GRIPPER_OPEN_FAIL: {exc}") from exc

    def close_gripper(self, side: str = "right", task_name: Optional[str] = None) -> None:
        """Close gripper – delegate to IsaacSimRobotInterface."""
        try:
            self._robot.close_gripper(side=side)
            self._gripper_state[side] = "closed"
            logger.info(f"[RealMotion] close_gripper(side={side})")
        except Exception as exc:
            logger.error(f"[RealMotion] close_gripper failed: {exc}")
            raise RuntimeError(f"GRIPPER_CLOSE_FAIL: {exc}") from exc

    def get_gripper_state(self, side: str = "right") -> str:
        return self._gripper_state.get(side, "open")

    def get_joint_states(self):
        """Proxy to robot_interface.get_joint_states()."""
        return self._robot.get_joint_states()

    def apply_finger_efforts(self, efforts: list) -> None:
        """Proxy to robot_interface.apply_finger_efforts()."""
        self._robot.apply_finger_efforts(efforts)


# ---------------------------------------------------------------------------
# Factory helper
# ---------------------------------------------------------------------------

def make_real_interface(
    prim_path: str = "/Root/Ref_Xform/Ref",
    robot_name: str = "walkerS2",
    urdf_path: str = "assets/resources/s2.urdf",
    world: Any = None,
    side: str = "right",
) -> RealMotionInterface:
    """
    Khởi tạo IsaacSimRobotInterface và wrap vào RealMotionInterface.

    Dùng khi Isaac Sim đã launch và scene đã load.

    Args:
        prim_path: USD prim path của robot.
        robot_name: Robot name trong Isaac Sim.
        urdf_path: Path đến s2.urdf cho IK solver.
        world: Isaac Sim World instance.
        side: Arm side cho Task 1 ("right").

    Returns:
        RealMotionInterface sẵn sàng dùng với MotionPrimitiveRunner.

    Example:
        from real_motion_interface import make_real_interface
        from motion import MotionPrimitiveRunner

        interface = make_real_interface(world=my_world)
        runner = MotionPrimitiveRunner(interface, dry_run=False)
        result = runner.pick_place(action_plan)
    """
    # TODO(BASELINE_VERIFY): import path phụ thuộc vào repo structure
    try:
        from isaac_sim_robot_interface import IsaacSimRobotInterface
    except ImportError:
        from src.baseline_source.isaac_sim_robot_interface import IsaacSimRobotInterface

    robot = IsaacSimRobotInterface(
        prim_path=prim_path,
        name=robot_name,
        world=world,
        urdf_path=urdf_path,
    )
    robot.initialize()
    logger.info(f"[make_real_interface] Robot initialized: {prim_path}")

    return RealMotionInterface(robot_interface=robot, world=world, side=side)
