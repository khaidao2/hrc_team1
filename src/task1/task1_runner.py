"""
task1_runner.py – HRC2026 Task 1 Role 3
=========================================
Gọi file này từ bên trong workspace Isaac Sim đang chạy.

Paste vào cuối main.py (sau đoạn robot.initialize()):

    from task1_runner import run_task1
    run_task1(robot, world)
"""

from __future__ import annotations

import json
import logging
import os
import sys
from typing import Any, Optional

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# RobotArticulation adapter
# ---------------------------------------------------------------------------

class RobotArticulationAdapter:
    """
    Wrap RobotArticulation (baseline) để dùng với RealMotionInterface.

    RobotArticulation trong workspace có:
      - apply_action(ArticulationActions)
      - get_joint_positions()
      - get_joint_velocities()
      - dof_names

    Nhưng KHÔNG có control_dual_arm_ik() hay open/close_gripper().
    Adapter này bridge các hàm đó.
    """

    # Joint names từ isaac_sim_robot_interface.py
    ARM_JOINT_NAMES = [
        "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
        "L_elbow_roll_joint",     "L_elbow_yaw_joint",
        "L_wrist_pitch_joint",    "L_wrist_roll_joint",
        "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
        "R_elbow_roll_joint",     "R_elbow_yaw_joint",
        "R_wrist_pitch_joint",    "R_wrist_roll_joint",
    ]
    FINGER_JOINT_NAMES = [
        "L_finger1_joint", "L_finger2_joint",
        "R_finger1_joint", "R_finger2_joint",
    ]

    GRIPPER_OPEN_WIDTH  = -0.0215
    GRIPPER_CLOSE_WIDTH =  0.01
    GRIPPER_CLOSE_TAU   =  100.0

    def __init__(self, robot: Any, world: Any, urdf_path: Optional[str] = None):
        """
        Args:
            robot: RobotArticulation instance (đã initialize).
            world: Isaac Sim World instance.
            urdf_path: Path đến s2.urdf cho DualArmIK.
        """
        self._robot  = robot
        self._world  = world
        self._ik     = None
        self._finger_state = "open"

        # Build joint index maps
        dof_names = list(robot.dof_names)
        self.arm_joint_indices    = [dof_names.index(j) for j in self.ARM_JOINT_NAMES if j in dof_names]
        self.finger_joint_indices = [dof_names.index(j) for j in self.FINGER_JOINT_NAMES if j in dof_names]

        # Init IK nếu có urdf
        if urdf_path and os.path.isfile(urdf_path):
            self._init_ik(urdf_path, dof_names)

    def _init_ik(self, urdf_path: str, dof_names: list):
        try:
            from src.baseline_source.DualArmIK import DualArmIK
        except ImportError:
            try:
                from DualArmIK import DualArmIK
            except ImportError:
                logger.warning("[Adapter] DualArmIK not found – IK disabled, using joint-space hold")
                return

        self._ik = DualArmIK(urdf_path)
        import torch
        positions = self._robot.get_joint_positions().flatten().tolist()
        self._ik.sync_joint_positions(dof_names, positions)
        self._ik.save_initial_q()
        logger.info("[Adapter] DualArmIK initialized")

    # ------------------------------------------------------------------
    # IK control (dùng bởi RealMotionInterface)
    # ------------------------------------------------------------------

    def control_dual_arm_ik(
        self,
        step_size: float,
        left_target_xyzrpy=None,
        right_target_xyzrpy=None,
        **kwargs,
    ) -> dict:
        """Solve IK và apply action. Returns dict với left/right_success."""
        import torch
        from isaacsim.core.utils.types import ArticulationActions

        if self._ik is None:
            # Không có IK – giữ nguyên vị trí hiện tại
            return {"left_success": True, "right_success": True}

        dof_names = list(self._robot.dof_names)
        positions = self._robot.get_joint_positions().flatten().tolist()
        self._ik.sync_joint_positions(dof_names, positions)

        result = self._ik.solve_dual_arm(
            left_target_xyzrpy=left_target_xyzrpy,
            right_target_xyzrpy=right_target_xyzrpy,
            isaac_joint_names=dof_names,
            isaac_joint_positions=positions,
            **kwargs,
        )

        all_indices  = []
        all_positions = []

        if "left_joint_positions" in result:
            # TODO(BASELINE_VERIFY): LEFT_ARM_JOINTS order từ DualArmIK
            left_isaac_idx = [dof_names.index(j) for j in getattr(self._ik, "LEFT_ARM_JOINTS", [])
                              if j in dof_names]
            all_indices.extend(left_isaac_idx)
            all_positions.extend(result["left_joint_positions"])

        if "right_joint_positions" in result:
            right_isaac_idx = [dof_names.index(j) for j in getattr(self._ik, "RIGHT_ARM_JOINTS", [])
                               if j in dof_names]
            all_indices.extend(right_isaac_idx)
            all_positions.extend(result["right_joint_positions"])

        if all_indices:
            self._robot.apply_action(
                ArticulationActions(
                    joint_positions=torch.tensor([all_positions], dtype=torch.float32),
                    joint_indices=torch.tensor(all_indices, dtype=torch.int32),
                )
            )

        return result

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------

    def open_gripper(self, side: str = "right", **kwargs) -> None:
        import torch
        from isaacsim.core.utils.types import ArticulationActions

        target = [self.GRIPPER_OPEN_WIDTH, self.GRIPPER_OPEN_WIDTH]
        if side == "left":
            indices = torch.tensor(self.finger_joint_indices[:2], dtype=torch.int32)
        elif side == "right":
            indices = torch.tensor(self.finger_joint_indices[2:4], dtype=torch.int32)
        else:
            indices = torch.tensor(self.finger_joint_indices, dtype=torch.int32)
            target  = target * 2

        self._robot.apply_action(
            ArticulationActions(
                joint_positions=torch.tensor([target], dtype=torch.float32),
                joint_indices=indices,
            )
        )
        self._finger_state = "open"
        logger.info(f"[Adapter] open_gripper(side={side})")

    def close_gripper(self, side: str = "right", **kwargs) -> None:
        import torch
        from isaacsim.core.utils.types import ArticulationActions

        target = [self.GRIPPER_CLOSE_WIDTH, self.GRIPPER_CLOSE_WIDTH]
        if side == "left":
            indices = torch.tensor(self.finger_joint_indices[:2], dtype=torch.int32)
        elif side == "right":
            indices = torch.tensor(self.finger_joint_indices[2:4], dtype=torch.int32)
        else:
            indices = torch.tensor(self.finger_joint_indices, dtype=torch.int32)
            target  = target * 2

        self._robot.apply_action(
            ArticulationActions(
                joint_positions=torch.tensor([target], dtype=torch.float32),
                joint_indices=indices,
            )
        )
        self._finger_state = "closed"
        logger.info(f"[Adapter] close_gripper(side={side})")

    def get_gripper_state(self, side: str = "right") -> str:
        return self._finger_state

    def get_joint_states(self) -> dict:
        try:
            import torch
            pos = self._robot.get_joint_positions().flatten()
            vel = self._robot.get_joint_velocities().flatten()
            arm_idx    = torch.tensor(self.arm_joint_indices, dtype=torch.long)
            finger_idx = torch.tensor(self.finger_joint_indices, dtype=torch.long)
            return {
                "arm_positions":    pos[arm_idx].tolist(),
                "finger_positions": pos[finger_idx].tolist(),
                "arm_velocities":   vel[arm_idx].tolist(),
            }
        except Exception as exc:
            logger.error(f"[Adapter] get_joint_states failed: {exc}")
            return {}

    def apply_finger_efforts(self, efforts: list) -> None:
        import torch
        from isaacsim.core.utils.types import ArticulationActions
        idx = torch.tensor(self.finger_joint_indices, dtype=torch.int32)
        self._robot.apply_action(
            ArticulationActions(
                joint_efforts=torch.tensor([efforts], dtype=torch.float32),
                joint_indices=idx,
            )
        )


# ---------------------------------------------------------------------------
# Main entry point – gọi từ workspace
# ---------------------------------------------------------------------------

def run_task1(
    robot: Any,
    world: Any,
    plans_path: str = "tests/mock_action_plans.json",
    urdf_path: str = "assets/resources/s2.urdf",
    log_dir: str = "logs/task1",
    side: str = "right",
):
    """
    Chạy Task 1 pick-place từ workspace Isaac Sim.

    Paste vào cuối main.py sau đoạn robot.initialize():

        from task1_runner import run_task1
        run_task1(robot, world)

    Args:
        robot:      RobotArticulation instance (đã initialize).
        world:      Isaac Sim World instance (đang play).
        plans_path: Path đến action plans JSON.
        urdf_path:  Path đến s2.urdf cho IK.
        log_dir:    Output directory cho logs.
        side:       Arm side ("right" cho Task 1).
    """
    # Add src/task1 vào path
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)

    from real_motion_interface import RealMotionInterface
    from motion import MotionPrimitiveRunner
    from motion_utils import DEFAULT_PARAMS

    # Wrap RobotArticulation → adapter → RealMotionInterface
    adapter   = RobotArticulationAdapter(robot, world, urdf_path=urdf_path)
    interface = RealMotionInterface(
        robot_interface=adapter,
        world=world,
        side=side,
        ik_steps=150,
    )

    runner = MotionPrimitiveRunner(
        interface=interface,
        params=DEFAULT_PARAMS,
        log_dir=log_dir,
        dry_run=False,
    )

    # Load plans
    if not os.path.isfile(plans_path):
        logger.error(f"[task1_runner] plans not found: {plans_path}")
        return []

    with open(plans_path, encoding="utf-8") as f:
        plans = json.load(f)

    logger.info(f"[task1_runner] Running {len(plans)} plans on Isaac Sim...")

    results = []
    for plan in plans:
        result = runner.pick_place(plan)
        results.append(result)
        tag = "SUCCESS" if result.status == "success" else f"FAIL({result.failure_reason})"
        logger.info(f"  {result.object_id}: {tag}  dur={result.duration_s:.2f}s")

    success = sum(1 for r in results if r.status == "success")
    logger.info(f"[task1_runner] Done: {success}/{len(results)} success")
    return results
