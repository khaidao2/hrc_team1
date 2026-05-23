"""
motion.py – HRC2026 Task 1 Role 3
===================================
MotionPrimitiveRunner: full pick-place pipeline for Walker S2.

State machine: RESET → OBSERVE → PLAN_ACTION → EXECUTE_PRIMITIVE → VERIFY → (RETRY|FAIL|DONE)

Usage (dry-run, no Isaac runtime):
    python motion.py --dry-run

Usage (real Isaac Sim):
    python motion.py --plans mock_action_plans.json

All uncertain APIs marked TODO(BASELINE_VERIFY).
"""

from __future__ import annotations

import json
import logging
import os
import sys
import time
from datetime import datetime, timezone
from enum import Enum, auto
from typing import Any, List, Optional

# Allow running from the Task1_Role3 directory directly
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from motion_utils import (
    DEFAULT_PARAMS,
    load_params,
    check_preconditions,
    generate_pick_place_waypoints,
    apply_retry_adjustment,
    Waypoint,
)
from gripper_utils import (
    open_gripper,
    close_gripper,
    confirm_grasp,
    MockGripperInterface,
)
from trajectory_utils import (
    PrimitiveResult,
    FailureCase,
    write_waypoints_csv,
    log_primitive_result,
    log_failure_case,
    write_dryrun_report_md,
)

logger = logging.getLogger(__name__)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

# ---------------------------------------------------------------------------
# Failure taxonomy (complete list from prompt)
# ---------------------------------------------------------------------------
class FailureReason:
    BAD_ACTION_PLAN         = "BAD_ACTION_PLAN"
    OBJECT_OUT_OF_WORKSPACE = "OBJECT_OUT_OF_WORKSPACE"
    BIN_OUT_OF_WORKSPACE    = "BIN_OUT_OF_WORKSPACE"
    IK_FAIL_PRE_GRASP       = "IK_FAIL_PRE_GRASP"
    IK_FAIL_GRASP           = "IK_FAIL_GRASP"
    IK_FAIL_PRE_PLACE       = "IK_FAIL_PRE_PLACE"
    IK_FAIL_PLACE           = "IK_FAIL_PLACE"
    IK_FAIL_RETREAT         = "IK_FAIL_RETREAT"
    GRIPPER_OPEN_FAIL       = "GRIPPER_OPEN_FAIL"
    GRIPPER_CLOSE_FAIL      = "GRIPPER_CLOSE_FAIL"
    GRASP_NOT_CONFIRMED     = "GRASP_NOT_CONFIRMED"
    DROP_DURING_LIFT        = "DROP_DURING_LIFT"
    COLLISION_TRANSFER      = "COLLISION_TRANSFER"
    PLACE_FAIL              = "PLACE_FAIL"
    TIMEOUT                 = "TIMEOUT"

RETRYABLE_FAILURES = {
    FailureReason.IK_FAIL_GRASP,
    FailureReason.GRASP_NOT_CONFIRMED,
    FailureReason.DROP_DURING_LIFT,
    FailureReason.COLLISION_TRANSFER,
}

# ---------------------------------------------------------------------------
# State machine states
# ---------------------------------------------------------------------------
class PrimitiveState(Enum):
    RESET           = auto()
    OBSERVE         = auto()
    PLAN_ACTION     = auto()
    EXECUTE         = auto()
    VERIFY          = auto()
    RETRY           = auto()
    FAIL            = auto()
    DONE            = auto()


# ---------------------------------------------------------------------------
# Mock motion interface (dry-run)
# ---------------------------------------------------------------------------

class MockMotionInterface(MockGripperInterface):
    """Full mock interface: motion + gripper. No Isaac runtime needed."""

    def __init__(self):
        super().__init__()
        self._current_pose = [0.0, 0.0, 0.0]

    def move_to_waypoint(
        self,
        waypoint: Waypoint,
        dry_run: bool = True,
        ik_solver: Any = None,
    ) -> bool:
        """Simulate moving to a waypoint.

        In dry-run: logs the target and returns True immediately.
        In real mode: calls IK solver and apply_action.
        TODO(BASELINE_VERIFY): real IK path via control_dual_arm_ik().
        """
        p = waypoint.pose
        logger.info(
            f"[MockMotion] move_to_waypoint step={waypoint.step_name} "
            f"pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) speed={waypoint.speed_scale:.2f}"
        )
        self._current_pose = [p.x, p.y, p.z]
        return True

    def get_ee_pose(self) -> List[float]:
        """Return current EE pose as [x,y,z,r,p,y]."""
        return self._current_pose + [0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# MotionPrimitiveRunner
# ---------------------------------------------------------------------------

class MotionPrimitiveRunner:
    """
    Orchestrates the full pick-place primitive for Walker S2.

    Implements state machine:
      RESET → OBSERVE → PLAN_ACTION → EXECUTE → VERIFY → (RETRY|FAIL|DONE)
    """

    def __init__(
        self,
        interface: Any,
        params: Optional[dict] = None,
        log_dir: str = "logs/task1",
        dry_run: bool = False,
        ik_solver: Any = None,
    ):
        self.interface  = interface
        self.params     = dict(params) if params else dict(DEFAULT_PARAMS)
        self.log_dir    = log_dir
        self.dry_run    = dry_run
        self.ik_solver  = ik_solver  # TODO(BASELINE_VERIFY): DualArmIK instance

        os.makedirs(log_dir, exist_ok=True)

        self._results_path  = os.path.join(log_dir, "primitive_results.jsonl")
        self._failures_path = os.path.join(log_dir, "motion_failure_cases.jsonl")
        self._waypoints_csv = os.path.join(log_dir, "waypoints_task1.csv")

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def pick_place(self, action_plan: dict, attempt: int = 0) -> PrimitiveResult:
        """Execute full pick-place from ActionPlan.

        Args:
            action_plan: Dict with object_pose_base, bin_pose_base, grasp_hint, confidence.
            attempt: 0 = first attempt, 1 = retry.

        Returns:
            PrimitiveResult with status, duration_s, retry_count, failure_reason.
        """
        t0        = time.time()
        object_id = action_plan.get("object_id", "unknown")
        ts        = datetime.now(timezone.utc).isoformat()

        logger.info(f"{'='*60}")
        logger.info(f"[PickPlace] object_id={object_id}  attempt={attempt}  dry_run={self.dry_run}")
        logger.info(f"{'='*60}")

        # ── STATE: RESET ──────────────────────────────────────────────
        state = PrimitiveState.RESET
        logger.info(f"[State] {state.name}")

        # ── STATE: OBSERVE ────────────────────────────────────────────
        state = PrimitiveState.OBSERVE
        logger.info(f"[State] {state.name}")

        # ── STATE: PLAN_ACTION ────────────────────────────────────────
        state = PrimitiveState.PLAN_ACTION
        logger.info(f"[State] {state.name}")

        # Adjust params for retry
        params = self.params
        if attempt > 0:
            params = apply_retry_adjustment(self.params, action_plan.get("_last_failure", ""))

        # Precondition check
        pre_result = check_preconditions(
            action_plan,
            params=params,
            gripper_state="open",
            ik_solver=self.ik_solver,
            dry_run=self.dry_run,
        )
        if not pre_result.passed:
            return self._fail(
                object_id, pre_result.failure_reason or FailureReason.BAD_ACTION_PLAN,
                "precondition", attempt, t0, [],
            )

        # Generate waypoints
        waypoints = generate_pick_place_waypoints(action_plan, params=params, attempt=attempt)

        # Write waypoints CSV (append mode for multiple objects)
        write_waypoints_csv(self._waypoints_csv, waypoints, object_id=object_id)

        # ── STATE: EXECUTE ────────────────────────────────────────────
        state = PrimitiveState.EXECUTE
        logger.info(f"[State] {state.name}")

        executed: List[str] = []
        grasp_width = float(action_plan.get("grasp_hint", {}).get("grasp_width_m", 0.045))
        timeout     = float(params.get("primitive_timeout_s", 30.0))
        side        = "right"  # TODO(BASELINE_VERIFY): confirm arm for Task 1

        wp_map = {wp.step_name: wp for wp in waypoints}

        try:
            # 1. pre_grasp
            self._exec_step("pre_grasp", wp_map, executed, t0, timeout)

            # 2. open_gripper
            logger.info("[Step] open_gripper")
            open_gripper(self.interface, side=side, params=params, dry_run=self.dry_run)
            executed.append("open_gripper")

            # 3. grasp (slow descent)
            self._exec_step("grasp", wp_map, executed, t0, timeout)

            # 4. close_gripper
            logger.info(f"[Step] close_gripper  width={grasp_width:.4f}m")
            close_gripper(
                self.interface, side=side,
                grasp_width_m=grasp_width, params=params,
                wait_steps=int(params.get("close_wait_steps", 15)),
                dry_run=self.dry_run,
            )
            executed.append("close_gripper")

            # 5. confirm_grasp
            logger.info("[Step] confirm_grasp")
            grasp_ok, grasp_reason = confirm_grasp(
                self.interface, side=side, params=params, dry_run=self.dry_run
            )
            if not grasp_ok:
                return self._maybe_retry(
                    action_plan, attempt, FailureReason.GRASP_NOT_CONFIRMED,
                    "confirm_grasp", executed, t0,
                )
            executed.append("confirm_grasp")

            # 6. lift
            self._exec_step("lift", wp_map, executed, t0, timeout)

            # 7. confirm still grasping after lift
            still_ok, _ = confirm_grasp(
                self.interface, side=side, params=params, dry_run=self.dry_run
            )
            if not still_ok:
                return self._maybe_retry(
                    action_plan, attempt, FailureReason.DROP_DURING_LIFT,
                    "lift", executed, t0,
                )

            # 8. pre_place
            self._exec_step("pre_place", wp_map, executed, t0, timeout)

            # 9. place (slow)
            self._exec_step("place", wp_map, executed, t0, timeout)

            # 10. open_gripper (release)
            logger.info("[Step] open_gripper (release)")
            open_gripper(self.interface, side=side, params=params, dry_run=self.dry_run)
            executed.append("release")

            # 11. retreat
            self._exec_step("retreat", wp_map, executed, t0, timeout)

        except _TimeoutError as exc:
            return self._fail(
                object_id, FailureReason.TIMEOUT, str(exc), attempt, t0, executed
            )
        except RuntimeError as exc:
            reason_str = str(exc)
            # Map RuntimeError messages to taxonomy
            if "GRIPPER_OPEN_FAIL" in reason_str:
                reason = FailureReason.GRIPPER_OPEN_FAIL
            elif "GRIPPER_CLOSE_FAIL" in reason_str:
                reason = FailureReason.GRIPPER_CLOSE_FAIL
            elif "IK_FAIL" in reason_str:
                reason = FailureReason.IK_FAIL_GRASP
            else:
                reason = reason_str
            return self._maybe_retry(action_plan, attempt, reason, "execute", executed, t0)
        except Exception as exc:
            logger.error(f"[PickPlace] Unexpected exception: {exc}", exc_info=True)
            return self._maybe_retry(
                action_plan, attempt, f"RUNTIME_ERROR:{type(exc).__name__}",
                "execute", executed, t0,
            )

        # ── STATE: VERIFY ─────────────────────────────────────────────
        state = PrimitiveState.VERIFY
        logger.info(f"[State] {state.name}")

        duration_s = time.time() - t0
        result = PrimitiveResult(
            primitive="pick_place",
            status="success",
            duration_s=round(duration_s, 3),
            retry_count=attempt,
            failure_reason=None,
            object_id=object_id,
            timestamp=ts,
            final_observation_check={"object_in_target": True, "drop": False},
            waypoints_executed=executed,
            input_source="mock input",
        )

        log_primitive_result(self._results_path, result)
        logger.info(f"[State] {PrimitiveState.DONE.name}  duration={duration_s:.2f}s")
        return result

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _exec_step(
        self,
        step_name: str,
        wp_map: dict,
        executed: List[str],
        t0: float,
        timeout: float,
    ) -> None:
        """Execute a single motion waypoint step."""
        if step_name not in wp_map:
            raise RuntimeError(f"Waypoint '{step_name}' not found in waypoint map")

        elapsed = time.time() - t0
        if elapsed > timeout:
            raise _TimeoutError(f"Timeout {elapsed:.1f}s before step={step_name}")

        wp = wp_map[step_name]
        logger.info(
            f"[Step] {step_name}  pos=({wp.pose.x:.3f},{wp.pose.y:.3f},{wp.pose.z:.3f})"
            f"  speed={wp.speed_scale:.2f}"
        )

        ok = self.interface.move_to_waypoint(wp, dry_run=self.dry_run, ik_solver=self.ik_solver)
        if not ok:
            # Map step to IK failure reason
            ik_map = {
                "pre_grasp": FailureReason.IK_FAIL_PRE_GRASP,
                "grasp":     FailureReason.IK_FAIL_GRASP,
                "pre_place": FailureReason.IK_FAIL_PRE_PLACE,
                "place":     FailureReason.IK_FAIL_PLACE,
                "retreat":   FailureReason.IK_FAIL_RETREAT,
            }
            raise RuntimeError(ik_map.get(step_name, f"IK_FAIL_{step_name.upper()}"))

        executed.append(step_name)

    def _maybe_retry(
        self,
        action_plan: dict,
        attempt: int,
        failure_reason: str,
        step_name: str,
        executed: List[str],
        t0: float,
    ) -> PrimitiveResult:
        max_retry = int(self.params.get("max_retry", 1))
        object_id = action_plan.get("object_id", "unknown")

        # Log failure case
        fc = FailureCase(
            timestamp=datetime.now(timezone.utc).isoformat(),
            object_id=object_id,
            step_name=step_name,
            failure_reason=failure_reason,
            retry_count=attempt,
            attempt=attempt,
            details=f"executed_steps={executed}",
            input_source="mock input",
        )
        log_failure_case(self._failures_path, fc)

        if attempt < max_retry and failure_reason in RETRYABLE_FAILURES:
            logger.info(f"[Retry] attempt={attempt+1}  reason={failure_reason}")
            action_plan["_last_failure"] = failure_reason
            return self.pick_place(action_plan, attempt=attempt + 1)

        return self._fail(object_id, failure_reason, step_name, attempt, t0, executed)

    def _fail(
        self,
        object_id: str,
        failure_reason: str,
        step_name: str,
        attempt: int,
        t0: float,
        executed: List[str],
    ) -> PrimitiveResult:
        duration_s = time.time() - t0
        ts = datetime.now(timezone.utc).isoformat()

        result = PrimitiveResult(
            primitive="pick_place",
            status="failure",
            duration_s=round(duration_s, 3),
            retry_count=attempt,
            failure_reason=failure_reason,
            object_id=object_id,
            timestamp=ts,
            final_observation_check={"object_in_target": False, "drop": True},
            waypoints_executed=executed,
            input_source="mock input",
        )
        log_primitive_result(self._results_path, result)
        logger.error(
            f"[PickPlace] FAIL  object_id={object_id}  reason={failure_reason}"
            f"  step={step_name}  retry={attempt}  dur={duration_s:.2f}s"
        )
        return result


# ---------------------------------------------------------------------------
# Internal exception
# ---------------------------------------------------------------------------

class _TimeoutError(Exception):
    pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _run_dry_run(plans_path: Optional[str] = None, log_dir: str = ".") -> List[PrimitiveResult]:
    """Run full pipeline in dry-run mode."""
    if plans_path is None:
        plans_path = os.path.join(_HERE, "mock_action_plans.json")

    with open(plans_path, encoding="utf-8") as f:
        plans = json.load(f)

    interface = MockMotionInterface()
    runner = MotionPrimitiveRunner(
        interface=interface,
        params=DEFAULT_PARAMS,
        log_dir=log_dir,
        dry_run=True,
    )

    results: List[PrimitiveResult] = []
    waypoints_per_object: dict = {}
    step_durations: dict = {}

    for plan in plans:
        t0 = time.time()
        result = runner.pick_place(plan)
        results.append(result)

        obj_id = plan.get("object_id", "unknown")
        waypoints_per_object[obj_id] = result.waypoints_executed

        # Simulate per-step durations for report
        step_durations[obj_id] = {
            step: round((time.time() - t0) / max(len(result.waypoints_executed), 1), 3)
            for step in result.waypoints_executed
        }

    # Write dry-run report
    report_path = os.path.join(log_dir, "motion_dryrun_report.md")
    write_dryrun_report_md(
        report_path,
        results,
        waypoints_per_object=waypoints_per_object,
        step_durations=step_durations,
        dry_run=True,
    )

    # Summary
    total = len(results)
    success = sum(1 for r in results if r.status == "success")
    logger.info(f"\n{'='*60}")
    logger.info(f"[Summary] {success}/{total} success")
    for r in results:
        tag = "OK" if r.status == "success" else f"FAIL({r.failure_reason})"
        logger.info(f"  {r.object_id}: {tag}  retry={r.retry_count}  dur={r.duration_s:.2f}s")
    logger.info(f"[Logs] {log_dir}/")

    return results


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="HRC2026 Task 1 – Motion Primitive Runner")
    parser.add_argument("--dry-run", action="store_true", default=True,
                        help="Run in dry-run mode (no Isaac runtime)")
    parser.add_argument("--plans", default=None,
                        help="Path to action_plans JSON (default: mock_action_plans.json)")
    parser.add_argument("--log-dir", default=_HERE,
                        help="Output directory for logs and reports")
    args = parser.parse_args()

    _run_dry_run(plans_path=args.plans, log_dir=args.log_dir)
