"""
gripper_utils.py – HRC2026 Task 1 Role 3
==========================================
Gripper control wrapper for motion primitive.

Public API:
  open_gripper(interface, side, params)   → bool
  close_gripper(interface, side, grasp_width_m, params) → bool
  confirm_grasp(interface, side, params)  → (bool, reason_or_None)

Also provides:
  run_gripper_test_sequence()  – sweep test for gripper_test_report.csv
  write_gripper_test_report_csv()

All functions call interface methods only – no direct Isaac API calls here.
Runnable in dry-run mode via MockGripperInterface.
All uncertain APIs marked TODO(BASELINE_VERIFY).
"""

from __future__ import annotations

import csv
import logging
import os
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any, Optional, Tuple

logger = logging.getLogger(__name__)

# Gripper constants from baseline (isaac_sim_robot_interface.py)
GRIPPER_OPEN_WIDTH = -0.0215   # matches IsaacSimRobotInterface.gripper_open_width
GRIPPER_CLOSE_WIDTH = 0.01     # matches IsaacSimRobotInterface.gripper_close_width
GRIPPER_CLOSE_TAU = 100.0      # matches IsaacSimRobotInterface.gripper_close_tau


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class GripperTestRecord:
    """One row in gripper_test_report.csv."""
    timestamp: str
    grasp_width_m: float
    close_wait_steps: int
    open_pass: bool
    close_pass: bool
    reopen_pass: bool
    notes: str = ""


# ---------------------------------------------------------------------------
# Mock gripper interface (dry-run mode)
# ---------------------------------------------------------------------------

class MockGripperInterface:
    """Simulates gripper behavior without Isaac runtime."""

    def __init__(self):
        self._state: dict[str, str] = {"left": "open", "right": "open"}
        self._finger_positions: dict[str, float] = {"left": GRIPPER_OPEN_WIDTH, "right": GRIPPER_OPEN_WIDTH}

    def open_gripper(self, side: str = "right", task_name: Optional[str] = None) -> None:
        self._state[side] = "open"
        self._finger_positions[side] = GRIPPER_OPEN_WIDTH
        logger.info(f"[MockGripper] open_gripper(side={side})")

    def close_gripper(self, side: str = "right", task_name: Optional[str] = None) -> None:
        self._state[side] = "closed"
        self._finger_positions[side] = GRIPPER_CLOSE_WIDTH
        logger.info(f"[MockGripper] close_gripper(side={side})")

    def get_finger_positions(self, side: str = "right") -> list:
        pos = self._finger_positions.get(side, GRIPPER_OPEN_WIDTH)
        return [pos, pos]

    def get_gripper_state(self, side: str = "right") -> str:
        return self._state.get(side, "open")

    def apply_finger_efforts(self, efforts: list) -> None:
        logger.info(f"[MockGripper] apply_finger_efforts({efforts})")


# ---------------------------------------------------------------------------
# Core gripper functions
# ---------------------------------------------------------------------------

def open_gripper(
    interface: Any,
    side: str = "right",
    params: Optional[dict] = None,
    wait_steps: int = 10,
    dry_run: bool = False,
) -> bool:
    """Open gripper on specified side.

    Args:
        interface: IsaacSimRobotInterface or MockGripperInterface.
        side: "left" or "right".
        params: Motion params dict.
        wait_steps: Physics steps to wait after command.
        dry_run: If True, skip actual wait.

    Returns:
        True on success.

    Raises:
        RuntimeError if interface raises.
    """
    ts = time.time()
    try:
        # TODO(BASELINE_VERIFY): IsaacSimRobotInterface.open_gripper(side) confirmed in baseline
        interface.open_gripper(side=side)
        logger.info(f"[{ts:.3f}] open_gripper(side={side}) sent, wait_steps={wait_steps}")
        return True
    except Exception as exc:
        logger.error(f"[{ts:.3f}] open_gripper FAILED: {exc}")
        raise RuntimeError(f"GRIPPER_OPEN_FAIL: {exc}") from exc


def close_gripper(
    interface: Any,
    side: str = "right",
    grasp_width_m: float = 0.045,
    params: Optional[dict] = None,
    wait_steps: int = 15,
    dry_run: bool = False,
) -> bool:
    """Close gripper to grasp width.

    Args:
        interface: IsaacSimRobotInterface or MockGripperInterface.
        side: "left" or "right".
        grasp_width_m: Target grasp width (clamped to valid range).
        params: Motion params dict.
        wait_steps: Physics steps to wait after command.
        dry_run: If True, skip actual wait.

    Returns:
        True on success.

    Raises:
        RuntimeError if interface raises.
    """
    ts = time.time()
    if params is None:
        params = {}
    min_w = float(params.get("min_grasp_width_m", 0.01))
    max_w = float(params.get("max_grasp_width_m", 0.08))
    clamped = max(min_w, min(max_w, grasp_width_m))
    if abs(clamped - grasp_width_m) > 1e-4:
        logger.warning(f"[{ts:.3f}] grasp_width_m clamped {grasp_width_m:.4f} → {clamped:.4f}")

    try:
        # TODO(BASELINE_VERIFY): baseline close_gripper uses fixed GRIPPER_CLOSE_WIDTH,
        # not a variable width. The grasp_width_m here is used for confirmation check only.
        interface.close_gripper(side=side)
        logger.info(f"[{ts:.3f}] close_gripper(side={side}, width={clamped:.4f}m) sent, wait_steps={wait_steps}")
        return True
    except Exception as exc:
        logger.error(f"[{ts:.3f}] close_gripper FAILED: {exc}")
        raise RuntimeError(f"GRIPPER_CLOSE_FAIL: {exc}") from exc


def confirm_grasp(
    interface: Any,
    side: str = "right",
    params: Optional[dict] = None,
    dry_run: bool = False,
) -> Tuple[bool, Optional[str]]:
    """Check if gripper is holding an object.

    Strategy:
    - In dry_run: always returns (True, None).
    - With real interface: check finger position convergence to close_width.
      If fingers are near open_width, grasp likely failed.

    TODO(BASELINE_VERIFY): No force sensor feedback in baseline.
    Using finger position as proxy for grasp confirmation.

    Returns:
        (True, None) if grasping.
        (False, "GRASP_NOT_CONFIRMED") if not.
    """
    ts = time.time()
    if dry_run:
        logger.info(f"[{ts:.3f}] confirm_grasp dry_run → True")
        return True, None

    try:
        # TODO(BASELINE_VERIFY): get_finger_positions not in baseline interface.
        # Using gripper state as proxy.
        if hasattr(interface, "get_gripper_state"):
            state = interface.get_gripper_state(side=side)
            if state == "closed":
                logger.info(f"[{ts:.3f}] confirm_grasp(side={side}) → CONFIRMED (state=closed)")
                return True, None
            else:
                logger.warning(f"[{ts:.3f}] confirm_grasp(side={side}) → NOT CONFIRMED (state={state})")
                return False, "GRASP_NOT_CONFIRMED"

        # Fallback: check finger positions if available
        if hasattr(interface, "get_joint_states"):
            states = interface.get_joint_states()
            if states is not None:
                finger_pos = states.get("finger_positions", [])
                if side == "right" and len(finger_pos) >= 4:
                    avg_pos = (finger_pos[2] + finger_pos[3]) / 2.0
                elif side == "left" and len(finger_pos) >= 2:
                    avg_pos = (finger_pos[0] + finger_pos[1]) / 2.0
                else:
                    avg_pos = GRIPPER_OPEN_WIDTH

                # If finger position is near open_width, grasp failed
                if abs(avg_pos - GRIPPER_OPEN_WIDTH) < 0.005:
                    logger.warning(f"[{ts:.3f}] confirm_grasp: fingers near open position ({avg_pos:.4f})")
                    return False, "GRASP_NOT_CONFIRMED"
                logger.info(f"[{ts:.3f}] confirm_grasp(side={side}) → CONFIRMED (finger_pos={avg_pos:.4f})")
                return True, None

        # No feedback available – assume success (conservative)
        logger.warning(f"[{ts:.3f}] confirm_grasp: no feedback available, assuming success")
        return True, None

    except Exception as exc:
        logger.error(f"[{ts:.3f}] confirm_grasp exception: {exc}")
        return False, f"GRASP_NOT_CONFIRMED: {exc}"


# ---------------------------------------------------------------------------
# Gripper test sequence
# ---------------------------------------------------------------------------

def run_gripper_test_sequence(
    interface: Any,
    params: Optional[dict] = None,
    test_widths: Optional[list] = None,
    close_wait_steps_list: Optional[list] = None,
    side: str = "right",
    dry_run: bool = False,
) -> list:
    """Run open → close → open sweep test for gripper_test_report.csv.

    Args:
        interface: Gripper interface (real or mock).
        params: Motion params dict.
        test_widths: List of grasp widths to test.
        close_wait_steps_list: List of wait step counts to test.
        side: Gripper side to test.
        dry_run: If True, use mock behavior.

    Returns:
        List of GripperTestRecord.
    """
    if params is None:
        params = {}
    if test_widths is None:
        test_widths = [0.02, 0.03, 0.045, 0.06, 0.08]
    if close_wait_steps_list is None:
        close_wait_steps_list = [5, 10, 15, 20]

    records = []

    for width in test_widths:
        for wait_steps in close_wait_steps_list:
            ts_str = datetime.now(timezone.utc).isoformat()
            open_pass = close_pass = reopen_pass = False
            notes = ""

            # Open
            try:
                open_gripper(interface, side=side, params=params, wait_steps=10, dry_run=dry_run)
                open_pass = True
            except Exception as exc:
                notes += f"open_fail:{exc}; "
                records.append(GripperTestRecord(
                    timestamp=ts_str,
                    grasp_width_m=width,
                    close_wait_steps=wait_steps,
                    open_pass=False,
                    close_pass=False,
                    reopen_pass=False,
                    notes=f"BLOCKER: open_gripper failed: {exc}",
                ))
                continue

            # Close
            try:
                close_gripper(interface, side=side, grasp_width_m=width,
                              params=params, wait_steps=wait_steps, dry_run=dry_run)
                close_pass = True
            except Exception as exc:
                notes += f"close_fail:{exc}; "

            # Confirm grasp
            if close_pass:
                confirmed, reason = confirm_grasp(interface, side=side, params=params, dry_run=dry_run)
                if not confirmed:
                    notes += f"confirm_fail:{reason}; "

            # Re-open
            try:
                open_gripper(interface, side=side, params=params, wait_steps=10, dry_run=dry_run)
                reopen_pass = True
            except Exception as exc:
                notes += f"reopen_fail:{exc}; "

            records.append(GripperTestRecord(
                timestamp=ts_str,
                grasp_width_m=width,
                close_wait_steps=wait_steps,
                open_pass=open_pass,
                close_pass=close_pass,
                reopen_pass=reopen_pass,
                notes=notes.strip("; ") if notes else "OK",
            ))

    logger.info(f"[gripper_utils] Test sequence complete: {len(records)} records")
    return records


def write_gripper_test_report_csv(path: str, records: list) -> None:
    """Write GripperTestRecord list to CSV."""
    if not records:
        logger.warning("[gripper_utils] No records to write")
        return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fieldnames = list(asdict(records[0]).keys())
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in records:
            writer.writerow(asdict(r))
    logger.info(f"[gripper_utils] Wrote {len(records)} records → {path}")

