"""
motion_debug.py – HRC2026 Task 1 Role 3
=========================================
Dry-run script: runs full pick-place pipeline with mock ActionPlan.
No Isaac runtime required.

Usage:
    python motion_debug.py
    python motion_debug.py --plans mock_action_plans.json --log-dir .
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from datetime import datetime, timezone

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from motion_utils import DEFAULT_PARAMS, generate_pick_place_waypoints, check_preconditions
from gripper_utils import (
    MockGripperInterface,
    run_gripper_test_sequence,
    write_gripper_test_report_csv,
)
from trajectory_utils import (
    PrimitiveResult,
    FailureCase,
    write_waypoints_csv,
    log_primitive_result,
    log_failure_case,
    write_dryrun_report_md,
)
from motion import MotionPrimitiveRunner, MockMotionInterface

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _banner(title: str) -> None:
    logger.info("")
    logger.info("=" * 60)
    logger.info(f"  {title}")
    logger.info("=" * 60)


def _load_plans(path: str) -> list:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Step 1: Precondition check dry-run
# ---------------------------------------------------------------------------

def run_precondition_checks(plans: list, params: dict) -> None:
    _banner("STEP 1 – Precondition Checks")
    for plan in plans:
        obj_id = plan.get("object_id", "unknown")
        result = check_preconditions(plan, params=params, dry_run=True)
        status = "PASS" if result.passed else f"FAIL ({result.failure_reason})"
        logger.info(f"  [{status}] {obj_id}")
        if not result.passed:
            logger.warning(f"    Details: {result.details}")


# ---------------------------------------------------------------------------
# Step 2: Waypoint generation dry-run
# ---------------------------------------------------------------------------

def run_waypoint_generation(plans: list, params: dict, log_dir: str) -> dict:
    _banner("STEP 2 – Waypoint Generation")
    waypoints_per_object = {}

    for plan in plans:
        obj_id = plan.get("object_id", "unknown")
        waypoints = generate_pick_place_waypoints(plan, params=params, attempt=0)
        waypoints_per_object[obj_id] = [wp.step_name for wp in waypoints]

        logger.info(f"  {obj_id}: {len(waypoints)} waypoints")
        for wp in waypoints:
            p = wp.pose
            logger.info(
                f"    [{wp.step_name:12s}] "
                f"pos=({p.x:.3f},{p.y:.3f},{p.z:.3f})  "
                f"speed={wp.speed_scale:.2f}"
            )

    # Write combined waypoints CSV for first plan (Task 1 mock object)
    if plans:
        first_plan = plans[0]
        first_id = first_plan.get("object_id", "obj_mock_001")
        wps = generate_pick_place_waypoints(first_plan, params=params, attempt=0)
        csv_path = os.path.join(log_dir, "waypoints_task1.csv")
        write_waypoints_csv(csv_path, wps, object_id=first_id)
        logger.info(f"  waypoints_task1.csv → {csv_path}")

    return waypoints_per_object


# ---------------------------------------------------------------------------
# Step 3: Gripper test dry-run
# ---------------------------------------------------------------------------

def run_gripper_tests(log_dir: str) -> None:
    _banner("STEP 3 – Gripper Validation (dry-run)")
    mock_gripper = MockGripperInterface()
    records = run_gripper_test_sequence(
        interface=mock_gripper,
        params=DEFAULT_PARAMS,
        test_widths=[0.02, 0.03, 0.045, 0.06, 0.08],
        close_wait_steps_list=[5, 10, 15, 20],
        side="right",
        dry_run=True,
    )
    csv_path = os.path.join(log_dir, "gripper_test_report.csv")
    write_gripper_test_report_csv(csv_path, records)

    passed = sum(1 for r in records if r.open_pass and r.close_pass and r.reopen_pass)
    logger.info(f"  Gripper tests: {passed}/{len(records)} passed → {csv_path}")


# ---------------------------------------------------------------------------
# Step 4: Full pipeline dry-run
# ---------------------------------------------------------------------------

def run_full_pipeline(plans: list, params: dict, log_dir: str) -> list:
    _banner("STEP 4 – Full Pick-Place Pipeline (dry-run)")

    interface = MockMotionInterface()
    runner = MotionPrimitiveRunner(
        interface=interface,
        params=params,
        log_dir=log_dir,
        dry_run=True,
    )

    results = []
    step_durations = {}

    for plan in plans:
        obj_id = plan.get("object_id", "unknown")
        logger.info(f"\n  Running: {obj_id}")
        t0 = time.time()
        result = runner.pick_place(plan)
        elapsed = time.time() - t0
        results.append(result)

        tag = "SUCCESS" if result.status == "success" else f"FAIL({result.failure_reason})"
        logger.info(f"  → {tag}  retry={result.retry_count}  dur={result.duration_s:.2f}s")

        # Simulate per-step durations
        n_steps = max(len(result.waypoints_executed), 1)
        step_durations[obj_id] = {
            step: round(elapsed / n_steps, 3)
            for step in result.waypoints_executed
        }

    return results, step_durations


# ---------------------------------------------------------------------------
# Step 5: Write reports
# ---------------------------------------------------------------------------

def write_reports(
    results: list,
    waypoints_per_object: dict,
    step_durations: dict,
    log_dir: str,
) -> None:
    _banner("STEP 5 – Writing Reports")

    report_path = os.path.join(log_dir, "motion_dryrun_report.md")
    write_dryrun_report_md(
        report_path,
        results,
        waypoints_per_object=waypoints_per_object,
        step_durations=step_durations,
        dry_run=True,
    )
    logger.info(f"  motion_dryrun_report.md → {report_path}")
    logger.info(f"  primitive_results.jsonl → {os.path.join(log_dir, 'primitive_results.jsonl')}")
    logger.info(f"  motion_failure_cases.jsonl → {os.path.join(log_dir, 'motion_failure_cases.jsonl')}")


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

def print_summary(results: list) -> None:
    _banner("SUMMARY")
    total = len(results)
    success = sum(1 for r in results if r.status == "success")
    logger.info(f"  Total: {total}  Success: {success}  Failure: {total - success}")
    logger.info(f"  Success rate: {success/total*100:.1f}%" if total > 0 else "  No results")
    for r in results:
        tag = "✓" if r.status == "success" else "✗"
        reason = f"  ({r.failure_reason})" if r.failure_reason else ""
        logger.info(f"  {tag} {r.object_id}{reason}  retry={r.retry_count}  dur={r.duration_s:.2f}s")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(plans_path: str, log_dir: str) -> None:
    logger.info(f"[motion_debug] HRC2026 Task 1 Role 3 – Dry-Run")
    logger.info(f"[motion_debug] plans={plans_path}  log_dir={log_dir}")
    logger.info(f"[motion_debug] Input source: mock input")

    os.makedirs(log_dir, exist_ok=True)
    plans = _load_plans(plans_path)
    params = DEFAULT_PARAMS

    run_precondition_checks(plans, params)
    waypoints_per_object = run_waypoint_generation(plans, params, log_dir)
    run_gripper_tests(log_dir)
    results, step_durations = run_full_pipeline(plans, params, log_dir)
    write_reports(results, waypoints_per_object, step_durations, log_dir)
    print_summary(results)

    logger.info(f"\n[motion_debug] Done. All outputs in: {log_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HRC2026 Task 1 – Motion Dry-Run Debug Script")
    parser.add_argument(
        "--plans",
        default=os.path.join(_HERE, "mock_action_plans.json"),
        help="Path to action plans JSON",
    )
    parser.add_argument(
        "--log-dir",
        default=_HERE,
        help="Output directory for all generated files",
    )
    args = parser.parse_args()
    main(plans_path=args.plans, log_dir=args.log_dir)
