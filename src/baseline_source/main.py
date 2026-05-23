"""
main.py – HRC2026 Task 1 Role 3
================================
Entry point để chạy Task 1 pick-place trong Isaac Sim workspace.

Cách dùng (từ repo root):
    python src/task1/main.py
    python src/task1/main.py --plans tests/mock_action_plans_task1.json
    python src/task1/main.py --headless

Hoặc paste vào cuối main.py của workspace (sau robot.initialize()):
    from src.task1.task1_runner import run_task1
    run_task1(robot, world)
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

# ── Path setup ────────────────────────────────────────────────────────────────
_HERE = Path(__file__).parent.resolve()       # src/task1/
_REPO_ROOT = _HERE.parent.parent.resolve()    # repo root

if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

# ── Default paths ─────────────────────────────────────────────────────────────
DEFAULT_PLANS_PATH = str(_REPO_ROOT / "tests" / "mock_action_plans_task1.json")
DEFAULT_URDF_PATH  = "/home/ubuntu/hrc2026_workspace/assets/resources/s2.urdf"
DEFAULT_TASK_CFG   = str(_REPO_ROOT / "configs" / "Part_Sorting.yaml")
DEFAULT_LOG_DIR    = str(_REPO_ROOT / "logs" / "task1")
DEFAULT_PRIM_PATH  = "/Root/Ref_Xform/Ref"


# ── Helpers ───────────────────────────────────────────────────────────────────

def _print_summary(results: list) -> None:
    total   = len(results)
    success = sum(1 for r in results if r.status == "success")
    logger.info("")
    logger.info("=" * 60)
    logger.info("  TASK 1 SUMMARY")
    logger.info("=" * 60)
    logger.info(f"  Total: {total}  Success: {success}  Failure: {total - success}")
    if total > 0:
        logger.info(f"  Success rate: {success / total * 100:.1f}%")
    for r in results:
        tag    = "✓" if r.status == "success" else "✗"
        reason = f"  ({r.failure_reason})" if r.failure_reason else ""
        logger.info(f"  {tag} {r.object_id}{reason}  retry={r.retry_count}  dur={r.duration_s:.2f}s")
    logger.info("=" * 60)


# ── Isaac Sim bootstrap ───────────────────────────────────────────────────────

def _init_isaac_sim(task_cfg_path: str, headless: bool, prim_path: str, urdf_path: str):
    """Launch Isaac Sim, load Part_Sorting scene, initialize robot.

    Returns (kit, world, robot, data_logger).
    """
    import yaml

    # 1. SimulationApp – must be first Isaac import
    from isaacsim import SimulationApp
    logger.info("[main] Step 1: Creating SimulationApp (headless=%s)...", headless)
    kit = SimulationApp({"headless": headless, "width": 1280, "height": 720})
    logger.info("[main] SimulationApp ready")

    # 2. Load scene USD – resolve root_path to absolute
    import omni.usd as omni_usd
    with open(task_cfg_path, "r") as f:
        task_cfg = yaml.safe_load(f)

    # root_path in yaml may be relative; resolve against known assets location
    ASSETS_ROOT = "/home/ubuntu/hrc2026_workspace/assets/resources"
    raw_root = task_cfg.get("root_path", "")
    if not os.path.isabs(raw_root) or not os.path.isdir(raw_root):
        task_cfg["root_path"] = ASSETS_ROOT
        logger.info("[main] root_path overridden to: %s", ASSETS_ROOT)

    scene_usd = os.path.join(task_cfg["root_path"], task_cfg.get("scene_usd", ""))
    if not os.path.isfile(scene_usd):
        raise FileNotFoundError(f"Scene USD not found: {scene_usd}")

    logger.info("[main] Step 2: Loading scene USD: %s", scene_usd)
    omni_usd.get_context().open_stage(scene_usd)
    logger.info("[main] Scene USD loaded")

    # 3. World
    from isaacsim.core.api import World
    logger.info("[main] Step 3: Creating World...")
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 20.0,
    )
    world.initialize_physics()
    logger.info("[main] World initialized")

    # 4. SceneBuilder
    logger.info("[main] Step 4: Building scene...")
    src_path = str(_REPO_ROOT / "src")
    if src_path not in sys.path:
        sys.path.insert(0, src_path)

    from baseline_source.SceneBuilder import SceneBuilder
    from baseline_source.DataLogger import DataLogger

    data_logger = DataLogger(enabled=False, csv_path="", camera_enabled=False, camera_hdf5_path="")
    scene = SceneBuilder(task_cfg, data_logger=data_logger)
    scene.build_all()
    scene.build_robot()
    logger.info("[main] Scene built")

    # 5. Play + warmup
    world.play()
    for _ in range(10):
        world.step(render=False)
    logger.info("[main] Physics warmed up (10 steps)")

    # 6. Robot interface
    logger.info("[main] Step 5: Initializing robot interface...")
    try:
        from baseline_source.isaac_sim_robot_interface import IsaacSimRobotInterface
    except ImportError:
        from isaac_sim_robot_interface import IsaacSimRobotInterface

    robot = IsaacSimRobotInterface(
        prim_path=prim_path,
        name="walkerS2",
        world=world,
        urdf_path=urdf_path,
    )
    robot.initialize()
    logger.info("[main] Robot interface ready")

    return kit, world, robot, data_logger


# ── Main ──────────────────────────────────────────────────────────────────────

def main(
    plans_path: str = DEFAULT_PLANS_PATH,
    task_cfg_path: str = DEFAULT_TASK_CFG,
    urdf_path: str = DEFAULT_URDF_PATH,
    log_dir: str = DEFAULT_LOG_DIR,
    prim_path: str = DEFAULT_PRIM_PATH,
    headless: bool = False,
    side: str = "right",
) -> list:
    """Run Task 1 pick-place in Isaac Sim. Returns list of PrimitiveResult."""
    logger.info("[main] HRC2026 Task 1 Role 3")
    logger.info("[main] plans=%s", plans_path)
    logger.info("[main] task_cfg=%s", task_cfg_path)

    os.makedirs(log_dir, exist_ok=True)

    kit, world, robot, data_logger = _init_isaac_sim(
        task_cfg_path=task_cfg_path,
        headless=headless,
        prim_path=prim_path,
        urdf_path=urdf_path,
    )

    results = []
    try:
        # Run Task 1 pick-place – pass IsaacSimRobotInterface directly to RealMotionInterface
        # (bypasses RobotArticulationAdapter which expects RobotArticulation, not IsaacSimRobotInterface)
        from real_motion_interface import RealMotionInterface
        from motion import MotionPrimitiveRunner
        from motion_utils import DEFAULT_PARAMS
        import json

        interface = RealMotionInterface(
            robot_interface=robot,
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

        if not os.path.isfile(plans_path):
            logger.error("[main] plans not found: %s", plans_path)
            return []

        with open(plans_path, encoding="utf-8") as f:
            plans = json.load(f)

        logger.info("[main] Running %d plans...", len(plans))
        for plan in plans:
            result = runner.pick_place(plan)
            results.append(result)
            tag = "SUCCESS" if result.status == "success" else f"FAIL({result.failure_reason})"
            logger.info("  %s: %s  dur=%.2fs", result.object_id, tag, result.duration_s)

        _print_summary(results)

        # Keep sim open until window is closed
        logger.info("[main] Task done. Running sim loop (close window to exit)...")
        while kit.is_running():
            world.step(render=True)

    except KeyboardInterrupt:
        logger.info("[main] Interrupted by user")
    finally:
        data_logger.close()
        try:
            kit.close()
        except Exception:
            pass
        logger.info("[main] Shutdown complete")

    return results


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HRC2026 Task 1 – Isaac Sim runner")
    parser.add_argument(
        "--plans",
        default=DEFAULT_PLANS_PATH,
        help="Path to action plans JSON (default: tests/mock_action_plans_task1.json)",
    )
    parser.add_argument(
        "--task-cfg",
        default=DEFAULT_TASK_CFG,
        help="Path to Part_Sorting.yaml (default: Ubtech_sim/config/Part_Sorting.yaml)",
    )
    parser.add_argument(
        "--urdf",
        default=DEFAULT_URDF_PATH,
        help="Path to s2.urdf for IK solver (default: assets/resources/s2.urdf)",
    )
    parser.add_argument(
        "--log-dir",
        default=DEFAULT_LOG_DIR,
        help="Output directory for logs (default: logs/task1/)",
    )
    parser.add_argument(
        "--prim-path",
        default=DEFAULT_PRIM_PATH,
        help="USD prim path of robot (default: /Root/Ref_Xform/Ref)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Run without GUI",
    )
    parser.add_argument(
        "--side",
        default="right",
        choices=["right", "left"],
        help="Arm side for pick-place (default: right)",
    )
    args = parser.parse_args()

    main(
        plans_path=args.plans,
        task_cfg_path=args.task_cfg,
        urdf_path=args.urdf,
        log_dir=args.log_dir,
        prim_path=args.prim_path,
        headless=args.headless,
        side=args.side,
    )
