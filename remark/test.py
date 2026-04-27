from __future__ import annotations

import argparse
import contextlib
import copy
import io
import json
import random
import sys
import time
from dataclasses import fields
from datetime import datetime
from pathlib import Path
from statistics import mean
from typing import Any, Dict, List, Optional

import yaml

# 保证运行该脚本时，库正常import
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Controller.collision_handler import detect_collisions, resolve_collisions
from Controller.single_control import chase_point, chase_target, move_drone
from base.base_manager import BaseManager
from algorithms.auction_assign import ImprovedAuctionConfig, greedy_assignment
from algorithms.cbs_pathplan import cbs_plan_paths
from utils.map_loader import generate_buildings
from utils.map_grid import MapGrid
from drones.factory import (
    create_attack_drone,
    create_drone_team,
    create_interceptor_drone,
    create_scout_drone,
    create_tank_drone,
)


def load_config(config_path: Path) -> Dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def is_offensive(drone: Any) -> bool:
    return getattr(drone, "drone_type", "") in {"AttackDrone", "TankDrone"}


def build_assignment_config(config: Dict[str, Any]) -> ImprovedAuctionConfig:
    raw = config.get("assignment", {})
    if not isinstance(raw, dict):
        return ImprovedAuctionConfig()
    valid_keys = {item.name for item in fields(ImprovedAuctionConfig)}
    kwargs = {key: raw[key] for key in valid_keys if key in raw}
    return ImprovedAuctionConfig(**kwargs)


def build_map_grid(config: Dict[str, Any]) -> MapGrid:
    """Build obstacle map data in the same style as main simulation."""
    buildings = generate_buildings(config=config)
    return MapGrid(buildings, cell_size=5.0)


def spawn_random_drone(config: Dict[str, Any], drones: List[Any], max_offensive: int = 80) -> None:
    offensive = [d for d in drones if is_offensive(d) and d.is_alive()]
    if len(offensive) >= max_offensive:
        return

    for _ in range(random.randint(1, 3)):
        drone_type = random.choice(["attack", "tank"])
        position = [random.uniform(-450, 450), random.uniform(-400, 400), 100]
        name = f"{drone_type.capitalize()}-{random.randint(1000, 9999)}"
        if drone_type == "attack":
            create_attack_drone(name, position, config, drones)
        else:
            create_tank_drone(name, position, config, drones)


def balance_defenders(config: Dict[str, Any], drones: List[Any]) -> None:
    offensive = [d for d in drones if is_offensive(d) and d.is_alive()]
    defensive = [d for d in drones if not is_offensive(d) and d.is_alive()]
    need = max(0, len(offensive) - len(defensive))
    if need <= 0:
        return

    for _ in range(need):
        drone_type = random.choice(["scout", "interceptor"])
        position = [0.0, 0.0, 0.0]
        name = f"{drone_type.capitalize()}-{random.randint(1000, 9999)}"
        if drone_type == "scout":
            create_scout_drone(name, position, config, drones)
        else:
            create_interceptor_drone(name, position, config, drones)


def bootstrap_large_swarm(
    config: Dict[str, Any],
    drones: List[Any],
    offensive_count: int,
    defensive_count: int,
) -> None:
    existing_off = sum(1 for d in drones if is_offensive(d))
    existing_def = sum(1 for d in drones if not is_offensive(d))

    add_off = max(0, offensive_count - existing_off)
    add_def = max(0, defensive_count - existing_def)

    for idx in range(add_off):
        is_attack = (idx % 2 == 0)
        x = random.uniform(-480, -100)
        y = random.uniform(-420, 420)
        z = random.uniform(60, 140)
        if is_attack:
            create_attack_drone(f"Attack-B-{idx:03d}", [x, y, z], config, drones)
        else:
            create_tank_drone(f"Tank-B-{idx:03d}", [x, y, z], config, drones)

    for idx in range(add_def):
        is_scout = (idx % 2 == 0)
        x = random.uniform(-80, 80)
        y = random.uniform(-80, 80)
        z = random.uniform(20, 70)
        if is_scout:
            create_scout_drone(f"Scout-B-{idx:03d}", [x, y, z], config, drones)
        else:
            create_interceptor_drone(f"Interceptor-B-{idx:03d}", [x, y, z], config, drones)


def update_strategy_and_measure(
    alive_drones: List[Any],
    base_position: List[float],
    assignment_cfg: ImprovedAuctionConfig,
    map_grid: Optional[MapGrid],
    cbs_eval_max_agents: int,
    enable_pathplan: bool,
) -> Dict[str, float]:
    offensive = [d for d in alive_drones if is_offensive(d) and d.is_alive()]
    defensive = [d for d in alive_drones if not is_offensive(d) and d.is_alive()]

    for drone in offensive:
        chase_point(drone, base_position, map_grid=map_grid)

    assignment_cost_ms = 0.0
    pathplan_cost_ms = 0.0
    pathplan_calls = 0
    pathplan_agents = 0
    pathplan_success = 0
    assigned_pairs = 0
    if offensive and defensive:
        t0 = time.perf_counter()
        assignment = greedy_assignment(defensive, offensive, config=assignment_cfg)
        assignment_cost_ms = (time.perf_counter() - t0) * 1000.0

        defenders_by_name = {d.name: d for d in defensive}
        plan_inputs = []
        defenders_order = []

        for def_name, target in assignment.items():
            defender = defenders_by_name.get(def_name)
            if defender is None:
                continue
            if target is None:
                defender.set_velocity([0.0, 0.0, 0.0])
                continue

            assigned_pairs += 1
            goal = target.position if hasattr(target, "position") else None
            if goal is None:
                chase_target(defender, target, map_grid=map_grid)
                continue
            plan_inputs.append((defender, goal))
            defenders_order.append(defender)

        if map_grid is not None and plan_inputs and cbs_eval_max_agents > 0 and enable_pathplan:
            capped_inputs = plan_inputs[:cbs_eval_max_agents]
            capped_defenders = defenders_order[:cbs_eval_max_agents]
            fallback_defenders = defenders_order[cbs_eval_max_agents:]

            pathplan_calls = 1
            pathplan_agents = len(capped_inputs)
            t1 = time.perf_counter()
            world_paths = cbs_plan_paths(
                capped_inputs,
                map_grid=map_grid,
                max_agents=max(cbs_eval_max_agents, 1),
            )
            pathplan_cost_ms = (time.perf_counter() - t1) * 1000.0

            if world_paths:
                for defender, path in zip(capped_defenders, world_paths):
                    if path and len(path) >= 2:
                        chase_point(defender, path[1])
                        pathplan_success += 1
                    elif path and len(path) == 1:
                        chase_point(defender, path[0])
                        pathplan_success += 1
                    else:
                        target = assignment.get(defender.name)
                        if target is not None:
                            chase_target(defender, target, map_grid=map_grid)
            else:
                for defender in capped_defenders:
                    target = assignment.get(defender.name)
                    if target is not None:
                        chase_target(defender, target, map_grid=map_grid)

            for defender in fallback_defenders:
                target = assignment.get(defender.name)
                if target is not None:
                    chase_target(defender, target, map_grid=map_grid)
        else:
            for defender in defenders_order:
                target = assignment.get(defender.name)
                if target is not None:
                    chase_target(defender, target, map_grid=map_grid)
    else:
        for defender in defensive:
            defender.set_velocity([0.0, 0.0, 0.0])

    return {
        "assignment_ms": assignment_cost_ms,
        "pathplan_ms": pathplan_cost_ms,
        "pathplan_calls": float(pathplan_calls),
        "pathplan_agents": float(pathplan_agents),
        "pathplan_success": float(pathplan_success),
        "assigned_pairs": float(assigned_pairs),
    }


def evaluate_once(
    config: Dict[str, Any],
    duration_s: float,
    fps: int,
    enable_spawn: bool,
    seed: Optional[int],
    offensive_count: int,
    defensive_count: int,
    silent_events: bool,
    cbs_eval_max_agents: int,
    strategy_interval_frames: int,
    pathplan_interval_frames: int,
) -> Dict[str, Any]:
    if seed is not None:
        random.seed(seed)

    drones = create_drone_team(config)
    bootstrap_large_swarm(config, drones, offensive_count, defensive_count)
    base_manager = BaseManager(config)
    assignment_cfg = build_assignment_config(config)
    map_grid = build_map_grid(config)

    frame_time = 1.0 / max(fps, 1)
    max_frames = int(duration_s * fps) if duration_s > 0 else 3000

    spawn_timer = 0.0
    next_spawn_time = random.uniform(1.0, 3.0)

    attacker_destroyed_names = set()
    defender_destroyed_names = set()

    assignment_costs_ms: List[float] = []
    pathplan_costs_ms: List[float] = []
    assignment_call_count = 0
    pathplan_call_count = 0
    pathplan_total_agents = 0.0
    pathplan_total_success = 0.0
    total_assigned_pairs = 0.0

    episode_start = time.perf_counter()

    simulated_seconds = 0.0
    strategy_interval_frames = max(1, int(strategy_interval_frames))
    pathplan_interval_frames = max(1, int(pathplan_interval_frames))

    for frame_idx in range(max_frames):
        if base_manager.is_destroyed():
            break

        collisions = detect_collisions(drones)
        if collisions:
            resolve_collisions(collisions)

        for drone in drones:
            drone.update_battery(frame_time)

        for drone in drones:
            if drone.destroyed:
                if is_offensive(drone):
                    attacker_destroyed_names.add(drone.name)
                else:
                    defender_destroyed_names.add(drone.name)

                if drone.falling:
                    drone.update_fall(frame_time)
                elif drone.impact:
                    drone.update_death_timer(frame_time)

        drones = [d for d in drones if not d.should_remove()]
        alive_drones = [d for d in drones if not d.destroyed]
        if not alive_drones:
            break

        if frame_idx % strategy_interval_frames == 0:
            strategy_metrics = update_strategy_and_measure(
                alive_drones=alive_drones,
                base_position=config["base"]["position"],
                assignment_cfg=assignment_cfg,
                map_grid=map_grid,
                cbs_eval_max_agents=cbs_eval_max_agents,
                enable_pathplan=(frame_idx % pathplan_interval_frames == 0),
            )
            if strategy_metrics["assignment_ms"] > 0:
                assignment_call_count += 1
                assignment_costs_ms.append(strategy_metrics["assignment_ms"])
            if strategy_metrics["pathplan_calls"] > 0:
                pathplan_call_count += int(strategy_metrics["pathplan_calls"])
                pathplan_costs_ms.append(strategy_metrics["pathplan_ms"])
                pathplan_total_agents += strategy_metrics["pathplan_agents"]
                pathplan_total_success += strategy_metrics["pathplan_success"]
            total_assigned_pairs += strategy_metrics["assigned_pairs"]

        for drone in alive_drones:
            move_drone(drone, frame_time)

        if silent_events:
            with contextlib.redirect_stdout(io.StringIO()):
                base_manager.check_collisions(drones)
        else:
            base_manager.check_collisions(drones)

        if enable_spawn:
            spawn_timer += frame_time
            if spawn_timer >= next_spawn_time:
                spawn_timer = 0.0
                next_spawn_time = random.uniform(1.0, 3.0)
                if random.random() < 0.9:
                    spawn_random_drone(config, drones)
            balance_defenders(config, drones)

        simulated_seconds += frame_time

    runtime_s = time.perf_counter() - episode_start

    alive_attackers = sum(1 for d in drones if is_offensive(d) and d.is_alive())
    alive_defenders = sum(1 for d in drones if (not is_offensive(d)) and d.is_alive())

    result = {
        "runtime_seconds": runtime_s,
        "simulated_seconds": simulated_seconds,
        "base_health": base_manager.get_health(),
        "base_health_ratio": (
            base_manager.get_health() / max(base_manager.get_max_health(), 1e-9)
        ),
        "attackers_shot_down": len(attacker_destroyed_names),
        "defenders_lost": len(defender_destroyed_names),
        "alive_attackers": alive_attackers,
        "alive_defenders": alive_defenders,
        "assignment_calls": assignment_call_count,
        "assignment_avg_ms": mean(assignment_costs_ms) if assignment_costs_ms else 0.0,
        "assignment_max_ms": max(assignment_costs_ms) if assignment_costs_ms else 0.0,
        "pathplan_calls": pathplan_call_count,
        "pathplan_avg_ms": mean(pathplan_costs_ms) if pathplan_costs_ms else 0.0,
        "pathplan_max_ms": max(pathplan_costs_ms) if pathplan_costs_ms else 0.0,
        "pathplan_avg_agents": (
            pathplan_total_agents / max(pathplan_call_count, 1)
        ),
        "pathplan_success_rate": (
            pathplan_total_success / max(pathplan_total_agents, 1.0)
        ),
        "avg_assigned_pairs_per_frame": (
            total_assigned_pairs / max(pathplan_call_count, 1)
        ),
    }

    if base_manager.is_destroyed():
        result["winner"] = "attackers"
    elif result["alive_attackers"] == 0:
        result["winner"] = "defenders"
    else:
        result["winner"] = "undecided"

    return result


def aggregate_results(runs: List[Dict[str, Any]]) -> Dict[str, Any]:
    numeric_keys = [
        "runtime_seconds",
        "simulated_seconds",
        "base_health",
        "base_health_ratio",
        "attackers_shot_down",
        "defenders_lost",
        "alive_attackers",
        "alive_defenders",
        "assignment_calls",
        "assignment_avg_ms",
        "assignment_max_ms",
        "pathplan_calls",
        "pathplan_avg_ms",
        "pathplan_max_ms",
        "pathplan_avg_agents",
        "pathplan_success_rate",
        "avg_assigned_pairs_per_frame",
    ]

    summary = {
        "runs": len(runs),
        "winners": {
            "attackers": sum(1 for r in runs if r["winner"] == "attackers"),
            "defenders": sum(1 for r in runs if r["winner"] == "defenders"),
            "undecided": sum(1 for r in runs if r["winner"] == "undecided"),
        },
    }

    for key in numeric_keys:
        values = [float(r[key]) for r in runs]
        summary[f"{key}_avg"] = mean(values)
        summary[f"{key}_min"] = min(values)
        summary[f"{key}_max"] = max(values)

    return summary


def print_run(idx: int, result: Dict[str, Any]) -> None:
    print(
        f"[Run {idx}] winner={result['winner']}, runtime={result['runtime_seconds']:.3f}s, "
        f"base={result['base_health']:.1f}, shot_down={result['attackers_shot_down']}, "
        f"lost={result['defenders_lost']}, assign_avg={result['assignment_avg_ms']:.4f}ms, "
        f"path_avg={result['pathplan_avg_ms']:.4f}ms, "
        f"path_success={result['pathplan_success_rate']*100.0:.1f}%"
    )


def run_evaluation(
    *,
    duration: float,
    fps: int,
    runs: int,
    seed: int,
    enable_spawn: bool,
    config: str,
    output: str,
    show_runs: bool,
    offensive_count: int,
    defensive_count: int,
    silent_events: bool,
    cbs_eval_max_agents: int,
    strategy_interval_frames: int,
    pathplan_interval_frames: int,
) -> Path:
    config_path = Path(config).resolve()
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    base_config = load_config(config_path)

    all_results = []
    for i in range(runs):
        run_config = copy.deepcopy(base_config)
        run_seed = seed + i
        print(f"\nRunning episode {i + 1}/{runs} ...")
        result = evaluate_once(
            config=run_config,
            duration_s=duration,
            fps=fps,
            enable_spawn=enable_spawn,
            seed=run_seed,
            offensive_count=offensive_count,
            defensive_count=defensive_count,
            silent_events=silent_events,
            cbs_eval_max_agents=cbs_eval_max_agents,
            strategy_interval_frames=strategy_interval_frames,
            pathplan_interval_frames=pathplan_interval_frames,
        )
        all_results.append(result)
        if show_runs or runs > 1:
            print_run(i + 1, result)

    summary = aggregate_results(all_results)

    print("\n=== Aggregate Summary ===")
    print(f"runs={summary['runs']}, winners={summary['winners']}")
    print(
        "base_health(avg/min/max)="
        f"{summary['base_health_avg']:.2f}/"
        f"{summary['base_health_min']:.2f}/"
        f"{summary['base_health_max']:.2f}"
    )
    print(
        "attackers_shot_down(avg/min/max)="
        f"{summary['attackers_shot_down_avg']:.2f}/"
        f"{summary['attackers_shot_down_min']:.2f}/"
        f"{summary['attackers_shot_down_max']:.2f}"
    )
    print(
        "defenders_lost(avg/min/max)="
        f"{summary['defenders_lost_avg']:.2f}/"
        f"{summary['defenders_lost_min']:.2f}/"
        f"{summary['defenders_lost_max']:.2f}"
    )
    print(
        "assignment_avg_ms(avg/min/max)="
        f"{summary['assignment_avg_ms_avg']:.4f}/"
        f"{summary['assignment_avg_ms_min']:.4f}/"
        f"{summary['assignment_avg_ms_max']:.4f}"
    )
    print(
        "pathplan_avg_ms(avg/min/max)="
        f"{summary['pathplan_avg_ms_avg']:.4f}/"
        f"{summary['pathplan_avg_ms_min']:.4f}/"
        f"{summary['pathplan_avg_ms_max']:.4f}"
    )
    print(
        "pathplan_success_rate(avg/min/max)="
        f"{summary['pathplan_success_rate_avg']*100.0:.2f}%/"
        f"{summary['pathplan_success_rate_min']*100.0:.2f}%/"
        f"{summary['pathplan_success_rate_max']*100.0:.2f}%"
    )

    output_path = Path(output).resolve() if output else (
        PROJECT_ROOT
        / "remark"
        / f"eval_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )

    report = {
        "meta": {
            "date": datetime.now().isoformat(timespec="seconds"),
            "config": str(config_path),
            "duration_s": duration,
            "fps": fps,
            "runs": runs,
            "seed": seed,
            "enable_spawn": enable_spawn,
            "offensive_count": offensive_count,
            "defensive_count": defensive_count,
            "map_grid_cell_size": 5.0,
            "cbs_eval_max_agents": cbs_eval_max_agents,
            "strategy_interval_frames": strategy_interval_frames,
            "pathplan_interval_frames": pathplan_interval_frames,
        },
        "summary": summary,
        "runs": all_results,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nJSON report saved: {output_path}")
    return output_path


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description="Evaluate assignment and path-planning algorithm performance in AirCombat."
    )
    parser.add_argument("--duration", type=float, default=180.0, help="Simulation time per run (seconds).")
    parser.add_argument("--fps", type=int, default=30, help="Simulation FPS.")
    parser.add_argument("--runs", type=int, default=5, help="How many episodes to evaluate.")
    parser.add_argument("--seed", type=int, default=42, help="Base random seed.")
    parser.add_argument("--offensive-count", type=int, default=90, help="Initial attacker count.")
    parser.add_argument("--defensive-count", type=int, default=90, help="Initial defender count.")
    parser.add_argument("--show-runs", action="store_true", help="Print per-run detail lines.")
    parser.add_argument(
        "--show-events",
        action="store_true",
        help="Show internal event prints, including base hit logs.",
    )
    parser.add_argument(
        "--enable-spawn",
        action="store_true",
        help="Enable random attacker spawning and defender balancing.",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=str(PROJECT_ROOT / "config.yaml"),
        help="Path to config.yaml",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="Optional output JSON path. Defaults to remark/eval_report_<timestamp>.json",
    )
    parser.add_argument(
        "--cbs-eval-max-agents",
        type=int,
        default=6,
        help="How many assigned defenders are included in each frame's CBS benchmark.",
    )
    parser.add_argument(
        "--strategy-interval-frames",
        type=int,
        default=2,
        help="Run assignment/strategy update every N frames.",
    )
    parser.add_argument(
        "--pathplan-interval-frames",
        type=int,
        default=4,
        help="Run CBS path planning every N frames (must be >= 1).",
    )

    args = parser.parse_args(argv)

    run_evaluation(
        duration=args.duration,
        fps=args.fps,
        runs=args.runs,
        seed=args.seed,
        enable_spawn=args.enable_spawn,
        config=args.config,
        output=args.output,
        show_runs=args.show_runs,
        offensive_count=args.offensive_count,
        defensive_count=args.defensive_count,
        silent_events=(not args.show_events),
        cbs_eval_max_agents=max(0, args.cbs_eval_max_agents),
        strategy_interval_frames=max(1, args.strategy_interval_frames),
        pathplan_interval_frames=max(1, args.pathplan_interval_frames),
    )


if __name__ == "__main__":
    main()
