from __future__ import annotations

import argparse
import contextlib
import copy
import io
import json
import random
import sys
import time
from dataclasses import fields, replace
from datetime import datetime
from pathlib import Path
from statistics import mean
from typing import Any, Dict, List, Optional

import yaml

# 保证运行该脚本时，库正常import
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Controller.collision_handler import detect_collisions, resolve_collisions, is_offensive_type
from Controller.single_control import chase_point, chase_target, move_drone
from base.base_manager import BaseManager
from algorithms.auction_assign import ImprovedAuctionConfig, greedy_assignment
from algorithms.cbs_pathplan import cbs_plan_paths
from utils.map_loader import generate_buildings
from utils.map_grid import MapGrid
from drones.factory import (
    create_attack_drone,
    create_anti_ew_drone,
    create_anti_radiation_drone,
    create_drone_team,
    create_ew_drone,
    create_interceptor_drone,
    create_scout_drone,
    create_tank_drone,
)
import numpy as np


def load_config(config_path: Path) -> Dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def is_offensive(drone: Any) -> bool:
    """使用统一的 offensive 类型判断，与 main.py 保持一致。"""
    return is_offensive_type(getattr(drone, "drone_type", ""))


def _distance_3d(a: Any, b: Any) -> float:
    pa = getattr(a, "position", a)
    pb = getattr(b, "position", b)
    return sum((x - y) ** 2 for x, y in zip(pa[:3], pb[:3])) ** 0.5


def _fallback_assignment(defensive: List[Any], offensive: List[Any]) -> Dict[str, Optional[Any]]:
    assignment: Dict[str, Optional[Any]] = {d.name: None for d in defensive}
    if not offensive:
        return assignment
    remaining = set(range(len(offensive)))
    for defender in defensive:
        if not remaining:
            break
        best_idx = min(remaining, key=lambda idx: _distance_3d(defender, offensive[idx]))
        assignment[defender.name] = offensive[best_idx]
        remaining.remove(best_idx)
    return assignment


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


def spawn_random_drone(
    config: Dict[str, Any], drones: List[Any],
    max_offensive: int = 80,
    ard_prob: float = 0.15, ew_prob: float = 0.10,
) -> None:
    """随机生成进攻方（含 4 种机型：Attack/Tank/ARD/EW）。"""
    offensive = [d for d in drones if is_offensive(d) and d.is_alive()]
    if len(offensive) >= max_offensive:
        return

    base_pos = config.get("base", {}).get("position", [0.0, 0.0, 0.0])

    for _ in range(random.randint(1, 3)):
        roll = random.random()
        position = [random.uniform(-450, 450), random.uniform(-400, 400), 100]
        if roll < ard_prob:
            name = f"AntiRad-{random.randint(1000, 9999)}"
            create_anti_radiation_drone(name, position, config, drones)
        elif roll < ard_prob + ew_prob:
            name = f"EW-{random.randint(1000, 9999)}"
            create_ew_drone(name, position, config, drones)
        else:
            drone_type = random.choice(["attack", "tank"])
            name = f"{drone_type.capitalize()}-{random.randint(1000, 9999)}"
            if drone_type == "attack":
                create_attack_drone(name, position, config, drones)
            else:
                create_tank_drone(name, position, config, drones)


def balance_defenders(config: Dict[str, Any], drones: List[Any], anti_ew_prob: float = 0.15) -> None:
    """平衡防守方数量（Scout/Interceptor/AntiEW 三种）。"""
    offensive = [d for d in drones if is_offensive(d) and d.is_alive()]
    defensive = [d for d in drones if not is_offensive(d) and d.is_alive()]
    need = max(0, len(offensive) - len(defensive))
    if need <= 0:
        return

    base_pos = config.get("base", {}).get("position", [0.0, 0.0, 0.0])

    for _ in range(need):
        if random.random() < anti_ew_prob:
            name = f"AntiEW-{random.randint(1000, 9999)}"
            create_anti_ew_drone(name, base_pos, config, drones)
        else:
            drone_type = random.choice(["scout", "interceptor"])
            name = f"{drone_type.capitalize()}-{random.randint(1000, 9999)}"
            if drone_type == "scout":
                create_scout_drone(name, base_pos, config, drones)
            else:
                create_interceptor_drone(name, base_pos, config, drones)


def bootstrap_large_swarm(
    config: Dict[str, Any],
    drones: List[Any],
    offensive_count: int,
    defensive_count: int,
) -> None:
    """初始化大规模蜂群（7 种机型混编）。

    进攻方配比: 40% Attack, 30% Tank, 18% ARD, 12% EW
    防守方配比: 40% Scout, 40% Interceptor, 20% AntiEW
    """
    existing_off = sum(1 for d in drones if is_offensive(d) and d.is_alive())
    existing_def = sum(1 for d in drones if not is_offensive(d) and d.is_alive())

    add_off = max(0, offensive_count - existing_off)
    add_def = max(0, defensive_count - existing_def)

    # 进攻方：按比例分配 4 种机型
    off_types: list = []
    for i in range(add_off):
        ratio = i / max(add_off, 1)
        if ratio < 0.40:
            off_types.append("attack")
        elif ratio < 0.70:
            off_types.append("tank")
        elif ratio < 0.88:
            off_types.append("ard")
        else:
            off_types.append("ew")

    for idx, dtype in enumerate(off_types):
        x = random.uniform(-480, -100)
        y = random.uniform(-420, 420)
        z = random.uniform(60, 140)
        if dtype == "attack":
            create_attack_drone(f"Attack-B-{idx:03d}", [x, y, z], config, drones)
        elif dtype == "tank":
            create_tank_drone(f"Tank-B-{idx:03d}", [x, y, z], config, drones)
        elif dtype == "ard":
            create_anti_radiation_drone(f"AntiRad-B-{idx:03d}", [x, y, z], config, drones)
        elif dtype == "ew":
            create_ew_drone(f"EW-B-{idx:03d}", [x, y, z], config, drones)

    # 防守方：按比例分配 3 种机型
    def_types: list = []
    for i in range(add_def):
        ratio = i / max(add_def, 1)
        if ratio < 0.40:
            def_types.append("scout")
        elif ratio < 0.80:
            def_types.append("interceptor")
        else:
            def_types.append("anti_ew")

    for idx, dtype in enumerate(def_types):
        x = random.uniform(-80, 80)
        y = random.uniform(-80, 80)
        z = random.uniform(20, 70)
        if dtype == "scout":
            create_scout_drone(f"Scout-B-{idx:03d}", [x, y, z], config, drones)
        elif dtype == "interceptor":
            create_interceptor_drone(f"Interceptor-B-{idx:03d}", [x, y, z], config, drones)
        elif dtype == "anti_ew":
            create_anti_ew_drone(f"AntiEW-B-{idx:03d}", [x, y, z], config, drones)


def update_strategy_and_measure(
    alive_drones: List[Any],
    base_position: List[float],
    assignment_cfg: ImprovedAuctionConfig,
    map_grid: Optional[MapGrid],
    cbs_eval_max_agents: int,
    enable_pathplan: bool,
    prev_assignment: Optional[Dict[str, Optional[str]]] = None,
    ew_disabled_set: Optional[set] = None,
) -> Dict[str, Any]:
    """执行一帧追逐/分配策略并测量性能。

    新增 EW/ARD/AntiEW 行为与多维指标，保持最小化改动。
    """
    offensive = [d for d in alive_drones if is_offensive(d) and d.is_alive()]
    defensive = [d for d in alive_drones if not is_offensive(d) and d.is_alive()]

    # ── EW 干扰检测 ──
    ew_active = 0
    defenders_jammed = 0
    ew_disabled_this_frame = 0
    jammed_mask = np.zeros(len(defensive), dtype=bool)

    # 收集活跃 EW + AntiEW
    ew_drones = [
        d for d in offensive
        if d.drone_type == "EWDrone" and getattr(d, "jamming_active", False) and d.is_alive()
    ]
    anti_ew_drones = [
        d for d in defensive
        if d.drone_type == "AntiEWDrone" and d.is_alive()
    ]

    ew_active = len(ew_drones)

    # EW 干扰：标记范围内的防守方（AntiEW 免疫）
    if ew_drones:
        for i, defender in enumerate(defensive):
            if defender.drone_type == "AntiEWDrone":
                continue
            for ew in ew_drones:
                jamming_radius = getattr(ew, "jamming_radius", 200.0)
                dist = sum((p - q) ** 2 for p, q in zip(defender.position, ew.position)) ** 0.5
                if dist <= jamming_radius:
                    jammed_mask[i] = True
                    defenders_jammed += 1
                    break

    # AntiEW 关停 EW：靠近则永久关闭干扰
    for aew in anti_ew_drones:
        disruption_radius = getattr(aew, "disruption_radius", 80.0)
        for ew in ew_drones:
            if not getattr(ew, "jamming_active", False):
                continue
            dist = sum((p - q) ** 2 for p, q in zip(aew.position, ew.position)) ** 0.5
            if dist <= disruption_radius:
                ew.disable_jamming()
                ew_disabled_this_frame += 1

    # ── ARD 自毁检测 ──
    ard_self_destructs = 0
    for d in offensive:
        if d.drone_type == "AntiRadiationDrone" and d.is_alive():
            dist_to_base = sum((p - q) ** 2 for p, q in zip(d.position, base_position)) ** 0.5
            if dist_to_base <= getattr(d, "self_destruct_range", 80.0):
                d.health = 0  # 触发自毁
                ard_self_destructs += 1

    # ── 进攻方追逐基地 ──
    for drone in offensive:
        chase_point(drone, base_position, map_grid=map_grid)

    # ── 分配 ──
    assignment_cost_ms = 0.0
    pathplan_cost_ms = 0.0
    pathplan_calls = 0
    pathplan_agents = 0
    pathplan_success = 0
    assigned_pairs = 0
    reassignments = 0

    assignment: Dict[str, Optional[Any]] = {}

    if offensive and defensive:
        t0 = time.perf_counter()
        try:
            assignment = greedy_assignment(
                defensive,
                offensive,
                config=assignment_cfg,
                map_grid=map_grid,
                jammed_mask=jammed_mask if jammed_mask.any() else None,
            )
        except RuntimeError as exc:
            if "did not converge" in str(exc):
                retry_cfg = replace(
                    assignment_cfg,
                    max_iterations_per_phase=int(assignment_cfg.max_iterations_per_phase * 3),
                )
                try:
                    assignment = greedy_assignment(
                        defensive, offensive, config=retry_cfg, map_grid=map_grid,
                        jammed_mask=jammed_mask if jammed_mask.any() else None,
                    )
                except RuntimeError:
                    assignment = _fallback_assignment(defensive, offensive)
            else:
                raise
        assignment_cost_ms = (time.perf_counter() - t0) * 1000.0

        # 统计重分配率
        if prev_assignment is not None:
            for def_name, target in assignment.items():
                prev_target = prev_assignment.get(def_name)
                cur_target = target.name if target is not None else None
                if prev_target is not None and cur_target is not None and prev_target != cur_target:
                    reassignments += 1

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
                capped_inputs, map_grid=map_grid, max_agents=max(cbs_eval_max_agents, 1),
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

    # ── 被干扰防守方减速 ──
    for i, defender in enumerate(defensive):
        if jammed_mask[i]:
            vel = getattr(defender, "velocity", [0.0, 0.0, 0.0])
            defender.set_velocity([v * 0.3 for v in vel])

    unassigned = sum(1 for v in assignment.values() if v is None)
    total_defenders = len(defensive) if defensive else 1

    return {
        "assignment_ms": assignment_cost_ms,
        "pathplan_ms": pathplan_cost_ms,
        "pathplan_calls": float(pathplan_calls),
        "pathplan_agents": float(pathplan_agents),
        "pathplan_success": float(pathplan_success),
        "assigned_pairs": float(assigned_pairs),
        # 新增指标
        "ew_active": float(ew_active),
        "defenders_jammed": float(defenders_jammed),
        "ew_disabled": float(ew_disabled_this_frame),
        "ard_self_destructs": float(ard_self_destructs),
        "reassignments": float(reassignments),
        "unassigned_ratio": unassigned / max(total_defenders, 1),
        "_assignment": assignment,
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

    # 新增跟踪
    ew_active_samples: List[float] = []
    defenders_jammed_samples: List[float] = []
    ew_disabled_total = 0.0
    ard_self_destructs_total = 0.0
    reassignment_total = 0.0
    unassigned_samples: List[float] = []

    prev_assignment: Dict[str, Optional[str]] = {}

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
                prev_assignment=prev_assignment,
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

            # 记录新增指标
            ew_active_samples.append(strategy_metrics["ew_active"])
            defenders_jammed_samples.append(strategy_metrics["defenders_jammed"])
            ew_disabled_total += strategy_metrics["ew_disabled"]
            ard_self_destructs_total += strategy_metrics["ard_self_destructs"]
            reassignment_total += strategy_metrics["reassignments"]
            unassigned_samples.append(strategy_metrics["unassigned_ratio"])

            # 更新上一帧分配记录（用于下一帧比较重分配）
            prev_assignment = {
                def_name: target.name if target is not None else None
                for def_name, target in (strategy_metrics.get("_assignment", {})).items()
            }

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
        # 新增指标
        "ew_active_avg": mean(ew_active_samples) if ew_active_samples else 0.0,
        "defenders_jammed_avg": mean(defenders_jammed_samples) if defenders_jammed_samples else 0.0,
        "ew_disabled_total": ew_disabled_total,
        "ard_self_destructs_total": ard_self_destructs_total,
        "reassignment_rate": (
            reassignment_total / max(total_assigned_pairs, 1.0)
        ),
        "unassigned_defender_ratio": mean(unassigned_samples) if unassigned_samples else 0.0,
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
        "ew_active_avg",
        "defenders_jammed_avg",
        "ew_disabled_total",
        "ard_self_destructs_total",
        "reassignment_rate",
        "unassigned_defender_ratio",
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
        f"path_success={result['pathplan_success_rate']*100.0:.1f}%, "
        f"reassign={result['reassignment_rate']*100.0:.1f}%"
    )
    print(
        f"       EW: active={result['ew_active_avg']:.1f} jammed={result['defenders_jammed_avg']:.1f} "
        f"disabled={result['ew_disabled_total']:.0f} ARD={result['ard_self_destructs_total']:.0f} "
        f"unassigned={result['unassigned_defender_ratio']*100.0:.1f}%"
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
    print(
        "ew_active_avg(avg/min/max)="
        f"{summary['ew_active_avg_avg']:.2f}/"
        f"{summary['ew_active_avg_min']:.2f}/"
        f"{summary['ew_active_avg_max']:.2f}"
    )
    print(
        "defenders_jammed_avg(avg/min/max)="
        f"{summary['defenders_jammed_avg_avg']:.2f}/"
        f"{summary['defenders_jammed_avg_min']:.2f}/"
        f"{summary['defenders_jammed_avg_max']:.2f}"
    )
    print(
        "reassignment_rate(avg/min/max)="
        f"{summary['reassignment_rate_avg']*100.0:.2f}%/"
        f"{summary['reassignment_rate_min']*100.0:.2f}%/"
        f"{summary['reassignment_rate_max']*100.0:.2f}%"
    )
    print(
        "unassigned_defender_ratio(avg/min/max)="
        f"{summary['unassigned_defender_ratio_avg']*100.0:.2f}%/"
        f"{summary['unassigned_defender_ratio_min']*100.0:.2f}%/"
        f"{summary['unassigned_defender_ratio_max']*100.0:.2f}%"
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
    parser.add_argument("--duration", type=float, default=120.0, help="Simulation time per run (seconds).")
    parser.add_argument("--fps", type=int, default=30, help="Simulation FPS.")
    parser.add_argument("--runs", type=int, default=10, help="How many episodes to evaluate.")
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
