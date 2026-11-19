"""攻防对应关系追踪 / 量化（无界面控制台输出）

用法：
    python tools/battle_trace.py --duration 30 --max-offensive 30 --spawn 0.5
    python tools/battle_trace.py --focus Scout-123

输出三类行：
    SUMMARY,t,attackers,visible,jammers,defenders,assigned,pair_min_dist,base_health
    TRACE,t,name,x,y,z,vx,vy,vz,target,tx,ty,tz,dist,follower_idx,follower_len,jammed
    COLLISION/BASE 事件
"""
import argparse
import math
import random
import sys
import time
from collections import Counter
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import main
from base.base_manager import BaseManager
from drones.offensive.anti_radiation_drone import AntiRadiationDrone


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class BattleTrace:
    def __init__(self, focus_name=None, trace_interval=0.5):
        self.focus_name = focus_name
        self.focus = None
        self.trace_interval = trace_interval
        self.sim_time = 0.0
        self.last_trace = -1e9
        self.prev_targets = {}
        self.target_switches = 0
        self.pair_min = float("inf")
        self.pair_sum = 0.0
        self.pair_n = 0
        self.collisions = 0
        self.collision_events = []
        self.base_health = None
        self.base_min = None
        self.visible_samples = []
        self.assigned_samples = []
        self.attacker_samples = []
        self.defender_samples = []
        self.max_attackers = 0
        self.max_defenders = 0
        self.dead_seen = set()
        self.death_causes = Counter()
        self.base_hits = 0
        self.focus_min = float("inf")
        self.focus_switches = 0
        self.focus_prev = None
        self.last_counts = None

    def install(self):
        from drones.base_drone import BaseDrone

        orig_apply = BaseDrone.apply_damage
        def apply_damage(drone_self, damage):
            was_alive = drone_self.is_alive()
            r = orig_apply(drone_self, damage)
            if (was_alive and not drone_self.is_alive()
                    and id(drone_self) not in self.dead_seen):
                self.dead_seen.add(id(drone_self))
                role = "ATTACK" if main.is_offensive_type(drone_self.drone_type) else "DEFEND"
                cause = getattr(drone_self, "death_cause", None) or "unknown"
                self.death_causes[(role, cause)] += 1
                print(f"DEATH,{self.sim_time:.2f},{drone_self.name},"
                      f"{drone_self.drone_type},{role},{cause}")
            return r
        BaseDrone.apply_damage = apply_damage

        orig_strategy = main.update_chase_strategy
        def strategy(drones, offensive_targets=None, *a, **k):
            r = orig_strategy(drones, offensive_targets, *a, **k)
            self.after_strategy(drones, k)
            return r
        main.update_chase_strategy = strategy

        orig_follow = main.update_path_followers
        def follow(drones, sim_time):
            r = orig_follow(drones, sim_time)
            self.trace_focus(drones, sim_time)
            return r
        main.update_path_followers = follow

        orig_resolve = main.resolve_collisions
        def resolve(collisions, *a, **k):
            for a_drone, b_drone in collisions:
                if main.is_offensive_type(a_drone.drone_type) != main.is_offensive_type(b_drone.drone_type):
                    self.collisions += 1
                    self.collision_events.append((self.sim_time, a_drone.name, b_drone.name))
                    print(f"COLLISION,{self.sim_time:.2f},{a_drone.name},{b_drone.name},"
                          f"{a_drone.drone_type},{b_drone.drone_type}")
            return orig_resolve(collisions, *a, **k)
        main.resolve_collisions = resolve

        orig_base = BaseManager.check_collisions
        def base_check(bm, drones):
            before = bm.get_health()
            r = orig_base(bm, drones)
            after = bm.get_health()
            if after < before:
                self.base_hits += 1
            if self.base_health is None or after != self.base_health:
                self.base_health = after
                self.base_min = after if self.base_min is None else min(self.base_min, after)
                print(f"BASE,{self.sim_time:.2f},{before:.1f}->{after:.1f}")
            return r
        BaseManager.check_collisions = base_check

    def after_strategy(self, drones, kw):
        self.sim_time = float(kw.get("sim_time") or self.sim_time)
        attackers = [d for d in drones if main.is_offensive_type(d.drone_type) and d.is_alive()]
        defenders = [d for d in drones if not main.is_offensive_type(d.drone_type) and d.is_alive()]
        jammers = main._get_jammers(drones)
        base_positions = kw.get("base_positions") or []
        visible = [
            a for a in attackers
            if not isinstance(a, AntiRadiationDrone)
            and main._is_visible_to_defense(a, base_positions, jammers, defenders=defenders)
        ]
        assigned = [d for d in defenders if getattr(d, "_assigned_target", None) is not None]
        self.visible_samples.append(len(visible))
        self.assigned_samples.append(len(assigned))
        self.attacker_samples.append(len(attackers))
        self.defender_samples.append(len(defenders))
        self.max_attackers = max(self.max_attackers, len(attackers))
        self.max_defenders = max(self.max_defenders, len(defenders))
        self.last_counts = (len(attackers), len(defenders))

        for d in drones:
            if not d.destroyed or id(d) in self.dead_seen:
                continue
            self.dead_seen.add(id(d))
            role = "ATTACK" if main.is_offensive_type(d.drone_type) else "DEFEND"
            cause = getattr(d, "death_cause", None) or "unknown"
            self.death_causes[(role, cause)] += 1
            print(f"DEATH,{self.sim_time:.2f},{d.name},{d.drone_type},{role},{cause}")

        for d in defenders:
            tgt = getattr(d, "_assigned_target", None)
            name = getattr(tgt, "name", None)
            prev = self.prev_targets.get(d.name, "__init__")
            if prev != "__init__" and name is not None and name != prev:
                self.target_switches += 1
            self.prev_targets[d.name] = name

        pair_min = float("inf")
        for d in assigned:
            tgt = getattr(d, "_assigned_target", None)
            if tgt is not None and getattr(tgt, "is_alive", lambda: False)():
                dd = dist(d.position, tgt.position)
                pair_min = min(pair_min, dd)
                self.pair_min = min(self.pair_min, dd)
                self.pair_sum += dd
                self.pair_n += 1
        pair_text = f"{pair_min:.1f}" if pair_min != float("inf") else "-1"
        base_text = f"{self.base_health:.1f}" if self.base_health is not None else "-1"
        print(f"SUMMARY,{self.sim_time:.2f},{len(attackers)},{len(visible)},{len(jammers)},"
              f"{len(defenders)},{len(assigned)},{pair_text},{base_text}")

    def trace_focus(self, drones, sim_time):
        if sim_time - self.last_trace < self.trace_interval:
            return
        if self.focus is None:
            if self.focus_name:
                self.focus = next((d for d in drones if d.name == self.focus_name), None)
            if self.focus is None:
                self.focus = next(
                    (d for d in drones
                     if not main.is_offensive_type(d.drone_type) and d.is_alive()), None)
        if self.focus is None or self.focus.destroyed:
            return

        d = self.focus
        tgt = getattr(d, "_assigned_target", None)
        tp = tgt.position if tgt is not None else [0.0, 0.0, 0.0]
        dd = dist(d.position, tgt.position) if tgt is not None else -1.0
        if dd >= 0:
            self.focus_min = min(self.focus_min, dd)
        name = getattr(tgt, "name", None)
        if self.focus_prev is not None and name is not None and name != self.focus_prev:
            self.focus_switches += 1
        self.focus_prev = name

        f = getattr(d, "_path_follower", None)
        self.last_trace = sim_time
        print(f"TRACE,{sim_time:.2f},{d.name},{d.position[0]:.2f},{d.position[1]:.2f},{d.position[2]:.2f},"
              f"{d.velocity[0]:.2f},{d.velocity[1]:.2f},{d.velocity[2]:.2f},"
              f"{name},{tp[0]:.1f},{tp[1]:.1f},{tp[2]:.1f},{dd:.1f},"
              f"{getattr(f, 'idx', -1) if f is not None else -1},"
              f"{len(getattr(f, 'points', [])) if f is not None else -1},"
              f"{int(getattr(d, '_ew_jammed', False))}")

    def summary(self):
        avg_visible = sum(self.visible_samples) / max(1, len(self.visible_samples))
        avg_assigned = sum(self.assigned_samples) / max(1, len(self.assigned_samples))
        avg_attackers = sum(self.attacker_samples) / max(1, len(self.attacker_samples))
        avg_defenders = sum(self.defender_samples) / max(1, len(self.defender_samples))
        avg_pair = self.pair_sum / max(1, self.pair_n)
        coverage_visible = avg_assigned / max(1e-9, avg_visible)
        coverage_defenders = avg_assigned / max(1e-9, avg_defenders)
        print("=== BATTLE SUMMARY ===")
        print(f"samples={len(self.visible_samples)} avg_visible={avg_visible:.2f} "
              f"avg_assigned={avg_assigned:.2f} avg_attackers={avg_attackers:.2f} "
              f"avg_defenders={avg_defenders:.2f} coverage_visible={coverage_visible:.2f} "
              f"coverage_defenders={coverage_defenders:.2f} target_switches={self.target_switches} "
              f"collisions={self.collisions} base_hits={self.base_hits} "
              f"base_health={self.base_health} base_min={self.base_min} "
              f"pair_dist_avg={avg_pair:.1f} pair_dist_min={self.pair_min:.1f}")
        print(f"max_attackers_defenders=({self.max_attackers},{self.max_defenders}) "
              f"final_attackers_defenders={self.last_counts}")
        print(f"deaths={dict(self.death_causes)}")
        print(f"focus={getattr(self.focus, 'name', None)} focus_min_dist={self.focus_min:.1f} "
              f"focus_switches={self.focus_switches}")


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--max-offensive", type=int, default=30)
    parser.add_argument("--spawn", type=float, default=0.5)
    parser.add_argument("--plan-speed", type=float, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--focus", type=str, default="")
    parser.add_argument("--config", type=str, default=str(PROJECT_ROOT / "config.yaml"))
    args = parser.parse_args()

    config = main.load_config(Path(args.config))
    config["simulation"]["duration"] = args.duration
    config["simulation"]["max_offensive"] = args.max_offensive
    config["simulation"]["spawn_interval_min"] = args.spawn
    config["simulation"]["spawn_interval_max"] = args.spawn
    config["simulation"]["enable_visualization"] = False
    if args.plan_speed is not None:
        config.setdefault("path_planning", {})["plan_speed"] = args.plan_speed
    if args.seed is not None:
        random.seed(args.seed)

    trace = BattleTrace(args.focus or None)
    trace.install()
    t0 = time.time()
    try:
        main.run_simulation(config)
    finally:
        print(f"WALL,{time.time() - t0:.1f}")
        trace.summary()


if __name__ == "__main__":
    run()
