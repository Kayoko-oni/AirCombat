"""多 seed 攻防基准：调用 battle_trace，聚合基地/碰撞/分配/切换等指标。

用法：
    python tools/battle_bench.py --seeds 1,2,3 --duration 60
    python tools/battle_bench.py --seed-start 1 --seed-count 5 --duration 30
"""
import argparse
import re
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SUMMARY_FIELDS = [
    "avg_visible", "avg_assigned", "avg_attackers", "avg_defenders",
    "coverage_visible", "coverage_defenders", "target_switches",
    "collisions", "base_hits", "base_health", "pair_dist_min",
    "attack_deaths", "defend_deaths",
]


def run_seed(args, seed):
    cmd = [
        sys.executable, str(ROOT / "tools" / "battle_trace.py"),
        "--duration", str(args.duration),
        "--max-offensive", str(args.max_offensive),
        "--spawn", str(args.spawn),
        "--seed", str(seed),
    ]
    if args.plan_speed is not None:
        cmd += ["--plan-speed", str(args.plan_speed)]
    if args.config:
        cmd += ["--config", str(args.config)]
    proc = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True,
                          encoding="utf-8", errors="replace")
    if proc.returncode != 0:
        print(f"[seed {seed}] FAILED rc={proc.returncode}")
        print((proc.stderr or proc.stdout)[-2000:])
        return None
    for line in proc.stdout.splitlines():
        if not line.startswith("samples="):
            continue
        data = {}
        for m in re.finditer(r"(\w+)=([-\d.]+)", line):
            data[m.group(1)] = float(m.group(2))
        m = re.search(r"final_attackers_defenders=\((\d+), (\d+)\)", proc.stdout)
        if m:
            data["final_attackers"] = float(m.group(1))
            data["final_defenders"] = float(m.group(2))
        death_line = re.search(r"deaths=\{(.*?)\}", proc.stdout)
        if death_line:
            for role, cause, count in re.findall(
                    r"\('(ATTACK|DEFEND)', '([^']+)'\): (\d+)", death_line.group(1)):
                key = f"{role.lower()}_{cause}_deaths"
                data[key] = data.get(key, 0.0) + float(count)
            data["attack_deaths"] = sum(v for k, v in data.items()
                                        if k.startswith("attack_") and k.endswith("_deaths"))
            data["defend_deaths"] = sum(v for k, v in data.items()
                                        if k.startswith("defend_") and k.endswith("_deaths"))
        return data
    print(f"[seed {seed}] no summary found")
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=str, default="1,2,3")
    parser.add_argument("--seed-start", type=int, default=None)
    parser.add_argument("--seed-count", type=int, default=3)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--max-offensive", type=int, default=40)
    parser.add_argument("--spawn", type=float, default=0.8)
    parser.add_argument("--plan-speed", type=float, default=None)
    parser.add_argument("--config", type=str, default=None)
    args = parser.parse_args()
    if args.seed_start is not None:
        seeds = list(range(args.seed_start, args.seed_start + args.seed_count))
    else:
        seeds = [int(s) for s in args.seeds.split(",") if s.strip()]

    rows = []
    for seed in seeds:
        data = run_seed(args, seed)
        if data is None:
            continue
        data["seed"] = seed
        rows.append(data)
        print(f"seed={seed:>3}  base={data.get('base_health', 0):6.1f}  "
              f"collisions={data.get('collisions', 0):5.0f}  "
              f"assigned={data.get('avg_assigned', 0):5.2f}  "
              f"coverage={data.get('coverage_defenders', 0):4.2f}  "
              f"switches={data.get('target_switches', 0):4.0f}  "
              f"pair_min={data.get('pair_dist_min', 0):4.1f}  "
              f"deaths={int(data.get('attack_deaths', 0))}/{int(data.get('defend_deaths', 0))}  "
              f"final={int(data.get('final_attackers', 0))}/{int(data.get('final_defenders', 0))}")

    if not rows:
        print("no successful episodes")
        return 1
    print(f"\n=== AGGREGATE ({len(rows)} episodes, {args.duration:.0f}s) ===")
    header = f"{'metric':<24}{'mean':>10}{'min':>10}{'max':>10}"
    print(header)
    print("-" * len(header))
    for field in SUMMARY_FIELDS:
        vals = [r[field] for r in rows if field in r]
        if not vals:
            continue
        print(f"{field:<24}{statistics.mean(vals):>10.2f}"
              f"{min(vals):>10.2f}{max(vals):>10.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
