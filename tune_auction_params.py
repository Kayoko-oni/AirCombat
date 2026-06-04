#!/usr/bin/env python3
"""
CMA-ES 自动调参 — 搜索 ImprovedAuctionConfig 最优参数（纯离线，零 GPU）。

用法:
    python tune_auction_params.py                     # 默认: 30 代, 6 核, 每局 40s
    python tune_auction_params.py --generations 40    # 自定义代数
    python tune_auction_params.py --workers 8          # 并行核数
    python tune_auction_params.py --eval-seconds 30    # 每局评估时长
    python tune_auction_params.py --pop-size 10        # CMA-ES 种群大小
    python tune_auction_params.py --resume tune_results/cma_checkpoint  # 断点续调

输出: tune_results/best_params.json
"""

from __future__ import annotations

import argparse, json, os, sys, time, warnings
from copy import deepcopy
from datetime import datetime
from multiprocessing import Pool, cpu_count
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

# 抑制 Open3D 和日志噪音
os.environ["OPEN3D_DISABLED"] = "1"
warnings.filterwarnings("ignore")

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))


# ═══════════════════════════════════════════════════════════════════
# 参数空间：只优化 10 个核心连续参数
# 格式: (归一化下界, 归一化上界, 真实下界, 真实上界)
# ═══════════════════════════════════════════════════════════════════

PARAM_SPACE: List[Tuple[float, float, float, float]] = [
    (0.0, 1.0, 0.20, 0.95),    # attention_temperature
    (0.0, 1.0, 3.0,  6.0),     # top_k_proposers (int)
    (0.0, 1.0, 15.0, 55.0),    # urgency_weight
    (0.0, 1.0, 8.0,  28.0),    # load_balance_weight
    (0.0, 1.0, 6.0,  28.0),    # feasibility_weight
    (0.0, 1.0, 8.0,  35.0),    # attention_weight
    (0.0, 1.0, 20.0, 80.0),    # non_responder_penalty
    (0.0, 1.0, 0.0,  20.0),    # sticky_assignment_bonus
    (0.0, 1.0, 0.0,  15.0),    # reassignment_penalty
    (0.0, 1.0, 180.0,420.0),   # communication_range
]

PARAM_NAMES = [
    "attention_temperature", "top_k_proposers", "urgency_weight",
    "load_balance_weight", "feasibility_weight", "attention_weight",
    "non_responder_penalty", "sticky_assignment_bonus",
    "reassignment_penalty", "communication_range",
]

INTEGER_PARAMS = {"top_k_proposers"}


def norm_to_params(x: np.ndarray) -> Dict[str, float]:
    """[0,1]^n → 真实参数字典。"""
    d: Dict[str, float] = {}
    for i, (nl, nh, rl, rh) in enumerate(PARAM_SPACE):
        t = max(0.0, min(1.0, (float(x[i]) - nl) / (nh - nl)))
        val = rl + t * (rh - rl)
        d[PARAM_NAMES[i]] = max(rl, min(rh, val))
    return d


# ═══════════════════════════════════════════════════════════════════
# 核心：headless 仿真评估器（在子进程中运行）
# ═══════════════════════════════════════════════════════════════════

def _evaluate_one(params_norm: np.ndarray) -> float:
    """跑一局 headless 仿真，返回得分（越高越好）。"""
    import main as sim

    # ── 构建配置 ──
    config = sim.load_config(PROJECT_ROOT / "config.yaml")
    pdict = norm_to_params(params_norm)

    if "assignment" not in config:
        config["assignment"] = {}
    for name, val in pdict.items():
        config["assignment"][name] = int(round(val)) if name in INTEGER_PARAMS else float(val)

    eval_sec = float(os.environ.get("TUNE_EVAL_SECONDS", 40))
    config["simulation"]["duration"] = eval_sec
    config["simulation"]["fps"] = 30
    base_pos = config["base"]["position"]

    # ── 强制 headless ──
    sim.Open3DDisplay = None
    sim._ASSIGNMENT_CONFIG = sim._build_assignment_config(config)

    from base.base_manager import BaseManager
    from Controller.collision_handler import detect_collisions, resolve_collisions
    from Controller.single_control import move_drone

    drones: list = []
    base_mgr = BaseManager(config)

    fps = 30
    ft = 1.0 / fps
    t0 = time.perf_counter()
    deadline = t0 + eval_sec
    spawn_interval = config.get("simulation", {}).get("spawn_interval_min", 1.8)
    next_spawn = time.perf_counter() + np.random.uniform(0.3, 1.0)
    ard_timer = 0.0

    # 用 name 统计击杀，避免 Python id() 复用 bug
    attacker_types = {"AttackDrone", "TankDrone", "AntiRadiationDrone", "EWDrone"}
    defender_types = {"ScoutDrone", "InterceptorDrone", "AntiEWDrone"}
    killed_attackers: set = set()
    killed_defenders: set = set()

    def _count_kills(ds):
        for d in ds:
            if not d.destroyed:
                continue
            name = getattr(d, "name", "")
            if not name:
                continue
            dtype = getattr(d, "drone_type", "")
            if name in killed_attackers or name in killed_defenders:
                continue
            if dtype in attacker_types:
                killed_attackers.add(name)
            elif dtype in defender_types:
                killed_defenders.add(name)

    try:
        while time.perf_counter() < deadline and not base_mgr.is_destroyed():
            # 碰撞
            cols = detect_collisions(drones)
            if cols:
                resolve_collisions(cols)

            # 电量 + 状态更新
            for d in drones:
                d.update_battery(ft)
                if d.destroyed:
                    if d.falling:
                        d.update_fall(ft)
                    elif d.impact:
                        d.update_death_timer(ft)

            # 统计本帧新增的击杀
            _count_kills(drones)

            # 清理已移除的无人机
            drones = [d for d in drones if not d.should_remove()]
            alive = [d for d in drones if not d.destroyed]

            # ARD 自毁
            ard_timer += ft
            if ard_timer >= 0.5:
                ard_timer = 0.0
                from drones.offensive.anti_radiation_drone import AntiRadiationDrone
                for d in alive:
                    if isinstance(d, AntiRadiationDrone) and d.is_alive():
                        d.check_self_destruct(base_pos)

            # 追逐策略（核心：此处使用拍卖算法分配目标）
            sim.update_chase_strategy(
                alive, base_pos, sim._ASSIGNMENT_CONFIG,
                map_grid=None, base_position=base_pos,
            )

            # EW/AntiEW 生成（注意：必须传 drones 而非 alive，
            # 否则 AntiEW 被添加到临时列表，下一帧丢失 → 无限循环生成）
            sim._spawn_anti_ew_if_needed(config, drones, base_pos)

            # 移动
            for d in alive:
                move_drone(d, ft)

            # 基地碰撞
            base_mgr.check_collisions(drones)

            # 进攻方生成
            if time.perf_counter() >= next_spawn:
                next_spawn = time.perf_counter() + np.random.uniform(
                    spawn_interval * 0.7, spawn_interval * 1.3
                )
                if np.random.random() < 0.9:
                    sim.spawn_random_drone(config, drones, base_pos)

            # 防守方平衡
            sim.balance_defenders(config, drones)

    except Exception:
        pass

    # ── 最后一次计数 ──
    _count_kills(drones)

    attacker_deaths = len(killed_attackers)
    defender_deaths = len(killed_defenders)
    base_hp = base_mgr.get_health()

    if base_mgr.is_destroyed():
        score = -40.0 + attacker_deaths * 8.0 - defender_deaths * 8.0
    else:
        score = (
            base_hp * 0.40              # 血量 0-40 分
            + attacker_deaths * 12.0    # 击杀加分
            - defender_deaths * 8.0     # 损失扣分
        )

    # 惩罚"什么都没发生"的参数组合
    if attacker_deaths == 0 and defender_deaths == 0 and base_hp > 95:
        score -= 10.0

    return float(score)


# ═══════════════════════════════════════════════════════════════════
# CMA-ES 主循环
# ═══════════════════════════════════════════════════════════════════

def get_default_params() -> Dict[str, float]:
    from algorithms.auction_assign import ImprovedAuctionConfig
    from dataclasses import fields
    c = ImprovedAuctionConfig()
    return {f.name: getattr(c, f.name) for f in fields(c)}


def save_checkpoint(gen: int, best_params: dict, best_score: float, scores: list, out: Path):
    out.mkdir(parents=True, exist_ok=True)
    d = {
        "generation": gen,
        "best_score": best_score,
        "best_params": {k: round(v, 4) for k, v in best_params.items()},
        "mean_score": round(float(np.mean(scores)), 2),
        "std_score": round(float(np.std(scores)), 2),
        "timestamp": datetime.now().isoformat(),
    }
    (out / f"gen_{gen:04d}.json").write_text(json.dumps(d, indent=2, ensure_ascii=False))


def main():
    p = argparse.ArgumentParser(description="CMA-ES tune auction params")
    p.add_argument("--generations", type=int, default=30)
    p.add_argument("--workers", type=int, default=max(1, cpu_count() - 2))
    p.add_argument("--eval-seconds", type=float, default=40.0)
    p.add_argument("--pop-size", type=int, default=12)
    p.add_argument("--sigma", type=float, default=0.3)
    p.add_argument("--output", type=str, default="tune_results")
    p.add_argument("--resume", type=str, default=None)
    args = p.parse_args()

    os.environ["TUNE_EVAL_SECONDS"] = str(args.eval_seconds)
    out = Path(args.output).resolve()
    ndim = len(PARAM_NAMES)

    print("=" * 60)
    print("  CMA-ES 拍卖参数自动调优")
    print("=" * 60)
    print(f"  参数维度:     {ndim}")
    print(f"  代数:         {args.generations}")
    print(f"  种群大小:     {args.pop_size}")
    print(f"  初始步长:     {args.sigma}")
    print(f"  每局评估:     {args.eval_seconds}s (headless, 无 time.sleep)")
    print(f"  并行进程:     {args.workers}")
    print(f"  预计总仿真:   ~{args.generations * args.pop_size} 局")
    print(f"  结果目录:     {out}")
    print("=" * 60, flush=True)

    # CMA-ES
    import cma

    x0 = np.full(ndim, 0.5)
    if args.resume and Path(args.resume).exists():
        import pickle
        data = pickle.loads(Path(args.resume).read_bytes())
        es = cma.CMAEvolutionStrategy(x0, args.sigma,
            {"popsize": args.pop_size, "bounds": [0.0, 1.0],
             "maxiter": args.generations, "verbose": 1})
        es.feed_for_resume(data)
        print(f"[resume]  从 {args.resume} 恢复，当前第 {es.countiter} 代\n")
    else:
        es = cma.CMAEvolutionStrategy(
            x0, args.sigma,
            {"popsize": args.pop_size, "bounds": [0.0, 1.0],
             "maxiter": args.generations, "verbose": 1},
        )

    pool = Pool(processes=args.workers)
    best_score = -float("inf")
    best_params = {}

    try:
        while not es.stop():
            solutions = es.ask()
            gen = es.countiter

            t0 = time.perf_counter()
            scores = pool.map(_evaluate_one, list(solutions))
            elapsed = time.perf_counter() - t0

            es.tell(solutions, [-s for s in scores])  # CMA 最小化 → 取负

            idx = int(np.argmax(scores))
            gen_best = float(scores[idx])
            gen_best_p = norm_to_params(solutions[idx])

            if gen_best > best_score:
                best_score = gen_best
                best_params = gen_best_p

            print(
                f"[gen {gen:3d}] best={gen_best:7.1f}  "
                f"mean={np.mean(scores):7.1f}  std={np.std(scores):6.1f}  "
                f"time={elapsed:5.1f}s",
                flush=True,
            )

            if gen_best == best_score:
                print(f"          >>> NEW BEST: {best_score:.1f}")
                for k, v in gen_best_p.items():
                    print(f"              {k}: {v:.3f}")

            if gen % 5 == 0 or gen == 1:
                save_checkpoint(gen, best_params, best_score, scores, out)
                import pickle as _pickle
                (out / "cma_checkpoint").write_bytes(_pickle.dumps(es.pickle_dumps()))

    except KeyboardInterrupt:
        print("\n[interrupted]")
    finally:
        pool.close()
        pool.join()

    # 最终保存
    import pickle as _pickle
    (out / "cma_final").write_bytes(_pickle.dumps(es.pickle_dumps()))
    final = {
        "timestamp": datetime.now().isoformat(),
        "generations": es.countiter,
        "evaluations": es.countevals,
        "best_score": round(best_score, 2),
        "best_params": {k: round(v, 4) for k, v in best_params.items()},
        "defaults": {k: round(v, 4) for k, v in get_default_params().items()
                      if k in PARAM_NAMES},
    }
    (out / "best_params.json").write_text(json.dumps(final, indent=2, ensure_ascii=False))

    print("\n" + "=" * 60)
    print("  调参完成！")
    print(f"  最优得分: {best_score:.1f}")
    print(f"  最优参数 → {out / 'best_params.json'}")
    print("=" * 60)
    print("\n  核心参数对比 (默认 → 优化):")
    print("  " + "-" * 55)
    df = get_default_params()
    for n in PARAM_NAMES:
        d = df.get(n, float("nan"))
        o = best_params.get(n, float("nan"))
        a = "↑" if o > d else "↓" if o < d else "="
        print(f"  {n:28s} {d:8.3f} → {o:8.3f}  {a}")


if __name__ == "__main__":
    main()
