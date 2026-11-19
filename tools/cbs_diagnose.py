"""CBS 回归/诊断脚本（不依赖 open3d）

运行：
    python tools/cbs_diagnose.py

用确定性的"城市街区"地图检查：
- 同格起点、同格终点、相邻格交换、并行车道、7/12 架超限
- 返回路径是否真的无顶点/边冲突
- 不同规模下的耗时与部分解情况
- a_star_plan_world / line_of_sight_world 的基本可用性
"""
import itertools
import random
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import algorithms.cbs_pathplan as cbs
from utils.map_grid import MapGrid


# ---------------------------------------------------------------- map
def city_buildings(step=50.0, block=24.0, half=400.0, h=60.0):
    buildings = []
    x = -half
    while x <= half:
        y = -half
        while y <= half:
            if not (abs(x) < 60 and abs(y) < 60):  # 原点附近留作基地空地
                buildings.append(((x, y, h / 2.0), (block, block, h)))
            y += step
        x += step
    return buildings


MAP = MapGrid(city_buildings(), bounds=(-500, -500, 500, 500), cell_size=5.0)
FREE = [(gx, gy) for gx in range(0, MAP.width, 4) for gy in range(0, MAP.height, 4)
        if not MAP.is_grid_occupied(gx, gy)]


def w(gx, gy, z=30.0):
    x, y = MAP.grid_to_world(gx, gy)
    return [x, y, z]


# ---------------------------------------------------------------- instrumentation
_orig_astar = cbs._a_star_on_grid
CALLS = {"n": 0, "fail": 0}


def counting_astar(*a, **kw):
    CALLS["n"] += 1
    r = _orig_astar(*a, **kw)
    if not r:
        CALLS["fail"] += 1
    return r


cbs._a_star_on_grid = counting_astar


# ---------------------------------------------------------------- checks
def analyse_world_paths(paths):
    """世界坐标 -> 网格后统计冲突。空路径（部分解）会被跳过。"""
    valid = [(i, p) for i, p in enumerate(paths) if p]
    grid = {i: [MAP.world_to_grid(q[0], q[1]) for q in p] for i, p in valid}
    idx = [i for i, _ in valid]
    vertex, edge = [], []
    t_max = max((len(p) for _, p in valid), default=0)
    for t in range(t_max):
        if t > 0:  # t=0 同格起点按设计允许
            seen = {}
            for i in idx:
                p = grid[i]
                c = p[t] if t < len(p) else p[-1]
                if c in seen:
                    vertex.append((seen[c], i, c, t))
                else:
                    seen[c] = i
        if t + 1 >= t_max:
            continue
        for i, j in itertools.combinations(idx, 2):
            pi, pj = grid[i], grid[j]
            a0 = pi[t] if t < len(pi) else pi[-1]
            a1 = pi[t + 1] if t + 1 < len(pi) else pi[-1]
            b0 = pj[t] if t < len(pj) else pj[-1]
            b1 = pj[t + 1] if t + 1 < len(pj) else pj[-1]
            if a0 == b1 and b0 == a1 and a0 != a1:
                edge.append((i, j, a0, b1, t))
    return vertex, edge, grid


def min_pairwise_distance(paths, skip_t0=True):
    """路径间最小水平间距；默认跳过 t=0（同格起点是允许的）。"""
    valid = [p for p in paths if p]
    best = float("inf")
    t_max = max((len(p) for p in valid), default=0)
    for i, j in itertools.combinations(range(len(valid)), 2):
        for t in range(1 if skip_t0 else 0, t_max):
            a = valid[i][t] if t < len(valid[i]) else valid[i][-1]
            b = valid[j][t] if t < len(valid[j]) else valid[j][-1]
            d = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
            best = min(best, d)
    return best


RESULTS = []


def run(name, pairs, time_limit_ms=200, expect_valid=None):
    CALLS["n"] = CALLS["fail"] = 0
    t0 = time.perf_counter()
    paths = cbs.cbs_plan_paths(pairs, map_grid=MAP, time_limit_ms=time_limit_ms)
    dt = (time.perf_counter() - t0) * 1000
    returned = len(paths)
    valid = [p for p in paths if p]
    vertex, edge, _ = analyse_world_paths(paths) if paths else ([], [], {})
    min_d = min_pairwise_distance(paths) if len(valid) >= 2 else float("inf")
    if expect_valid is None:
        expect_valid = len(pairs)
    ok = (returned == len(pairs) and len(valid) == expect_valid
          and not vertex and not edge)
    RESULTS.append((name, ok))
    print(f"\n[{name}] agents={len(pairs)} elapsed={dt:.1f}ms "
          f"astar_calls={CALLS['n']} (fail={CALLS['fail']})")
    print(f"   returned={returned}/{len(pairs)} valid={len(valid)}/{expect_valid} "
          f"vertex_conflicts={len(vertex)} edge_conflicts={len(edge)} "
          f"min_dist={min_d:.2f}m  -> {'PASS' if ok else 'FAIL'}")
    return dt, paths


def main():
    print(f"MapGrid {MAP.width}x{MAP.height} cell={MAP.cell_size}m "
          f"free_samples={len(FREE)}")
    random.seed(7)

    # 0) 单机工具函数
    a, b = FREE[0], FREE[200]
    single = cbs.a_star_plan_world(w(*a), w(*b), MAP, max_time_ms=100)
    los = cbs.line_of_sight_world(tuple(w(*a)), tuple(w(*b)), MAP)
    print(f"\n[a_star_plan_world] len={len(single)}  line_of_sight={los}")

    # 1) 常规 6 机
    picks = random.sample(FREE, 12)
    run("1 常规6机", [(w(*picks[i]), w(*picks[i + 6])) for i in range(6)])

    # 2) 同格起点（补充防守方从同一基地出生）
    start = w(*picks[0])
    run("2 同格起点4机", [(start, w(*picks[1 + i])) for i in range(4)])

    # 3) 真正相邻的两个空格，交换目标
    pair = None
    for gx, gy in FREE:
        for nx, ny in ((gx + 1, gy), (gx, gy + 1)):
            if not MAP.is_grid_occupied(nx, ny):
                pair = ((gx, gy), (nx, ny))
                break
        if pair:
            break
    a, b = pair
    run("3 相邻格交换", [(w(*a), w(*b)), (w(*b), w(*a))])

    # 4) 相邻车道并行：网格无冲突，但物理间距 5m（规划层不含安全半径）
    gx0, gy0 = a
    run("4 相邻车道并行", [(w(gx0, gy0), w(gx0, gy0 + 30)),
                          (w(gx0 + 1, gy0), w(gx0 + 1, gy0 + 30))])

    # 5) 超过 max_agents：应规划前 6 架，其余给 []
    picks = random.sample(FREE, 28)
    dt, paths = run("5 七架超限", [(w(*picks[i]), w(*picks[i + 14])) for i in range(7)],
                    expect_valid=6)
    print(f"   -> 前6架有路径={sum(1 for p in paths[:6] if p)}, 第7架为空={not paths[6] if len(paths) > 6 else 'N/A'}")

    # 6) 同格终点：去重后应全部有解
    goal = w(*picks[0])
    run("6 同格终点3机", [(w(*picks[1 + i]), goal) for i in range(3)])

    # 7) 默认预算（200ms）下的同格起点：旧版会卡满 2s
    start = w(*FREE[0])
    run("7 同格起点/默认预算", [(start, w(*FREE[100 + i])) for i in range(3)])

    # 8) 密集随机 6 机 x15
    random.seed(11)
    times = []
    fails = 0
    for _ in range(15):
        picks = random.sample(FREE, 12)
        pairs = [(w(*picks[i]), w(*picks[i + 6])) for i in range(6)]
        CALLS["n"] = CALLS["fail"] = 0
        t0 = time.perf_counter()
        paths = cbs.cbs_plan_paths(pairs, map_grid=MAP, time_limit_ms=300)
        times.append((time.perf_counter() - t0) * 1000)
        v, e, _ = analyse_world_paths(paths) if paths else ([], [], {})
        if len(paths) != 6 or any(not p for p in paths) or v or e:
            fails += 1
    print(f"\n[8 密集随机6机x15] fail={fails}/15 avg={sum(times)/len(times):.1f}ms "
          f"max={max(times):.1f}ms -> {'PASS' if fails == 0 else 'FAIL'}")
    RESULTS.append(("8 密集随机6机x15", fails == 0))

    # 9) 12 架规模：允许部分解，但不应死循环
    picks = random.sample(FREE, 24)
    pairs = [(w(*picks[i]), w(*picks[i + 12])) for i in range(12)]
    CALLS["n"] = CALLS["fail"] = 0
    t0 = time.perf_counter()
    paths = cbs.cbs_plan_paths(pairs, map_grid=MAP, max_agents=12, time_limit_ms=300)
    dt = (time.perf_counter() - t0) * 1000
    valid = [p for p in paths if p]
    v, e, _ = analyse_world_paths(paths) if paths else ([], [], {})
    ok = bool(paths) and not v and not e
    RESULTS.append(("9 12架/部分解", ok))
    print(f"\n[9 12架] elapsed={dt:.1f}ms valid={len(valid)}/12 "
          f"vertex={len(v)} edge={len(e)} -> {'PASS' if ok else 'FAIL'}")

    # 10) 平滑开关：平滑后仍不应产生冲突
    orig_smooth = cbs._smooth_grid_path
    random.seed(23)
    broken = 0
    for _ in range(30):
        picks = random.sample(FREE, 12)
        pairs = [(w(*picks[i]), w(*picks[i + 6])) for i in range(6)]
        cbs._smooth_grid_path = lambda p, m: list(p)
        raw = cbs.cbs_plan_paths(pairs, map_grid=MAP)
        cbs._smooth_grid_path = orig_smooth
        sm = cbs.cbs_plan_paths(pairs, map_grid=MAP)
        if raw and sm:
            v0, e0, _ = analyse_world_paths(raw)
            v1, e1, _ = analyse_world_paths(sm)
            if (v0 + e0) == 0 and (v1 + e1) > 0:
                broken += 1
    RESULTS.append(("10 平滑不破坏无冲突", broken == 0))
    print(f"\n[10 平滑影响] 平滑后新出现冲突的实例={broken}/30 -> "
          f"{'PASS' if broken == 0 else 'FAIL'}")

    # 11) 带时间戳轨迹：点数=时间数、t0=0、严格递增
    random.seed(31)
    picks = random.sample(FREE, 12)
    pairs = [(w(*picks[i]), w(*picks[i + 6])) for i in range(6)]
    tick = 0.4
    schedules = cbs.cbs_plan_schedules(pairs, map_grid=MAP, tick_seconds=tick)
    ok = len(schedules) == 6
    for item in schedules:
        if not item or len(item[0]) != len(item[1]) or not item[1]:
            ok = False
            break
        times = item[1]
        if abs(times[0]) > 1e-9 or any(times[i] >= times[i + 1] for i in range(len(times) - 1)):
            ok = False
            break
        if len(times) > 1 and abs(times[1] - tick) > 1e-9:
            ok = False
            break
    RESULTS.append(("11 带时间戳轨迹", ok))
    print(f"\n[11 带时间戳轨迹] schedules={len(schedules)} "
          f"points/times={[len(s[0]) if s else 0 for s in schedules]} -> "
          f"{'PASS' if ok else 'FAIL'}")

    print("\n=== SUMMARY ===")
    for name, ok in RESULTS:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")


if __name__ == "__main__":
    main()
