"""CBS 性能测试脚本

用法示例:
    python tools/cbs_perf_test.py --agents 1 2 4 6 --runs 10

脚本会在当前地图上随机生成 start/goal 对，并对不同 agent 数量调用 cbs_plan_paths，统计平均耗时。
"""
import argparse
import random
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
import sys
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from utils.map_loader import generate_buildings
from utils.map_grid import MapGrid
from algorithms.cbs_pathplan import cbs_plan_paths


def sample_free_positions(map_grid, n):
    free_cells = []
    for gx in range(map_grid.width):
        for gy in range(map_grid.height):
            if not map_grid.is_grid_occupied(gx, gy):
                free_cells.append((gx, gy))
    if not free_cells:
        raise RuntimeError("地图上没有空闲格子可用于测试")
    picks = random.choices(free_cells, k=n)
    # 转为世界坐标，并附加 z=30
    out = []
    for gx, gy in picks:
        x, y = map_grid.grid_to_world(gx, gy)
        out.append([x, y, 30.0])
    return out


def run_one(map_grid, agent_count):
    # 构建 agent_count 个 (start, goal)
    starts = sample_free_positions(map_grid, agent_count)
    goals = sample_free_positions(map_grid, agent_count)
    pairs = list(zip(starts, goals))
    t0 = time.perf_counter()
    paths = cbs_plan_paths(pairs, map_grid=map_grid)
    elapsed = (time.perf_counter() - t0) * 1000.0
    success = bool(paths)
    return elapsed, success


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", type=int, nargs='+', default=[1,2,4,6])
    parser.add_argument("--runs", type=int, default=5)
    args = parser.parse_args()

    buildings = generate_buildings()
    map_grid = MapGrid(buildings, cell_size=5.0)
    print(f"Map grid: {map_grid.width}x{map_grid.height}, agents to test: {args.agents}")

    for n in args.agents:
        times = []
        success_count = 0
        for _ in range(args.runs):
            elapsed, success = run_one(map_grid, n)
            times.append(elapsed)
            success_count += int(success)
        print(f"agents={n}: avg={sum(times)/len(times):.2f}ms, max={max(times):.2f}ms, success_rate={success_count}/{args.runs}")

if __name__ == '__main__':
    main()
