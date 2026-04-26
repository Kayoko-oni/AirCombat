"""检查地图中建筑投影到网格后的覆盖与重叠情况。

输出内容：
- 建筑数量、网格尺寸
- 被建筑覆盖的格子数、重叠格子数与最大重叠深度
- 每栋建筑占用的格子数的统计（min/avg/max）
- 检查每栋建筑中心点在网格上是否被标记为占据（期望 True）

用法：
    python tools/map_grid_check.py
"""
from pathlib import Path
import sys
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from utils.map_loader import generate_buildings
from utils.map_grid import MapGrid
import numpy as np


def main():
    buildings = generate_buildings()
    map_grid = MapGrid(buildings, cell_size=5.0)
    w, h = map_grid.width, map_grid.height
    counts = np.zeros((w, h), dtype=int)
    per_building_counts = []
    centers_ok = []

    for idx, (center, size) in enumerate(buildings):
        cx, cy, cz = center
        sx, sy, sz = size
        left = cx - sx/2.0
        right = cx + sx/2.0
        bottom = cy - sy/2.0
        top = cy + sy/2.0
        gx_min, gy_min = map_grid.world_to_grid(left, bottom)
        gx_max, gy_max = map_grid.world_to_grid(right, top)
        cnt = 0
        for gx in range(gx_min, gx_max+1):
            for gy in range(gy_min, gy_max+1):
                if 0 <= gx < w and 0 <= gy < h:
                    counts[gx, gy] += 1
                    cnt += 1
        per_building_counts.append(cnt)
        centers_ok.append(map_grid.is_occupied(cx, cy))

    total_covered = np.sum(counts > 0)
    overlap_cells = np.sum(counts > 1)
    max_overlap = int(np.max(counts))
    avg_bld_cells = float(np.mean(per_building_counts)) if per_building_counts else 0.0

    print(f"Buildings: {len(buildings)}")
    print(f"Grid size: {w}x{h}, total cells: {w*h}")
    print(f"Total occupied cells (by at least one building): {total_covered}")
    print(f"Overlap cells (covered by >1 building): {overlap_cells}, max overlap depth: {max_overlap}")
    print(f"Per-building cells (min/avg/max): {min(per_building_counts)}/{avg_bld_cells:.1f}/{max(per_building_counts)}")

    bad_centers = [i for i, ok in enumerate(centers_ok) if not ok]
    if bad_centers:
        print(f"Warning: {len(bad_centers)} building centers are NOT marked occupied (indices sample): {bad_centers[:5]}")
    else:
        print("All building centers are marked occupied on the grid.")

    # 简单的方向检查：抽样几个空白点和占据点，打印其 grid 索引
    sample_occupied = np.argwhere(counts > 0)
    sample_free = np.argwhere(counts == 0)
    print("Sample occupied cell (gx,gy)->world:")
    if sample_occupied.size:
        gx, gy = sample_occupied[0]
        x, y = map_grid.grid_to_world(int(gx), int(gy))
        print(f"  {int(gx)},{int(gy)} -> {x:.2f},{y:.2f}")
    print("Sample free cell (gx,gy)->world:")
    if sample_free.size:
        gx, gy = sample_free[0]
        x, y = map_grid.grid_to_world(int(gx), int(gy))
        print(f"  {int(gx)},{int(gy)} -> {x:.2f},{y:.2f}")

if __name__ == '__main__':
    main()
