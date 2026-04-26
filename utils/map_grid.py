#将地图数据转换为二维数组，供路径规划部分调用此处是否可以通行，是工具类函数
import numpy as np
from typing import List, Tuple, Union

class MapGrid:
    def __init__(self, buildings: List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]], 
                 bounds: Tuple[float, float, float, float] = (-500, -500, 500, 500), 
                 cell_size: float = 5.0):
        """
        buildings: 建筑列表，每个元素为 ((cx, cy, cz), (sx, sy, sz))
        bounds: 地图边界 (min_x, min_y, max_x, max_y)
        cell_size: 网格单元边长（米）
        """
        self.min_x, self.min_y, self.max_x, self.max_y = bounds
        self.cell_size = cell_size
        self.width = int((self.max_x - self.min_x) / cell_size) + 1
        self.height = int((self.max_y - self.min_y) / cell_size) + 1
        self.obstacle_grid = np.zeros((self.width, self.height), dtype=bool)

        # 标记每个建筑覆盖的网格
        for (cx, cy, _), (sx, sy, _) in buildings:
            half_x = sx / 2.0
            half_y = sy / 2.0
            left = cx - half_x
            right = cx + half_x
            bottom = cy - half_y
            top = cy + half_y
            gx_min, gy_min = self.world_to_grid(left, bottom)
            gx_max, gy_max = self.world_to_grid(right, top)
            for gx in range(gx_min, gx_max + 1):
                for gy in range(gy_min, gy_max + 1):
                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        self.obstacle_grid[gx, gy] = True

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """世界坐标转网格索引"""
        gx = int((x - self.min_x) / self.cell_size)
        gy = int((y - self.min_y) / self.cell_size)
        gx = max(0, min(gx, self.width - 1))
        gy = max(0, min(gy, self.height - 1))
        return gx, gy

    def is_occupied(self, x: float, y: float) -> bool:
        """判断世界坐标点是否被建筑占据"""
        gx, gy = self.world_to_grid(x, y)
        return self.obstacle_grid[gx, gy]

    def is_grid_occupied(self, gx: int, gy: int) -> bool:
        """判断网格单元是否被占据"""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.obstacle_grid[gx, gy]
        return True  # 超出边界视为障碍物

    def get_neighbors(self, gx: int, gy: int) -> List[Tuple[int, int]]:
        """返回四邻域内可通行的邻居网格索引（不考虑高度）"""
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if not self.is_grid_occupied(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors