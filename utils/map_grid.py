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
        # 每个格子的最大建筑高度（世界坐标 z 的最高点），用于简单的 Z 轴避障判断
        self.obstacle_height = np.zeros((self.width, self.height), dtype=float)

        # 标记每个建筑覆盖的网格，并记录该格的最高建筑顶点高度
        for (cx, cy, cz), (sx, sy, sz) in buildings:
            half_x = sx / 2.0
            half_y = sy / 2.0
            left = cx - half_x
            right = cx + half_x
            bottom = cy - half_y
            top = cy + half_y
            top_z = float(cz) + float(sz) / 2.0
            gx_min, gy_min = self.world_to_grid(left, bottom)
            gx_max, gy_max = self.world_to_grid(right, top)
            for gx in range(gx_min, gx_max + 1):
                for gy in range(gy_min, gy_max + 1):
                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        self.obstacle_grid[gx, gy] = True
                        # 记录该格的最高顶点
                        if top_z > self.obstacle_height[gx, gy]:
                            self.obstacle_height[gx, gy] = top_z

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

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """将网格索引转换为该网格中心点的世界坐标（x, y）。

        注意：不包含高度(z)，路径规划器在需要时可用无人机的当前z作为高度。
        """
        # 网格左下角世界坐标 = (min_x, min_y)
        # 网格中心 = 左下角 + (gx + 0.5) * cell_size
        x = self.min_x + (gx + 0.5) * self.cell_size
        y = self.min_y + (gy + 0.5) * self.cell_size
        return x, y

    def get_cell_max_height(self, gx: int, gy: int) -> float:
        """返回指定格子中记录的最大建筑高度（世界坐标 z）。超出范围返回 0.0。"""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return float(self.obstacle_height[gx, gy])
        return 0.0

    def get_max_height_at_world(self, x: float, y: float) -> float:
        """返回给定世界坐标点所在格子的最大建筑高度。"""
        gx, gy = self.world_to_grid(x, y)
        return self.get_cell_max_height(gx, gy)

    def _bresenham_cells(self, x0: int, y0: int, x1: int, y1: int):
        """整数 Bresenham 生成器，返回两个格子坐标之间经过的格子（含终点）。"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        if dy <= dx:
            err = dx // 2
            while x != x1:
                yield x, y
                x += sx
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
            yield x1, y1
        else:
            err = dy // 2
            while y != y1:
                yield x, y
                y += sy
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
            yield x1, y1

    def max_height_along_line(self, p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> float:
        """返回两点连线上（按网格采样）遇到的最高建筑顶点高度。"""
        gx0, gy0 = self.world_to_grid(p1[0], p1[1])
        gx1, gy1 = self.world_to_grid(p2[0], p2[1])
        max_h = 0.0
        for gx, gy in self._bresenham_cells(gx0, gy0, gx1, gy1):
            if 0 <= gx < self.width and 0 <= gy < self.height:
                h = self.obstacle_height[gx, gy]
                if h > max_h:
                    max_h = float(h)
        return max_h