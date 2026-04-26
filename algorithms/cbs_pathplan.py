
"""
基于网格的简化 CBS 路径规划（仅支持二维 XY，点冲突约束）。

说明：
- 只在 XY 平面上做规划，Z 不参与路径搜索（返回的路径为三维点，但 Z 由起点或目标决定）。
- 需要一个 `MapGrid` 实例（utils.map_grid.MapGrid），它提供 `world_to_grid`, `grid_to_world`,
  `is_grid_occupied`, `get_neighbors`, `width`, `height` 等方法。
- 支持输入形式灵活：
  - 每个 agent 可以是一个二元组 (start, goal)，其中 start/goal 可以是世界坐标列表 [x,y,z]，
    或者是带有 `.position` 的对象（无人机实例）。
  - 也可传入一组拥有 `.position` 和 `.target` 属性的对象（向后兼容）。

返回值：每个 agent 对应一个路径，路径为世界坐标点列表 [[x,y,z], ...]；若某个 agent 无可行路径，则返回空列表。
"""

import heapq
import time
from typing import List, Tuple, Optional, Any


def _extract_positions(item: Any) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """从输入项中提取 (start_world, goal_world)。支持多种输入格式。
    返回值为两个长度为3的元组 (x,y,z)。若无法解析则返回 None。
    """
    # 情形 1：二元组/列表 (start, goal)
    if isinstance(item, (list, tuple)) and len(item) == 2:
        start, goal = item
        def to_pos(v):
            if hasattr(v, "position"):
                return tuple(v.position[:3])
            if isinstance(v, (list, tuple)) and len(v) >= 2:
                z = v[2] if len(v) > 2 else 0.0
                return (float(v[0]), float(v[1]), float(z))
            return None
        s = to_pos(start)
        g = to_pos(goal)
        if s is None or g is None:
            return None
        return s, g

    # 情形 2：对象，尝试读取 .position 和 .target（或 .target.position）
    if hasattr(item, "position"):
        s = tuple(item.position[:3])
        tgt = getattr(item, "target", None)
        if tgt is None:
            return None
        if hasattr(tgt, "position"):
            g = tuple(tgt.position[:3])
        elif isinstance(tgt, (list, tuple)) and len(tgt) >= 2:
            z = tgt[2] if len(tgt) > 2 else 0.0
            g = (float(tgt[0]), float(tgt[1]), float(z))
        else:
            return None
        return s, g

    return None


def _find_nearest_free(gx: int, gy: int, map_grid, max_radius: int = 5) -> Optional[Tuple[int, int]]:
    """如果给定网格被占据，尝试在邻域内寻找最近的空闲网格。"""
    from collections import deque
    if not map_grid.is_grid_occupied(gx, gy):
        return gx, gy
    visited = set()
    q = deque()
    q.append((gx, gy, 0))
    visited.add((gx, gy))
    while q:
        x, y, d = q.popleft()
        if d >= max_radius:
            continue
        for nx, ny in [(x-1,y),(x+1,y),(x,y-1),(x,y+1)]:
            if 0 <= nx < map_grid.width and 0 <= ny < map_grid.height and (nx, ny) not in visited:
                if not map_grid.is_grid_occupied(nx, ny):
                    return nx, ny
                visited.add((nx, ny))
                q.append((nx, ny, d+1))
    return None


def _a_star_on_grid(start: Tuple[int, int], goal: Tuple[int, int], map_grid, constraints: List[Tuple[int, int, int]],
                    time_limit_s: float = 0.05, max_steps: int = 20000) -> List[Tuple[int, int]]:
    """在网格上运行带时间约束的 A*。

    constraints: 列表 (x, y, t) —— 表示在时间 t 时该格子不可到达（点冲突约束）。
    返回网格索引列表（包含起点与终点），找不到返回空列表。
    """
    start_time = time.perf_counter()
    constraint_set = set((x, y, t) for x, y, t in constraints)

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    openq = []
    # (f, g, (x,y), path)
    heapq.heappush(openq, (heuristic(start, goal), 0, start, [start]))
    best_g = {start: 0}

    while openq:
        if time.perf_counter() - start_time > time_limit_s:
            return []
        f, g, curr, path = heapq.heappop(openq)
        if g > max_steps:
            continue
        if curr == goal:
            return path
        # 扩展八连通邻居（含等待 stay in place）
        neigh_offsets = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1),(0,0)]
        neighbors = []
        for dx, dy in neigh_offsets:
            nx, ny = curr[0] + dx, curr[1] + dy
            if not (0 <= nx < map_grid.width and 0 <= ny < map_grid.height):
                continue
            if dx == 0 and dy == 0:
                # stay in place
                neighbors.append((nx, ny))
                continue
            if map_grid.is_grid_occupied(nx, ny):
                continue
            # 对角移动时避免“切角穿墙”：要求两个相邻正交格都可通行
            if abs(dx) == 1 and abs(dy) == 1:
                if map_grid.is_grid_occupied(curr[0] + dx, curr[1]) or map_grid.is_grid_occupied(curr[0], curr[1] + dy):
                    continue
            neighbors.append((nx, ny))

        for nb in neighbors:
            ng = g + 1
            if (nb[0], nb[1], ng) in constraint_set:
                continue
            prev_best = best_g.get(nb)
            if prev_best is not None and ng >= prev_best:
                continue
            best_g[nb] = ng
            heapq.heappush(openq, (ng + heuristic(nb, goal), ng, nb, path + [nb]))

    return []


def _find_conflict(paths: List[List[Tuple[int, int]]]) -> Optional[Tuple[int, int, Tuple[int, int], int]]:
    """寻找第一个顶点冲突，返回 (agent1, agent2, pos, t) 或 None。"""
    if not paths:
        return None
    maxlen = max(len(p) for p in paths)
    for t in range(maxlen):
        seen = {}
        for i, p in enumerate(paths):
            pos = p[t] if t < len(p) else p[-1]
            if pos in seen:
                return seen[pos], i, pos, t
            seen[pos] = i
    return None


def cbs_plan_paths(agents: List[Any], map_grid=None, max_agents: int = 6, time_limit_ms: int = 2000) -> List[List[List[float]]]:
    """CBS 主入口：对输入 agents 进行路径规划。

    agents: 列表，每项可为 (start, goal) 或对象（需要 .position 与 .target）
    map_grid: MapGrid 实例；若为 None，则退化为不做网格约束的空实现（返回空路径列表）。
    max_agents: 超过此数量时建议不要使用 CBS（为性能考虑），函数会直接返回空列表。
    time_limit_ms: 整个 CBS 搜索的时间限制（毫秒）。
    """
    # 简单的防护：若没有 map_grid 或者 agent 数量过多则不给出规划
    if map_grid is None:
        return []
    if len(agents) == 0:
        return []
    if len(agents) > max_agents:
        # 为避免在低性能机器上卡死，超过阈值时直接放弃
        return []

    # 解析 start/goal
    starts_world = []
    goals_world = []
    orig_z = []  # 用于返回时恢复 z
    for item in agents:
        parsed = _extract_positions(item)
        if parsed is None:
            # 如果解析失败，填充一个占位（后续会导致失败并返回空结果）
            starts_world.append(None)
            goals_world.append(None)
            orig_z.append(0.0)
            continue
        s, g = parsed
        starts_world.append(s)
        goals_world.append(g)
        # 优先使用目标的 z；若目标高度为 0 则保留起点 z
        orig_z.append(g[2] if g[2] != 0.0 else s[2])

    # 将世界坐标转为网格索引
    starts = []
    goals = []
    for s, g in zip(starts_world, goals_world):
        if s is None or g is None:
            starts.append(None)
            goals.append(None)
            continue
        sgx, sgy = map_grid.world_to_grid(s[0], s[1])
        ggx, ggy = map_grid.world_to_grid(g[0], g[1])
        # 若目标/起点落在障碍物上，尝试寻找附近的空格子
        if map_grid.is_grid_occupied(sgx, sgy):
            nf = _find_nearest_free(sgx, sgy, map_grid, max_radius=3)
            if nf is None:
                starts.append(None)
                goals.append(None)
                continue
            sgx, sgy = nf
        if map_grid.is_grid_occupied(ggx, ggy):
            nf = _find_nearest_free(ggx, ggy, map_grid, max_radius=5)
            if nf is None:
                starts.append(None)
                goals.append(None)
                continue
            ggx, ggy = nf
        starts.append((sgx, sgy))
        goals.append((ggx, ggy))

    # 如果任一起点或目标解析失败，则直接返回空（避免部分规划带来的复杂性）
    if any(s is None or g is None for s, g in zip(starts, goals)):
        return []

    # 先用不带约束的 A* 为每个 agent 生成初始路径
    paths = []
    for si, gi in zip(starts, goals):
        p = _a_star_on_grid(si, gi, map_grid, [], time_limit_s=0.05)
        if not p:
            return []
        paths.append(p)

    # CBS open list
    class Node:
        def __init__(self, constraints, paths):
            self.constraints = list(constraints)  # [(agent, x, y, t)]
            self.paths = [list(p) for p in paths]
            self.cost = sum(len(p) for p in paths)
        def __lt__(self, other):
            return self.cost < other.cost

    open_list = []
    root = Node([], paths)
    heapq.heappush(open_list, root)

    deadline = time.perf_counter() + (time_limit_ms / 1000.0)

    while open_list:
        if time.perf_counter() > deadline:
            return []
        node = heapq.heappop(open_list)
        conflict = _find_conflict(node.paths)
        if conflict is None:
            # 将网格路径转回世界坐标并返回（三维点：使用 orig_z 保存的高度）
            world_paths = []
            for idx, path in enumerate(node.paths):
                # 对最终返回的路径做可视线平滑，减少阶梯式运动
                smooth_grid = _smooth_grid_path(path, map_grid)
                z = orig_z[idx]
                world_path = []
                for gx, gy in smooth_grid:
                    x, y = map_grid.grid_to_world(gx, gy)
                    world_path.append([x, y, z])
                world_paths.append(world_path)
            return world_paths

        a1, a2, pos, t = conflict
        # 针对冲突的双方分别生成约束并重规划其路径
        for agent_idx in (a1, a2):
            new_constraints = list(node.constraints) + [(agent_idx, pos[0], pos[1], t)]
            # 收集仅针对该 agent 的约束
            agent_specific = [(x, y, tt) for ag, x, y, tt in new_constraints if ag == agent_idx]
            starts_copy = starts[agent_idx]
            goals_copy = goals[agent_idx]
            new_path = _a_star_on_grid(starts_copy, goals_copy, map_grid, agent_specific, time_limit_s=0.1)
            if not new_path:
                continue
            new_paths = [list(p) for p in node.paths]
            new_paths[agent_idx] = new_path
            heapq.heappush(open_list, Node(new_constraints, new_paths))

    return []


def _bresenham_line(x0: int, y0: int, x1: int, y1: int):
    """Bresenham integer line generator between two grid cells (inclusive)."""
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


def _line_of_sight(a: Tuple[int, int], b: Tuple[int, int], map_grid) -> bool:
    """检查从格子 a 到格子 b 的直线是否不穿过任何占据格子（网格级视线检测）。"""
    for x, y in _bresenham_line(a[0], a[1], b[0], b[1]):
        if map_grid.is_grid_occupied(x, y):
            return False
    return True


def _smooth_grid_path(path: List[Tuple[int, int]], map_grid) -> List[Tuple[int, int]]:
    """对网格路径做贪心的可视线平滑（尽量连接可直达的更远点）。"""
    if not path:
        return path
    new_path = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        j = n - 1
        while j > i + 1:
            if _line_of_sight(path[i], path[j], map_grid):
                break
            j -= 1
        new_path.append(path[j])
        i = j
    return new_path


def line_of_sight_world(p1: Tuple[float, float, float], p2: Tuple[float, float, float], map_grid) -> bool:
    """世界坐标层面的视线检测（返回 True 表示两点在 XY 投影上可直达）。"""
    a = map_grid.world_to_grid(p1[0], p1[1])
    b = map_grid.world_to_grid(p2[0], p2[1])
    return _line_of_sight(a, b, map_grid)


def a_star_plan_world(start_world: Tuple[float, float, float], goal_world: Tuple[float, float, float], map_grid, max_time_ms: int = 50) -> List[List[float]]:
    """在世界坐标下执行 A*，返回世界坐标路径 [[x,y,z], ...] 或空列表。"""
    if map_grid is None:
        return []
    sgx, sgy = map_grid.world_to_grid(start_world[0], start_world[1])
    ggx, ggy = map_grid.world_to_grid(goal_world[0], goal_world[1])
    # 若起点或目标位于障碍上，尝试寻找附近空位
    if map_grid.is_grid_occupied(sgx, sgy):
        nf = _find_nearest_free(sgx, sgy, map_grid, max_radius=3)
        if nf is None:
            return []
        sgx, sgy = nf
    if map_grid.is_grid_occupied(ggx, ggy):
        nf = _find_nearest_free(ggx, ggy, map_grid, max_radius=5)
        if nf is None:
            return []
        ggx, ggy = nf

    grid_path = _a_star_on_grid((sgx, sgy), (ggx, ggy), map_grid, [], time_limit_s=max_time_ms / 1000.0)
    if not grid_path:
        return []

    smooth = _smooth_grid_path(grid_path, map_grid)
    z = goal_world[2] if goal_world[2] != 0.0 else start_world[2]
    world_path = []
    for gx, gy in smooth:
        x, y = map_grid.grid_to_world(gx, gy)
        world_path.append([x, y, z])
    return world_path