"""基于网格的 CBS（Conflict-Based Search）多机路径规划。

相对旧实现的关键变化（公开函数签名保持兼容）：
- 低层 A* 在 (格子, 时刻) 上搜索。约束与时间相关，旧版只按格子去重，
  既可能漏掉"等一会儿再走"的解，也会让部分约束永远无法满足。
- 冲突检测同时覆盖顶点冲突与交换（边）冲突。
- 停在目标格的 agent 不再被加约束：改由移动的一方绕行，并把它转成静态障碍，
  避免旧版"给已停在终点的 agent 加约束 -> 永远不可满足 -> 死循环"。
- t=0 的同格起点不视为冲突。本仿真不判同阵营伤害，补充防守方也从同一基地出生。
- 超时返回无冲突的子集（其余给 []），不再整批丢弃。
- 目标格去重：多架追到同一格时，重复者改用附近空闲格，避免抢同一个终点。
- 超过 max_agents 时只规划前 max_agents 架，其余返回 []，由调用方退化处理。
- 两个入口：`cbs_plan_paths` 返回平滑后的几何路径（可视化/旧调用方）；
  `cbs_plan_schedules` 返回原始 tick 轨迹 `(points, times)`，供 PathFollower 执行。

坐标与时间：只在 XY 上规划；一步 = 一次移动或原地等待，代价 1。低层用加权 Chebyshev
启发式（_H_WEIGHT=1.5，路径代价不超过最优的 1.5 倍；改成 1.0 即严格 A*）。
返回路径是折线，最后停留由调用方/执行器保证。
z 取目标高度，目标高度为 0 时取起点高度；若飞行高度需要参与避障，应另做 2.5D。
"""

import heapq
import time
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

from utils.logger import get_logger

LOGGER = get_logger(__name__)

_VERTEX = 0          # (agent, 0, gx, gy, t0, t1): t0<=t<=t1 期间禁止进入
_EDGE = 1            # (agent, 1, x1, y1, x2, y2, t): t 步禁止走这条边
_INF_T = 1 << 30
_DIAG_EPS = 0.001    # 同代价路径中偏好少走对角，避免贴着邻车道的无人机切来切去
_H_WEIGHT = 1.5      # 加权 A*：牺牲少量最优性换实时性（1.0 为严格 A*）
# 对角优先，路径更直；最后一项是原地等待
_OFFSETS = ((-1, -1), (1, -1), (-1, 1), (1, 1),
            (-1, 0), (1, 0), (0, -1), (0, 1), (0, 0))

Cell = Tuple[int, int]


def _extract_positions(item: Any) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """从输入项中提取 (start_world, goal_world)，兼容二元组与带 .position/.target 的对象。"""
    def to_pos(v) -> Optional[Tuple[float, float, float]]:
        if hasattr(v, "position"):
            v = v.position
        if isinstance(v, (list, tuple)) and len(v) >= 2:
            z = v[2] if len(v) > 2 else 0.0
            return float(v[0]), float(v[1]), float(z)
        return None

    if isinstance(item, (list, tuple)) and len(item) == 2:
        s = to_pos(item[0])
        g = to_pos(item[1])
        return (s, g) if s is not None and g is not None else None

    if hasattr(item, "position"):
        s = to_pos(item)
        tgt = getattr(item, "target", None)
        g = to_pos(tgt) if tgt is not None else None
        return (s, g) if s is not None and g is not None else None
    return None


def _nearest_free(gx: int, gy: int, map_grid, max_radius: int = 5,
                  forbidden: Optional[Set[Cell]] = None,
                  ref: Optional[Cell] = None) -> Optional[Cell]:
    """在 max_radius（Chebyshev）邻域内找最近的空闲格；forbidden 中的格子不选。

    ref 用于距离相同时的次级排序（例如让去重后的目标靠近该机起点）。
    """
    forbidden = forbidden or set()
    if not map_grid.is_grid_occupied(gx, gy) and (gx, gy) not in forbidden:
        return gx, gy
    best: Optional[Cell] = None
    best_key = None
    w, h = map_grid.width, map_grid.height
    for dx in range(-max_radius, max_radius + 1):
        for dy in range(-max_radius, max_radius + 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = gx + dx, gy + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            if map_grid.is_grid_occupied(nx, ny) or (nx, ny) in forbidden:
                continue
            ref_dist = ((nx - ref[0]) ** 2 + (ny - ref[1]) ** 2) if ref is not None else 0
            key = (abs(dx) + abs(dy), ref_dist, nx, ny)
            if best_key is None or key < best_key:
                best_key, best = key, (nx, ny)
    return best


def _flat_occupancy(map_grid) -> Optional[bytes]:
    """把占位图缓存成扁平 bytes，热循环里用下标判断比方法/numpy 索引更快。"""
    flat = getattr(map_grid, "_occ_flat", None)
    if flat is not None:
        return flat
    grid = getattr(map_grid, "obstacle_grid", None)
    if grid is None:
        return None
    try:
        flat = grid.tobytes()
    except AttributeError:
        return None
    map_grid._occ_flat = flat
    return flat


def _a_star_cells(start: Cell, goal: Cell, map_grid,
                  time_limit_s: float = 0.05) -> Optional[List[Cell]]:
    """无约束单机 A*：只按格子去重，用于初始解和 a_star_plan_world。

    无约束时不存在"等一会儿再走"的必要，因此不进入时间维，速度快很多。
    """
    w, h = map_grid.width, map_grid.height
    flat = _flat_occupancy(map_grid)
    if flat is not None:
        def is_wall(gx: int, gy: int) -> bool:
            return flat[gx * h + gy]
    else:
        is_wall = map_grid.is_grid_occupied
    if not (0 <= goal[0] < w and 0 <= goal[1] < h) or is_wall(*goal):
        return None

    def heuristic(cell: Cell) -> float:
        dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return (dx if dx > dy else dy) * _H_WEIGHT

    openq = [(heuristic(start), 0, start)]
    best_g: Dict[Cell, int] = {start: 0}
    came: Dict[Cell, Cell] = {}
    t_begin = time.perf_counter()
    pops = 0

    while openq:
        pops += 1
        if (pops & 63) == 0 and time.perf_counter() - t_begin > time_limit_s:
            return None
        _, neg_g, cell = heapq.heappop(openq)
        g = -neg_g
        if cell == goal:
            path = [cell]
            while cell in came:
                cell = came[cell]
                path.append(cell)
            path.reverse()
            return path
        if g > best_g.get(cell, _INF_T):
            continue
        ng = g + 1
        for dx, dy in _OFFSETS:
            if dx == 0 and dy == 0:
                continue
            nx, ny = cell[0] + dx, cell[1] + dy
            if not (0 <= nx < w and 0 <= ny < h) or is_wall(nx, ny):
                continue
            if dx and dy and (is_wall(cell[0] + dx, cell[1]) or is_wall(cell[0], cell[1] + dy)):
                continue
            if ng >= best_g.get((nx, ny), _INF_T):
                continue
            best_g[(nx, ny)] = ng
            came[(nx, ny)] = cell
            # f 相同时优先扩展更深的节点（-ng）；对角加极小罚值，让同代价路径更直
            diag_eps = _DIAG_EPS if (dx and dy) else 0.0
            heapq.heappush(openq, (ng + heuristic((nx, ny)) + diag_eps, -ng, (nx, ny)))
    return None


def _a_star_on_grid(start: Cell, goal: Cell, map_grid,
                    vertex_blocks: Sequence[Tuple[int, int, int, int]] = (),
                    edge_blocks: Sequence[Tuple[int, int, int, int, int]] = (),
                    time_limit_s: float = 0.05,
                    horizon: Optional[int] = None) -> Optional[List[Cell]]:
    """在 (格子, 时刻) 状态空间上做 A*；g 就等于时刻，所以无需 best_g。

    vertex_blocks: (gx, gy, t0, t1)，t0<=t<=t1 期间禁止进入该格
    edge_blocks:   (x1, y1, x2, y2, t)，禁止 t 时刻从 (x1,y1) 走到 (x2,y2)
    返回包含起点与终点的格子列表；无解或超时返回 None。
    """
    if not vertex_blocks and not edge_blocks:
        return _a_star_cells(start, goal, map_grid, time_limit_s)

    w, h = map_grid.width, map_grid.height
    flat = _flat_occupancy(map_grid)
    if flat is not None:
        def is_wall(gx: int, gy: int) -> bool:
            return flat[gx * h + gy]
    else:
        is_wall = map_grid.is_grid_occupied
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))
    if not (0 <= goal[0] < w and 0 <= goal[1] < h) or is_wall(*goal):
        return None

    # 约束按格子索引，避免每次扩展都扫描整个约束列表
    vmap: Dict[Cell, List[Tuple[int, int]]] = {}
    max_ct = 0
    for x, y, t0, t1 in vertex_blocks:
        vmap.setdefault((x, y), []).append((t0, t1))
        max_ct = max(max_ct, t0, t1 if t1 < _INF_T else t0)
    edge_set = set(edge_blocks)
    for e in edge_blocks:
        max_ct = max(max_ct, e[4])

    def blocked(cell: Cell, t: int) -> bool:
        for a, b in vmap.get(cell, ()):
            if a <= t <= b:
                return True
        return False

    def can_wait(cell: Cell, t0: int, t1: int) -> bool:
        """t0 时刻在该格的状态，能否只靠等待到达 t1。"""
        for a, b in vmap.get(cell, ()):
            if a <= t1 and b >= t0:
                return False
        return True

    def can_stay(cell: Cell, t: int) -> bool:
        """到 t 时刻为止，之后是否还能一直停在该格。"""
        for _, b in vmap.get(cell, ()):
            if b >= t:
                return False
        return True

    def heuristic(cell: Cell) -> float:
        dx, dy = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return (dx if dx > dy else dy) * _H_WEIGHT

    h0 = heuristic(start)
    if horizon is None:
        horizon = min(4096, max_ct + 2 * h0 + 64)
    if start == goal and can_stay(goal, 0):
        return [start]

    t_begin = time.perf_counter()
    openq = [(h0, 0, start)]
    # best_t[cell] = 目前到达该格的最早时刻；若更早的状态可以原地等到当前时刻，
    # 当前状态就是被支配的，可以跳过。这一剪枝把时间维拉回接近普通 A* 的规模。
    best_t: Dict[Cell, int] = {start: 0}
    seen = {(start[0], start[1], 0)}
    came: Dict[Tuple[int, int, int], Cell] = {}
    pops = 0

    while openq:
        pops += 1
        if (pops & 63) == 0 and time.perf_counter() - t_begin > time_limit_s:
            return None
        _, t, cell = heapq.heappop(openq)
        if cell == goal and can_stay(goal, t):
            path = [cell]
            key = (cell[0], cell[1], t)
            while key in came:
                parent = came[key]
                path.append(parent)
                key = (parent[0], parent[1], key[2] - 1)
            path.reverse()
            return path

        nt = t + 1
        if nt > horizon:
            continue
        for dx, dy in _OFFSETS:
            nx, ny = cell[0] + dx, cell[1] + dy
            if not (0 <= nx < w and 0 <= ny < h) or is_wall(nx, ny):
                continue
            # 禁止切角穿墙
            if dx and dy and (is_wall(cell[0] + dx, cell[1]) or is_wall(cell[0], cell[1] + dy)):
                continue
            if blocked((nx, ny), nt):
                continue
            if (dx or dy) and (cell[0], cell[1], nx, ny, t) in edge_set:
                continue
            if (nx, ny, nt) in seen:
                continue
            prev = best_t.get((nx, ny))
            if prev is not None and prev <= nt and can_wait((nx, ny), prev, nt):
                continue
            seen.add((nx, ny, nt))
            best_t[(nx, ny)] = nt if prev is None else min(prev, nt)
            came[(nx, ny, nt)] = cell
            diag_eps = _DIAG_EPS if (dx and dy) else 0.0
            heapq.heappush(openq, (nt + heuristic((nx, ny)) + diag_eps, nt, (nx, ny)))
    return None


def _iter_conflicts(paths: Sequence[Sequence[Cell]]):
    """按时间顺序产出所有冲突：(kind, a, b, cell_a, cell_b, t)。

    t=0 的同格起点不算冲突（同阵营集结/同基地出生）。
    """
    n = len(paths)
    if n < 2:
        return
    t_max = max(len(p) for p in paths)
    for t in range(t_max):
        if t > 0:
            seen: Dict[Cell, int] = {}
            for i, p in enumerate(paths):
                c = p[t] if t < len(p) else p[-1]
                j = seen.get(c)
                if j is not None:
                    yield ("vertex", j, i, c, c, t)
                else:
                    seen[c] = i
        if t + 1 >= t_max:
            continue
        for i in range(n):
            pi = paths[i]
            a0 = pi[t] if t < len(pi) else pi[-1]
            a1 = pi[t + 1] if t + 1 < len(pi) else pi[-1]
            if a0 == a1:
                continue
            for j in range(i + 1, n):
                pj = paths[j]
                b0 = pj[t] if t < len(pj) else pj[-1]
                b1 = pj[t + 1] if t + 1 < len(pj) else pj[-1]
                if b0 != b1 and a0 == b1 and b0 == a1:
                    yield ("edge", i, j, a0, a1, t)


def _find_conflict(paths: Sequence[Sequence[Cell]]):
    return next(_iter_conflicts(paths), None)


def _conflicting_agents(paths: Sequence[Sequence[Cell]]) -> Set[int]:
    bad: Set[int] = set()
    for _, a, b, *_ in _iter_conflicts(paths):
        bad.add(a)
        bad.add(b)
    return bad


def _split_constraints(constraints: frozenset, agent: int):
    """把约束集拆成该 agent 的顶点块与边块，供低层 A* 使用。"""
    vertex_blocks = []
    edge_blocks = []
    for c in constraints:
        if c[0] != agent:
            continue
        if c[1] == _VERTEX:
            vertex_blocks.append(c[2:6])
        else:
            edge_blocks.append(c[2:7])
    return vertex_blocks, edge_blocks


def _branch(conflict, paths: Sequence[Sequence[Cell]]):
    """由冲突生成子约束：返回 [(agent, constraint), ...]。"""
    kind, a, b, ca, cb, t = conflict
    if kind == "vertex":
        a_goal = t >= len(paths[a]) - 1
        b_goal = t >= len(paths[b]) - 1
        if a_goal and b_goal:
            # 两架都停在同一个目标格：去重后不应出现，交给上层放弃该分支
            return []
        if a_goal:
            return [(b, (b, _VERTEX, ca[0], ca[1], t, _INF_T))]
        if b_goal:
            return [(a, (a, _VERTEX, ca[0], ca[1], t, _INF_T))]
        return [
            (a, (a, _VERTEX, ca[0], ca[1], t, t)),
            (b, (b, _VERTEX, cb[0], cb[1], t, t)),
        ]
    return [
        (a, (a, _EDGE, ca[0], ca[1], cb[0], cb[1], t)),
        (b, (b, _EDGE, cb[0], cb[1], ca[0], ca[1], t)),
    ]


def _bresenham_line(x0: int, y0: int, x1: int, y1: int):
    """Bresenham 整数直线，含两端。"""
    dx, dy = abs(x1 - x0), abs(y1 - y0)
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


def _line_of_sight(a: Cell, b: Cell, map_grid) -> bool:
    """网格级视线检测；与 A* 一致，禁止对角切角穿墙。"""
    occupied = map_grid.is_grid_occupied
    prev = None
    for cell in _bresenham_line(a[0], a[1], b[0], b[1]):
        if occupied(cell[0], cell[1]):
            return False
        if prev is not None and prev[0] != cell[0] and prev[1] != cell[1]:
            if occupied(cell[0], prev[1]) or occupied(prev[0], cell[1]):
                return False
        prev = cell
    return True


def _smooth_grid_path(path: Sequence[Cell], map_grid) -> List[Cell]:
    """贪心可视线平滑：在不撞墙的前提下连接尽量远的点。"""
    if not path or len(path) <= 2:
        return list(path)
    new_path = [path[0]]
    i, n = 0, len(path)
    while i < n - 1:
        j = n - 1
        while j > i + 1 and not _line_of_sight(path[i], path[j], map_grid):
            j -= 1
        new_path.append(path[j])
        i = j
    return new_path


def _smooth_paths(paths: Sequence[Sequence[Cell]], map_grid) -> List[List[Cell]]:
    """平滑后重新检查冲突；平滑破坏了无冲突性就整体退回原路径。"""
    if not paths:
        return []
    if len(paths) == 1:
        return [_smooth_grid_path(paths[0], map_grid)]
    smoothed = [_smooth_grid_path(p, map_grid) for p in paths]
    if _find_conflict(smoothed) is None:
        return smoothed
    return [list(p) for p in paths]


def _paths_to_world(paths: Sequence[Optional[Sequence[Cell]]],
                    z_values: Sequence[float], map_grid) -> List[Optional[List[List[float]]]]:
    """网格路径 -> 世界坐标；None/空路径保留为 None。"""
    idx = [i for i, p in enumerate(paths) if p]
    if not idx:
        return [None] * len(paths)
    smoothed = _smooth_paths([paths[i] for i in idx], map_grid)
    result: List[Optional[List[List[float]]]] = [None] * len(paths)
    for i, path in zip(idx, smoothed):
        z = z_values[i]
        world_path = []
        for gx, gy in path:
            x, y = map_grid.grid_to_world(gx, gy)
            world_path.append([x, y, z])
        result[i] = world_path
    return result


def _separate_cells(cells: Sequence[Cell], map_grid, radius: int,
                    refs: Optional[Sequence[Cell]] = None) -> List[Cell]:
    """把重复的格子错开到附近空闲格。

    起点用它错开同基地出生的多架；终点用它避免多架抢同一个目标格。
    """
    used: Set[Cell] = set()
    result: List[Cell] = []
    for i, (gx, gy) in enumerate(cells):
        cell = (gx, gy)
        if cell in used:
            alt = _nearest_free(gx, gy, map_grid, max_radius=radius, forbidden=used,
                                ref=refs[i] if refs is not None else None)
            if alt is not None:
                LOGGER.debug("CBS: agent %d duplicate cell %s -> %s", i, cell, alt)
                cell = alt
            else:
                LOGGER.warning("CBS: duplicate cell %s has no nearby free cell", cell)
        used.add(cell)
        result.append(cell)
    return result


def _pad(paths: List, dropped: int) -> List:
    """为超过 max_agents 的 agent 补空路径，保持返回列表与输入对齐。"""
    if not paths:
        return []
    return list(paths) + [[] for _ in range(dropped)]


class _Node:
    """CBS 约束树节点：约束集 + 当前各路路径 + 代价（路径步数之和）。"""

    __slots__ = ("constraints", "paths", "cost")

    def __init__(self, constraints: frozenset, paths: Tuple[Tuple[Cell, ...], ...]):
        self.constraints = constraints
        self.paths = paths
        self.cost = sum(len(p) for p in paths)

    def __lt__(self, other: "_Node") -> bool:
        return self.cost < other.cost


def _plan_grid(agents: List[Any], map_grid, max_agents: int,
               time_limit_ms: int):
    """CBS 核心求解：返回 (grid_paths, z_values, dropped)。

    grid_paths 与截断后的 agents 对齐；不可用/冲突的 agent 为 None。
    完全没有可用路径时返回 ([], [], dropped)。
    """
    if map_grid is None or not agents:
        return [], [], 0

    dropped = 0
    if len(agents) > max_agents:
        dropped = len(agents) - max_agents
        agents = agents[:max_agents]
    total = len(agents)

    # 1) 世界坐标 -> 网格；起点/终点落在障碍里时吸附到最近空闲格
    starts: List[Cell] = []
    goals: List[Cell] = []
    z_values: List[float] = []
    for item in agents:
        parsed = _extract_positions(item)
        if parsed is None:
            LOGGER.warning("CBS: cannot parse agent input, abort")
            return [], [], dropped
        s, g = parsed
        sgx, sgy = map_grid.world_to_grid(s[0], s[1])
        ggx, ggy = map_grid.world_to_grid(g[0], g[1])
        start = _nearest_free(sgx, sgy, map_grid, max_radius=3)
        goal = _nearest_free(ggx, ggy, map_grid, max_radius=5)
        if start is None or goal is None:
            LOGGER.warning("CBS: start/goal stuck in obstacle, abort")
            return [], [], dropped
        starts.append(start)
        goals.append(goal)
        z_values.append(g[2] if g[2] != 0.0 else s[2])

    # 2) 同格起点错开、同一目标格去重
    starts = _separate_cells(starts, map_grid, radius=3)
    goals = _separate_cells(goals, map_grid, radius=4, refs=starts)

    # 3) 初始解：每机独立 A*，按剩余预算分配时间
    t_begin = time.perf_counter()
    deadline = t_begin + time_limit_ms / 1000.0
    paths: List[Tuple[Cell, ...]] = []

    def _give_up():
        """初始解阶段来不及：已算出的路径里挑无冲突的子集返回。"""
        bad = _conflicting_agents(paths)
        result = [None if i in bad else paths[i] for i in range(len(paths))]
        result += [None] * (total - len(paths))
        return (result, z_values, dropped) if any(result) else ([], [], dropped)

    for i, (s, g) in enumerate(zip(starts, goals)):
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return _give_up()
        per_agent = min(0.05, remaining / max(1, len(starts) - i))
        p = _a_star_on_grid(s, g, map_grid, time_limit_s=per_agent)
        if p is None:
            LOGGER.warning("CBS: no initial path for agent %d", i)
            return _give_up()
        paths.append(tuple(p))

    # 4) 高层 CBS：约束树 + 闭集
    root = _Node(frozenset(), tuple(paths))
    openq = [root]
    closed: Set[frozenset] = set()
    nodes = 0
    last = root
    solution = None
    while openq:
        if time.perf_counter() > deadline:
            break
        node = heapq.heappop(openq)
        if node.constraints in closed:
            continue
        closed.add(node.constraints)
        nodes += 1

        last = node
        conflict = _find_conflict(node.paths)
        if conflict is None:
            solution = node.paths
            break

        for agent, constraint in _branch(conflict, node.paths):
            if constraint in node.constraints:
                continue
            new_constraints = node.constraints | {constraint}
            if new_constraints in closed:
                continue
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                break
            vblocks, eblocks = _split_constraints(new_constraints, agent)
            new_path = _a_star_on_grid(starts[agent], goals[agent], map_grid,
                                       vblocks, eblocks, time_limit_s=min(0.05, remaining))
            if new_path is None:
                continue
            new_paths = list(node.paths)
            new_paths[agent] = tuple(new_path)
            heapq.heappush(openq, _Node(new_constraints, tuple(new_paths)))

    elapsed_ms = (time.perf_counter() - t_begin) * 1000.0

    if solution is not None:
        if _find_conflict(solution) is not None:  # 兜底：不应发生
            LOGGER.error("CBS: internal conflict validation failed")
            return [], [], dropped
        LOGGER.debug("CBS solved: agents=%d nodes=%d cost=%d elapsed=%.1fms",
                     len(solution), nodes, sum(len(p) for p in solution), elapsed_ms)
        return list(solution), z_values, dropped

    LOGGER.debug("CBS timeout/unsolved: agents=%d nodes=%d elapsed=%.1fms",
                 len(paths), nodes, elapsed_ms)
    bad = _conflicting_agents(last.paths)
    result = [None if i in bad else last.paths[i] for i in range(len(last.paths))]
    result += [None] * (total - len(result))
    return (result, z_values, dropped) if any(result) else ([], [], dropped)


def cbs_plan_paths(agents: List[Any], map_grid=None, max_agents: int = 6,
                   time_limit_ms: int = 200) -> List[List[List[float]]]:
    """CBS 主入口（几何路径）：返回平滑后的世界坐标路径，失败项为 []。"""
    grid_paths, z_values, dropped = _plan_grid(agents, map_grid, max_agents, time_limit_ms)
    if not grid_paths:
        return []
    world = _paths_to_world(grid_paths, z_values, map_grid)
    return _pad([p if p else [] for p in world], dropped)


def cbs_plan_schedules(agents: List[Any], map_grid=None, max_agents: int = 6,
                       time_limit_ms: int = 200,
                       tick_seconds: float = 0.5):
    """CBS 主入口（带时间戳）：返回 [(points, times), ...]，供路径执行器使用。

    points 为原始网格路径（不裁剪、不平滑），一个点对应一个 tick；
    times[i] = i * tick_seconds；None/[] 表示该机没有可用路径。
    """
    grid_paths, z_values, dropped = _plan_grid(agents, map_grid, max_agents, time_limit_ms)
    if not grid_paths:
        return []
    tick = max(1e-3, float(tick_seconds))
    result = []
    for path, z in zip(grid_paths, z_values):
        if not path:
            result.append(None)
            continue
        points = []
        for gx, gy in path:
            x, y = map_grid.grid_to_world(gx, gy)
            points.append([x, y, z])
        result.append((points, [i * tick for i in range(len(points))]))
    return _pad(result, dropped)

def line_of_sight_world(p1: Tuple[float, float, float], p2: Tuple[float, float, float],
                        map_grid) -> bool:
    """世界坐标层面（XY 投影）的视线检测。"""
    a = map_grid.world_to_grid(p1[0], p1[1])
    b = map_grid.world_to_grid(p2[0], p2[1])
    return _line_of_sight(a, b, map_grid)


def a_star_plan_world(start_world, goal_world, map_grid, max_time_ms: int = 50):
    """单机 A*：世界坐标进、世界坐标出；失败返回 []。"""
    if map_grid is None:
        return []
    sgx, sgy = map_grid.world_to_grid(start_world[0], start_world[1])
    ggx, ggy = map_grid.world_to_grid(goal_world[0], goal_world[1])
    start = _nearest_free(sgx, sgy, map_grid, max_radius=5)
    goal = _nearest_free(ggx, ggy, map_grid, max_radius=10)
    if start is None:
        LOGGER.warning("a_star_plan_world: start stuck in obstacle")
        return []
    if goal is None:
        LOGGER.warning("a_star_plan_world: goal stuck in obstacle")
        return []

    grid_path = _a_star_on_grid(start, goal, map_grid, time_limit_s=max_time_ms / 1000.0)
    if not grid_path:
        return []
    z = goal_world[2]
    world_path = []
    for gx, gy in _smooth_grid_path(grid_path, map_grid):
        x, y = map_grid.grid_to_world(gx, gy)
        world_path.append([x, y, z])
    return world_path
