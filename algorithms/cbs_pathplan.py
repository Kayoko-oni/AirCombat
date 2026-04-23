
# CBS算法粗糙实现（无复杂约束结构，仅支持点冲突，A*为4邻域无障碍）
import heapq
from typing import List, Tuple

def simple_a_star(start: Tuple[int, int], goal: Tuple[int, int], constraints: List[Tuple[int, int, int]]) -> List[Tuple[int, int]]:
    """粗糙的A*,只考虑点约束,4邻域无障碍物。"""
    from collections import deque
    open_set = []
    heapq.heappush(open_set, (0 + abs(goal[0]-start[0]) + abs(goal[1]-start[1]), 0, start, [start]))
    closed = set()
    constraint_set = set((x, y, t) for x, y, t in constraints)
    while open_set:
        f, g, curr, path = heapq.heappop(open_set)
        if (curr, g) in closed:
            continue
        closed.add((curr, g))
        if curr == goal:
            return path
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = curr[0]+dx, curr[1]+dy
            if 0 <= nx < 200 and 0 <= ny < 200:
                if (nx, ny, g+1) in constraint_set:
                    continue
                heapq.heappush(open_set, (g+1+abs(goal[0]-nx)+abs(goal[1]-ny), g+1, (nx, ny), path+[(nx, ny)]))
    return []

def simple_find_conflict(paths: List[List[Tuple[int, int]]]) -> Tuple[int, int, Tuple[int, int], int]:
    """返回第一个冲突：(agent1, agent2, 冲突点, 时间)"""
    maxlen = max(len(p) for p in paths)
    for t in range(maxlen):
        positions = {}
        for i, path in enumerate(paths):
            pos = path[t] if t < len(path) else path[-1]
            if pos in positions:
                return positions[pos], i, pos, t
            positions[pos] = i
    return None

def cbs_plan_paths_correct(drones) -> List[List[List[float]]]:
    # 坐标转换
    def pos_to_grid(pos):
        return (int(pos[0] // 10 + 100), int(pos[1] // 10 + 100))
    def grid_to_pos(grid_pos):
        return [(grid_pos[0] - 100) * 10, (grid_pos[1] - 100) * 10, 50.0]
    starts = [pos_to_grid(drone.position) for drone in drones]
    goals = [pos_to_grid(drone.target) for drone in drones]

    class Node:
        def __init__(self, constraints, paths, cost):
            self.constraints = constraints  # [(agent, x, y, t)]
            self.paths = paths
            self.cost = cost
        def __lt__(self, other):
            return self.cost < other.cost

    # 根节点
    root_paths = []
    for i in range(len(drones)):
        path = simple_a_star(starts[i], goals[i], [])
        if not path:
            return []
        root_paths.append(path)
    root = Node([], root_paths, sum(len(p) for p in root_paths))
    open_list = [root]

    while open_list:
        node = heapq.heappop(open_list)
        conflict = simple_find_conflict(node.paths)
        if not conflict:
            # 转回三维坐标
            return [[grid_to_pos(pos) for pos in path] for path in node.paths]
        a1, a2, pos, t = conflict
        for agent in [a1, a2]:
            new_constraints = node.constraints + [(agent, pos[0], pos[1], t)]
            new_paths = node.paths.copy()
            path = simple_a_star(starts[agent], goals[agent], [(x, y, tt) for ag, x, y, tt in new_constraints if ag == agent])
            if not path:
                continue
            new_paths = new_paths.copy()
            new_paths[agent] = path
            heapq.heappush(open_list, Node(new_constraints, new_paths, sum(len(p) for p in new_paths)))
    return []