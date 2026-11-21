import math
from typing import Tuple
from typing import List

from drones.base_drone import BaseDrone
from utils.geometry import clamp_position
import time
from typing import Optional

# 使用算法模块提供的轻量 A* 与视线检测工具
try:
    from algorithms.cbs_pathplan import a_star_plan_world, line_of_sight_world
except Exception:
    a_star_plan_world = None
    line_of_sight_world = None
try:
    from algorithms.path_tracker import PathTracker
except Exception:
    PathTracker = None


def move_drone(drone: BaseDrone, delta_time: float, map_grid=None,
               min_altitude: float = 5.0) -> None:
    """ 对目标无人机进行移动, 参数为一个无人机对象以及delta_time步长, 直接修改无人机对象, 不返回值 """
    if not drone.is_alive():
        return
    old_position = list(drone.position[:3])
    drone.update_position(delta_time)
    drone.position = clamp_position(drone.position, (-600, -600, 0), (600, 600, 350))
    if drone.position[2] < min_altitude:
        drone.position[2] = float(min_altitude)

    # ===== 建筑体避让 =====
    # 2D 网格只记录建筑 footprint 和楼顶高度。若下一步水平位置落进建筑 footprint，
    # 且高度还没到楼顶+安全余量，就禁止这帧的水平移动，并强制向上爬升。
    # 进攻方（PathTracker/直追）和防守方（CBS/PathFollower）都走这里。
    if map_grid is not None and hasattr(map_grid, "is_occupied"):
        try:
            new_x, new_y, new_z = drone.position
            if map_grid.is_occupied(new_x, new_y):
                roof = float(map_grid.get_max_height_at_world(new_x, new_y))
                safety = 5.0
                if new_z < roof + safety:
                    old_x, old_y, old_z = old_position
                    old_roof = float(map_grid.get_max_height_at_world(old_x, old_y))
                    old_inside = (map_grid.is_occupied(old_x, old_y)
                                  and old_z < old_roof + safety)
                    if (not old_inside) or map_grid.is_occupied(new_x, new_y):
                        drone.position[0] = old_x
                        drone.position[1] = old_y
                    # 直接给 z 增量，避免控制器下一帧把 vz 设回下降导致贴墙抽动
                    drone.position[2] = min(350.0, float(drone.position[2]) + 20.0 * delta_time)
                    drone.velocity[2] = max(float(drone.velocity[2]), 20.0)
        except Exception:
            pass

        # 兜底：碰撞分离等外部逻辑可能已把位置推进 footprint
        try:
            x, y, z = drone.position
            if map_grid.is_occupied(x, y):
                roof = float(map_grid.get_max_height_at_world(x, y))
                safety = 5.0
                if z < roof + safety:
                    old_x, old_y, old_z = old_position
                    old_roof = float(map_grid.get_max_height_at_world(old_x, old_y))
                    old_inside = (map_grid.is_occupied(old_x, old_y)
                                  and old_z < old_roof + safety)
                    if not old_inside:
                        x, y = old_x, old_y
                    z = min(350.0, float(z) + 20.0 * delta_time)
                    drone.position = [float(x), float(y), float(z)]
                    drone.velocity[2] = max(float(drone.velocity[2]), 20.0)
        except Exception:
            pass
    if drone.position[2] < min_altitude:
        drone.position[2] = float(min_altitude)


def set_speed(drone: BaseDrone, velocity: Tuple[float, float, float]) -> None:
    """ 设置无人机的数量, 传入要操作的无人机实例, 以及要设置的速度向量列表 """
    drone.set_velocity(list(velocity))
    #设置目标无人机的速度向量
    #调用目标无人机的成员函数.set_velocity


def chase_target(drone: BaseDrone, target: BaseDrone, map_grid: Optional[object] = None) -> None:
    """让无人机朝向目标无人机的位置追踪。若提供 map_grid，则会尝试避障。

    参数：
    - drone: 被控制的无人机实例
    - target: 目标无人机实例（要求有 `.position`）
    - map_grid: 可选的 `MapGrid` 实例；若提供且可用，则会尝试基于 A* 规划并缓存路径
    """
    if not drone.is_alive() or not target.is_alive():
        return
    target_pos = tuple(target.position[:3])
    chase_point(drone, target_pos, map_grid=map_grid)
    #如果追踪方与被追踪方 有一方已经坠毁, 则直接返回, 不进行追踪
    #计算追踪方位置指向被追踪方位置的方向向量direction
    #计算direction的模长, 即为二者之间的距离distance
    #如果距离小于0.01, 则直接返回
    #将方向向量direction除以距离distance, 完成归一化, 得到归一化的单位方向向量normalized
    #将单位方向向量normalized乘上无人机的最大速度.max_speed, 得到无人机的最大速度向量velocity
    #将无人机的速度设置为最大速度向量velocity, 无人机将沿着这个速度向量的方向运动

def chase_point(drone: BaseDrone, target_point: List[float], map_grid: Optional[object] = None) -> None:
    """让无人机朝向固定点追踪。若提供 map_grid，则使用 PathTracker（若可用）进行按需重规划和路径缓存。
    """
    if not drone.is_alive():
        return

    # 直接速度设置函数
    def _set_velocity_towards(dr: BaseDrone, pt: List[float]):
        direction = [t - p for t, p in zip(pt, dr.position)]
        distance = math.sqrt(sum(v * v for v in direction))
        if distance < 1e-3:
            return
        normalized = [v / distance for v in direction]
        velocity = [normalized[i] * dr.max_speed for i in range(3)]
        dr.set_velocity(velocity)

    # 如果没有地图或 PathTracker 不可用，退化为原直线追踪
    if map_grid is None or PathTracker is None:
        _set_velocity_towards(drone, target_point)
        return

    # 获取或创建跟踪器
    if not hasattr(drone, "_path_tracker") or drone._path_tracker is None:
        try:
            drone._path_tracker = PathTracker(drone)
        except Exception:
            drone._path_tracker = None

    tracker = getattr(drone, "_path_tracker", None)
    if tracker is None:
        _set_velocity_towards(drone, target_point)
        return

    next_wp = tracker.update(tuple(target_point[:3]), map_grid)
    if next_wp is None:
        # 目标可直达或规划失败 -> 直接追踪目标点
        _set_velocity_towards(drone, target_point)
    else:
        # 追踪下一个航点
        _set_velocity_towards(drone, next_wp)


class PathFollower:
    """按路径推进航点；优先使用 CBS 的 tick 时间表，缺失时按累计距离估算。"""

    __slots__ = ("drone", "plan_speed", "tolerance", "climb_speed", "altitude_tolerance",
                 "points", "times", "idx", "start_time")

    def __init__(self, drone: BaseDrone, plan_speed: float = 30.0, tolerance: float = 3.0,
                 climb_speed: float = 25.0, altitude_tolerance: float = 8.0):
        self.drone = drone
        self.plan_speed = max(0.5, float(plan_speed))
        self.tolerance = max(0.5, float(tolerance))
        self.climb_speed = max(1.0, float(climb_speed))
        self.altitude_tolerance = max(0.5, float(altitude_tolerance))
        self.points: List[Tuple[float, float, float]] = []
        self.times: List[float] = []
        self.idx = 0
        self.start_time = 0.0

    def set_path(self, points, start_time: float, plan_speed: Optional[float] = None,
                 times=None) -> bool:
        """装载路径；points 为世界坐标，start_time 为仿真时间。"""
        self.points = [
            (float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0)
            for p in points
        ]
        self.idx = 1 if len(self.points) > 1 else 0
        self.start_time = float(start_time)
        if times is not None and len(times) == len(self.points):
            base = float(times[0])
            self.times = [max(0.0, float(t) - base) for t in times]
            return bool(self.points)

        speed = max(0.5, float(plan_speed)) if plan_speed is not None else self.plan_speed
        self.times = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            dist = math.hypot(b[0] - a[0], b[1] - a[1])
            self.times.append(self.times[-1] + max(dist, 1e-6) / speed)
        return bool(self.points)

    def clear(self) -> None:
        self.points = []
        self.times = []
        self.idx = 0

    def finished(self) -> bool:
        return not self.points or self.idx >= len(self.points)

    def update(self, now: float) -> bool:
        """按当前仿真时间给速度；返回 False 表示路径已走完。"""
        if self.finished():
            return False
        elapsed = max(0.0, float(now) - self.start_time)
        last = len(self.points) - 1
        while self.idx < last and elapsed >= self.times[self.idx]:
            self.idx += 1

        target = self.points[self.idx]
        pos = self.drone.position
        dx, dy, dz = target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]
        hdist = math.hypot(dx, dy)
        remain_h = max(0.05, self.times[self.idx] - elapsed)
        vxy = min(self.drone.max_speed, hdist / remain_h)
        if hdist > 1e-6:
            vx, vy = dx / hdist * vxy, dy / hdist * vxy
        else:
            vx = vy = 0.0
        vz = 0.0
        if abs(dz) > self.altitude_tolerance:
            remain_v = max(0.5, self.times[self.idx] - elapsed)
            vz = max(-self.climb_speed, min(self.climb_speed, dz / remain_v))
        self.drone.set_velocity([vx, vy, vz])

        if hdist < self.tolerance:
            self.idx += 1
        return not self.finished()