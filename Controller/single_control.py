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


def move_drone(drone: BaseDrone, delta_time: float) -> None:
    """ 对目标无人机进行移动, 参数为一个无人机对象以及delta_time步长, 直接修改无人机对象, 不返回值 """
    if not drone.is_alive():
        return
    drone.update_position(delta_time)
    drone.position = clamp_position(drone.position, (-500, -500, 0), (500, 500, 100))
    # TODO: 使边界限制可配置而非硬编码
    
    #移动目标无人机
    #若此函数调用的无人机对象已经坠毁, 则不调用此函数, 直接return
    #调用drones —— base_drone —— update_position 函数, 更新无人机的位置
    #将无人机的位置设定在一定范围内: x[-500, 500] y[-500, 500] z[-100, 100]
     #   但是由于之后要加载地图, 所以z的范围如果在0到100之间可能会更加合理, 可以导入地图过后再做更改
    


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