import math
from typing import Tuple

from drones.base_drone import BaseDrone
from utils.geometry import clamp_position


def move_drone(drone: BaseDrone, delta_time: float) -> None:
    if not drone.is_alive():
        return
    drone.update_position(delta_time)
    drone.position = clamp_position(drone.position, (-500, -500, -100), (500, 500, 100))
    # TODO: 使边界限制可配置而非硬编码


def set_speed(drone: BaseDrone, velocity: Tuple[float, float, float]) -> None:
    drone.set_velocity(list(velocity))


def chase_target(drone: BaseDrone, target: BaseDrone) -> None:
    """让无人机朝向目标点追踪。"""
    if not drone.is_alive() or not target.is_alive():
        return
    direction = [t - p for t, p in zip(target.position, drone.position)]
    distance = math.sqrt(sum(v * v for v in direction))
    if distance < 1e-3:
        return
    normalized = [v / distance for v in direction]
    velocity = [normalized[i] * drone.max_speed for i in range(3)]
    drone.set_velocity(velocity)
