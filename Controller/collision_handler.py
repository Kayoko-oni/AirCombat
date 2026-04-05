from typing import List, Tuple

from drones.base_drone import BaseDrone
from utils.geometry import distance


def detect_collisions(drones: List[BaseDrone], min_distance: float = 8.0) -> List[Tuple[BaseDrone, BaseDrone]]:
    collisions = []
    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            if not drones[i].is_alive() or not drones[j].is_alive():
                continue
            if distance(drones[i].position, drones[j].position) < min_distance:
                collisions.append((drones[i], drones[j]))
    return collisions


def resolve_collisions(collisions: List[Tuple[BaseDrone, BaseDrone]]) -> None:
    for drone_a, drone_b in collisions:
        relative_speed = sum(abs(a - b) for a, b in zip(drone_a.velocity, drone_b.velocity))
        damage = max(35.0, min(130.0, relative_speed * 3.0))
        drone_a.apply_damage(damage)
        drone_b.apply_damage(damage)
        if drone_a.is_alive():
            drone_a.set_velocity([-v for v in drone_a.velocity])
        if drone_b.is_alive():
            drone_b.set_velocity([-v for v in drone_b.velocity])
