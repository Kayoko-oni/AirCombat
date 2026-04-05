from typing import List

from drones.base_drone import BaseDrone


def auction_assign(drones: List[BaseDrone], tasks: List[dict]) -> dict:
    assignment = {}
    for index, drone in enumerate(drones):
        assignment[drone.name] = tasks[index % len(tasks)] if tasks else None
    return assignment