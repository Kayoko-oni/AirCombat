from typing import List

from drones.base_drone import BaseDrone


def cbs_plan_paths(drones: List[BaseDrone]) -> List[List[List[float]]]:
    paths = []
    for drone in drones:
        path = [drone.position, [drone.position[0] + 10.0, drone.position[1], drone.position[2]]]
        paths.append(path)
    return paths
