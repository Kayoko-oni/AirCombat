from typing import List

from drones.base_drone import BaseDrone


class SwarmController:
    def __init__(self, drones: List[BaseDrone]):
        self.drones = drones

    def assign_tasks(self) -> None:
        # TODO: 使用 auction_assign 或其他任务分配算法
        for index, drone in enumerate(self.drones):
            drone.set_velocity([index * 0.5, 0.0, 0.0])

    def plan_paths(self) -> None:
        # TODO: 使用 CBS 或其他路径规划算法
        pass
