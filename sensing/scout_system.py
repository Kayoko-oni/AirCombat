from typing import List

from drones.base_drone import BaseDrone


class ScoutSystem:
    def __init__(self):
        self.detected = []

    def classify(self, drones: List[BaseDrone]) -> List[dict]:
        result = []
        for drone in drones:
            if drone.health > 0:
                result.append({
                    "name": drone.name,
                    "type": drone.drone_type,
                    "is_alive": drone.is_alive(),
                })
        self.detected = result
        return result
