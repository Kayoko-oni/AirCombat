from typing import List

from drones.base_drone import BaseDrone
from utils.geometry import distance


class RadarSensor:
    def __init__(self, range_m: float, pulse_interval: float = 1.0):
        self.range_m = range_m
        self.pulse_interval = pulse_interval

    def scan(self, drones: List[BaseDrone]) -> List[dict]:
        detections = []
        for drone in drones:
            if distance([0, 0, 0], drone.position) <= self.range_m:
                detections.append(
                    {
                        "name": drone.name,
                        "type": drone.drone_type,
                        "position": drone.position,
                        "health": drone.health,
                    }
                )
        return detections
