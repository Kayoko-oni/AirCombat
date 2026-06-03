"""电子战干扰无人机（Electronic Warfare Drone）— 进攻方。

压制防守方传感器+通信链路。存活时对半径内防守方施加干扰：
   - 被干扰的防守方不参与新任务分配
  - 路径规划速度大幅下降
干扰持续至被击落，或被反电子战无人机靠近后失效。
"""

from drones.base_drone import BaseDrone


class EWDrone(BaseDrone):
    def __init__(self, name: str, position=None, config=None):
        config = config or {}
        super().__init__(
            name=name,
            drone_type="EWDrone",
            position=position or [0.0, 0.0, 0.0],
            velocity=[config.get("speed", 8.0), 0.0, 0.0],
            health=config.get("health", 40.0),
            attack_power=config.get("attack_power", 15.0),
            battery=config.get("battery", 100.0),
            max_speed=config.get("max_speed", 25.0),
        )
        self.jamming_radius = config.get("jamming_radius", 200.0)
        self.jamming_active = True

    def is_in_jamming_range(self, defender_position: list) -> bool:
        if not self.jamming_active or self.destroyed:
            return False
        dist = sum((p - q) ** 2 for p, q in zip(self.position, defender_position)) ** 0.5
        return dist <= self.jamming_radius

    def disable_jamming(self):
        self.jamming_active = False
