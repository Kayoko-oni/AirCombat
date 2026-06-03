"""反电子战无人机（Anti-EW Drone）— 防守方。

对抗进攻方的电子战干扰无人机。不受 EW 干扰影响，
靠近 EW 无人机一定范围后使其干扰功能永久失效。

属性与侦察无人机一致（血量60、攻击力5）。
"""

from drones.base_drone import BaseDrone


class AntiEWDrone(BaseDrone):
    def __init__(self, name: str, position=None, config=None):
        config = config or {}
        super().__init__(
            name=name,
            drone_type="AntiEWDrone",
            position=position or [0.0, 0.0, 0.0],
            velocity=[config.get("speed", 15.0), 0.0, 0.0],
            health=config.get("health", 60.0),
            attack_power=config.get("attack_power", 5.0),
            battery=config.get("battery", 60.0),
            max_speed=config.get("max_speed", 40.0),
        )
        # 靠近 EW 无人机此距离后使其失效
        self.disruption_radius = config.get("disruption_radius", 150.0)
