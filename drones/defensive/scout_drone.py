from drones.base_drone import BaseDrone


class ScoutDrone(BaseDrone):
    def __init__(self, name: str, position=None, config=None):
        # 使用配置参数，如果没有则使用默认值
        config = config or {}
        super().__init__(
            name=name,
            drone_type="ScoutDrone",
            position=position or [0.0, 0.0, 0.0],
            velocity=[config.get("speed", 15.0), 0.0, 0.0],
            health=config.get("health", 60.0),
            attack_power=config.get("attack_power", 5.0),
            battery=config.get("battery", 60.0),
            max_speed=config.get("max_speed", 40.0),
        )
