from drones.base_drone import BaseDrone


class TankDrone(BaseDrone):
    def __init__(self, name: str, position=None, config=None):
        # 使用配置参数，如果没有则使用默认值
        config = config or {}
        super().__init__(
            name=name,
            drone_type="TankDrone",
            position=position or [0.0, 0.0, 0.0],
            velocity=[config.get("speed", 4.5), 0.0, 0.0],
            health=config.get("health", 150.0),
            attack_power=config.get("attack_power", 15.0),
            battery=config.get("battery", 120.0),
            max_speed=config.get("max_speed", 20.0),
        )
