from drones.base_drone import BaseDrone


class InterceptorDrone(BaseDrone):
    def __init__(self, name: str, position=None, config=None):
        # 使用配置参数，如果没有则使用默认值
        config = config or {}
        super().__init__(
            name=name,
            drone_type="InterceptorDrone",
            position=position or [0.0, 0.0, 0.0],
            velocity=[config.get("speed", 12.0), 0.0, 0.0],
            health=config.get("health", 90.0),
            attack_power=config.get("attack_power", 20.0),
            battery=config.get("battery", 90.0),
            max_speed=config.get("max_speed", 35.0),
        )
