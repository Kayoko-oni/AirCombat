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

        """
        侦察机类
        调用了父类BassDrone的构造函数 设定各项参数
        位置: 使用传入的数据, 如果没有传入位置, 则使用[0, 0, 0]
        速度: 初始速度设置为[15, 0, 0] 这样设置是否合理? 可以进行讨论.
        血量设置为60 攻击力设置为5
        初始电量设置为60
        最大速度设置为40
        """