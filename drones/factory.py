# drones/factory.py
"""无人机工厂函数，负责创建各机型实例并添加到列表。"""

from drones.offensive.attack_drone import AttackDrone
from drones.offensive.tank_drone import TankDrone
from drones.offensive.anti_radiation_drone import AntiRadiationDrone
from drones.offensive.ew_drone import EWDrone
from drones.defensive.scout_drone import ScoutDrone
from drones.defensive.interceptor_drone import InterceptorDrone
from drones.defensive.anti_ew_drone import AntiEWDrone


def create_attack_drone(name: str, position: list, config: dict, drones: list) -> None:
    """创建一架攻击机并直接加入 drones 列表, 参数为: name 位置坐标列表 config 要添加到的目标无人机列表(总列表为drones) """
    drone = AttackDrone(name=name, position=position, config=config["drones"]["attack"])
    drones.append(drone)


def create_tank_drone(name: str, position: list, config: dict, drones: list) -> None:
    """创建一架肉盾机并直接加入 drones 列表, 参数为: name 位置坐标列表 config 要添加到的目标无人机列表(总列表为drones) """
    drone = TankDrone(name=name, position=position, config=config["drones"]["tank"])
    drones.append(drone)


def create_anti_radiation_drone(name: str, position: list, config: dict, drones: list) -> None:
    """创建一架反辐射无人机并直接加入 drones 列表，同时初始化屏蔽方向"""
    drone_config = config.get("drones", {}).get("anti_radiation", {})
    drone = AntiRadiationDrone(name=name, position=position, config=drone_config)
    # 固化屏蔽方向：在多基地配置下选择最近的基地（向后兼容单基地）
    base_pos = None
    bases = config.get("bases")
    if isinstance(bases, list) and bases:
        # 选择与初始位置最近的基地
        try:
            import math
            best = None
            best_dist = float("inf")
            for b in bases:
                bp = b.get("position")
                if bp is None:
                    continue
                dx = position[0] - bp[0]
                dy = position[1] - bp[1]
                dz = position[2] - (bp[2] if len(bp) > 2 else 0.0)
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < best_dist:
                    best_dist = dist
                    best = bp
            base_pos = best or config.get("base", {}).get("position", [0.0, 0.0, 0.0])
        except Exception:
            base_pos = config.get("base", {}).get("position", [0.0, 0.0, 0.0])
    else:
        base_pos = config.get("base", {}).get("position", [0.0, 0.0, 0.0])

    drone.init_jamming_direction(base_pos)
    drones.append(drone)


def create_ew_drone(name: str, position: list, config: dict, drones: list) -> None:
    """创建一架电子战无人机并直接加入 drones 列表"""
    drone_config = config.get("drones", {}).get("ew", {})
    drone = EWDrone(name=name, position=position, config=drone_config)
    drones.append(drone)


def create_anti_ew_drone(name: str, position: list, config: dict, drones: list) -> None:
    """创建一架反电子战无人机并直接加入 drones 列表"""
    drone_config = config.get("drones", {}).get("anti_ew", {})
    drone = AntiEWDrone(name=name, position=position, config=drone_config)
    drones.append(drone)


def create_scout_drone(name: str, position: list, config: dict, drones: list) -> None:
    """创建一架侦察机并直接加入 drones 列表, 参数为: name 位置坐标列表 config 要添加到的目标无人机列表(总列表为drones) """
    drone = ScoutDrone(name=name, position=position, config=config["drones"]["scout"])
    drones.append(drone)


def create_interceptor_drone(name: str, position: list, config: dict, drones: list) -> None:
    """创建一架拦截机并直接加入 drones 列表, 参数为: name 位置坐标列表 config 要添加到的目标无人机列表(总列表为drones) """
    drone = InterceptorDrone(name=name, position=position, config=config["drones"]["interceptor"])
    drones.append(drone)


def create_drone_team(config: dict, drones: list = None) -> list:
    """创建初始四架无人机，返回无人机列表"""
    if drones is None:
        drones = []   # 如果没传列表，新建一个空列表
    create_attack_drone("Attack-01", [-100, -50, 20], config, drones)
    create_tank_drone("Tank-01", [-120, -50, 40], config, drones)
    create_scout_drone("Scout-01", [100, 50, 30], config, drones)
    create_interceptor_drone("Intercepter-01", [120, 50, 50], config, drones)
    return drones