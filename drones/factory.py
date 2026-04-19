# drones/factory.py
"""无人机工厂函数，负责创建各机型实例并添加到列表。"""

from drones.offensive.attack_drone import AttackDrone
from drones.offensive.tank_drone import TankDrone
from drones.defensive.scout_drone import ScoutDrone
from drones.defensive.interceptor_drone import InterceptorDrone


def create_attack_drone(name: str, position: list, config: dict, drones: list) -> None:
    """创建一架攻击机并直接加入 drones 列表, 参数为: name 位置坐标列表 config 要添加到的目标无人机列表(总列表为drones) """
    drone = AttackDrone(name=name, position=position, config=config["drones"]["attack"])
    drones.append(drone)


def create_tank_drone(name: str, position: list, config: dict, drones: list) -> None:
    """创建一架肉盾机并直接加入 drones 列表, 参数为: name 位置坐标列表 config 要添加到的目标无人机列表(总列表为drones) """
    drone = TankDrone(name=name, position=position, config=config["drones"]["tank"])
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