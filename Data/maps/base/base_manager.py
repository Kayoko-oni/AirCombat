from typing import List
from drones.base_drone import BaseDrone
from utils.geometry import distance

class BaseManager:
    def __init__(self, config: dict):
        self.max_health = config["base"]["health"]   # 最大血量
        self.health = config["base"]["health"]
        self.position = config["base"]["position"]   #所有要调用基地位置的地方的代码都改为了从字典中读取，若要更改基地位置，应在字典中更改
        self.collision_radius = config["base"]["collision_radius"]

    def check_collisions(self, drones: List[BaseDrone]) -> bool:
        """检测进攻方与基地碰撞，进攻方坠毁，基地扣除其血量。返回 True 表示基地被摧毁。"""
        for drone in drones:
            # 直接判断是否为进攻方
            if drone.drone_type in {"AttackDrone", "TankDrone"} and drone.is_alive():
                if distance(drone.position, self.position) < self.collision_radius:
                    damage = drone.attack_power         # 记录进攻方攻击力
                    drone.apply_damage(drone.health)  # 进攻方坠毁
                    self.health -= damage          # 基地扣血
                    print(f"Base hit by {drone.name}, damage={damage}, health={self.health:.1f}")
                    if self.health <= 0:
                        return True
        return False

    def is_destroyed(self) -> bool:
        return self.health <= 0

    def get_health(self) -> float:
        return self.health

    def get_max_health(self) -> float:
        return self.max_health