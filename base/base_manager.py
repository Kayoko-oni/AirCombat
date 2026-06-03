from typing import List
from drones.base_drone import BaseDrone
from utils.geometry import distance

# 会直接撞击基地的进攻方类型（反辐射无人机自毁而非撞击）
_BASE_CRASH_TYPES = {"AttackDrone", "TankDrone", "EWDrone"}


class BaseManager:
    def __init__(self, config: dict):
        self.max_health = config["base"]["health"]   # 最大血量
        self.health = config["base"]["health"]
        self.position = config["base"]["position"]   #所有要调用基地位置的地方的代码都改为了从字典中读取，若要更改基地位置，应在字典中更改
        self.collision_radius = config["base"]["collision_radius"]

    def check_collisions(self, drones: List[BaseDrone]) -> bool:
        """检测进攻方与基地碰撞，AttackDrone/TankDrone 撞击基地扣血；
        反辐射无人机不在此列（在靠近时自行坠毁，不造成基地伤害）。
        返回 True 表示基地被摧毁。"""
        for drone in drones:
            if drone.drone_type in _BASE_CRASH_TYPES and drone.is_alive():
                if distance(drone.position, self.position) < self.collision_radius:
                    damage = drone.attack_power         # 记录进攻方攻击力
                    # 标记为与基地碰撞造成的坠毁（视为碰撞类死亡）
                    drone.death_cause = "collision"
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