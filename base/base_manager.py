from typing import List, Dict
from drones.base_drone import BaseDrone
from utils.geometry import distance

# 会直接撞击基地的进攻方类型（反辐射无人机自毁而非撞击）
_BASE_CRASH_TYPES = {"AttackDrone", "TankDrone", "EWDrone"}


class BaseManager:
    """支持多基地的管理器。

    配置兼容：优先读取 config['bases']（列表），若不存在则回退到 config['base']（单基地）。
    每个基地内部以 dict 存储：{'name','position','health','max_health','collision_radius'}。
    """

    def __init__(self, config: dict):
        self.bases: List[Dict] = []
        # 首先尝试兼容多基地配置
        if isinstance(config.get("bases"), list) and config.get("bases"):
            for b in config.get("bases", []):
                pos = b.get("position") or b.get("pos") or [0.0, 0.0, 0.0]
                health = float(b.get("health", config.get("base", {}).get("health", 100)))
                collision = float(b.get("collision_radius", config.get("base", {}).get("collision_radius", 12.5)))
                name = b.get("name", f"base_{len(self.bases)}")
                self.bases.append({
                    "name": name,
                    "position": pos,
                    "health": health,
                    "max_health": health,
                    "collision_radius": collision,
                })
        else:
            # 向后兼容：使用单基地配置
            base_cfg = config.get("base", {})
            pos = base_cfg.get("position", [0.0, 0.0, 0.0])
            health = float(base_cfg.get("health", 100.0))
            collision = float(base_cfg.get("collision_radius", 12.5))
            self.bases.append({
                "name": base_cfg.get("name", "base_0"),
                "position": pos,
                "health": health,
                "max_health": health,
                "collision_radius": collision,
            })

    def check_collisions(self, drones: List[BaseDrone]) -> bool:
        """检测进攻方与各基地碰撞，AttackDrone/TankDrone/EWDrone 撞击基地扣血；
        反辐射无人机不在此列（在靠近时自行坠毁，不造成基地伤害）。
        返回 True 表示所有基地被摧毁（仿真结束条件）。"""
        for drone in drones:
            if drone.drone_type in _BASE_CRASH_TYPES and drone.is_alive():
                for base in self.bases:
                    if base["health"] <= 0:
                        continue
                    dist = distance(drone.position, base["position"])
                    # 增加 2 米缓冲，避免鬼畜
                    if dist < base["collision_radius"] + 2.0:
                        # 如果距离略大于碰撞半径，也强制触发（模拟撞击）
                        damage = drone.attack_power
                        drone.death_cause = "collision"
                        drone.apply_damage(drone.health)
                        base["health"] -= damage
                        print(f"Base {base['name']} hit by {drone.name}, damage={damage}, health={base['health']:.1f}, dist={dist:.2f}")
                        # 关键：将无人机位置直接拉到基地中心，确保它不再继续移动（可选）
                        # drone.position = base["position"][:]  # 如果希望立即消失可以不移动
                        break  # 一架无人机一次只撞击一个基地
        # 仿真结束条件：所有基地被摧毁
        return all(b["health"] <= 0 for b in self.bases)

    def is_destroyed(self) -> bool:
        return all(b["health"] <= 0 for b in self.bases)

    def get_health(self) -> float:
        """返回所有基地的聚合血量（总和），保持向后兼容。"""
        return sum(b["health"] for b in self.bases)

    def get_max_health(self) -> float:
        return sum(b.get("max_health", 0.0) for b in self.bases)

    def get_healths(self) -> List[float]:
        """返回各基地的剩余血量列表（按配置顺序）。"""
        return [b["health"] for b in self.bases]

    def get_positions(self) -> List[List[float]]:
        return [b["position"] for b in self.bases]