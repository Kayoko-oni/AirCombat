"""反辐射无人机：专门压制防守方雷达/通信节点。
不攻击基地，而是在接近基地一定距离后自行坠毁。
存活期间使防守方对其出现方向的感知被屏蔽。
"""

from __future__ import annotations

import math
from typing import List, Optional

from drones.base_drone import BaseDrone


class AntiRadiationDrone(BaseDrone):
    """反辐射无人机（Anti-Radiation Drone, ARD）

    核心机制：
    1. 朝基地方向飞行，不参与直接攻击
    2. 进入自毁距离后自行坠毁（不造成基地伤害）
    3. 存活期间产生一个"感知屏蔽扇区"：防守方无法获知该扇区内进攻方无人机信息
    4. 扇区中心方向 = 从基地指向 ARD 初始位置的方向（固定在生成时刻）
    """

    def __init__(self, name: str, position=None, config=None):
        config = config or {}
        super().__init__(
            name=name,
            drone_type="AntiRadiationDrone",
            position=position or [0.0, 0.0, 0.0],
            velocity=[config.get("speed", 10.0), 0.0, 0.0],
            health=config.get("health", 45.0),
            attack_power=config.get("attack_power", 0.0),   # 不直接攻击
            battery=config.get("battery", 90.0),
            max_speed=config.get("max_speed", 28.0),
        )

        # ---------- 自毁参数 ----------
        # 距离基地小于此值时自行坠毁（不造成基地伤害）
        self.self_destruct_distance: float = config.get("self_destruct_distance", 80.0)

        # ---------- 感知屏蔽参数 ----------
        # 屏蔽扇区半角（度），扇区总宽度为 2 * half_angle
        self.jamming_sector_half_angle_deg: float = config.get("jamming_sector_half_angle_deg", 60.0)
        self.jamming_sector_half_angle: float = math.radians(self.jamming_sector_half_angle_deg)

        # 屏蔽方向在生成时刻固定：从基地指向本机初始位置的方向
        self.jamming_direction: Optional[float] = None  # 弧度，atan2 结果
        self._jamming_direction_initialized: bool = False

    # ------------------------------------------------------------------
    # 感知屏蔽相关
    # ------------------------------------------------------------------
    def init_jamming_direction(self, base_position: List[float]) -> None:
        """在生成后由外部调用一次，根据基地位置固化屏蔽方向。

        base_position: [x, y, z] – 防守方基地坐标
        """
        if self._jamming_direction_initialized:
            return
        dx = self.position[0] - base_position[0]
        dy = self.position[1] - base_position[1]
        self.jamming_direction = math.atan2(dy, dx)
        self._jamming_direction_initialized = True

    @property
    def jamming_active(self) -> bool:
        """屏蔽是否活跃：存活且方向已初始化。"""
        return self.is_alive() and self._jamming_direction_initialized

    def is_in_jammed_sector(
        self,
        point: List[float],
        base_position: List[float],
    ) -> bool:
        """判断 point 是否处于本机屏蔽扇区内。

        扇区定义：以基地为原点，中心方向为 self.jamming_direction，
        半角为 self.jamming_sector_half_angle。

        返回 True 表示该点处于屏蔽范围内，防守方不可见。
        """
        if not self.jamming_active or self.jamming_direction is None:
            return False

        dx = point[0] - base_position[0]
        dy = point[1] - base_position[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return False   # 就是基地本身

        point_angle = math.atan2(dy, dx)
        # 计算角度差，归一化到 [-pi, pi]
        diff = point_angle - self.jamming_direction
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return abs(diff) <= self.jamming_sector_half_angle

    # ------------------------------------------------------------------
    # 自毁逻辑
    # ------------------------------------------------------------------
    def check_self_destruct(self, base_position: List[float]) -> bool:
        """检查是否应自行坠毁。若距离基地小于阈值则触发自毁，返回 True。"""
        if not self.is_alive():
            return False
        dx = self.position[0] - base_position[0]
        dy = self.position[1] - base_position[1]
        dz = self.position[2] - base_position[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < self.self_destruct_distance:
            self.death_cause = "self_destruct"
            self.apply_damage(self.health)
            return True
        return False

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    @staticmethod
    def compute_jamming_direction_from_position(
        position: List[float],
        base_position: List[float],
    ) -> float:
        """工具函数：根据位置计算屏蔽方向（弧度）。"""
        dx = position[0] - base_position[0]
        dy = position[1] - base_position[1]
        return math.atan2(dy, dx)
