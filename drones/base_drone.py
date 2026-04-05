from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class BaseDrone:
    """
    无人机基类，定义所有无人机的通用属性和方法。

    属性:
        name (str): 无人机名称
        drone_type (str): 无人机类型
        position (List[float]): 当前位置 [x, y, z]
        velocity (List[float]): 当前速度 [vx, vy, vz]
        health (float): 当前血量
        attack_power (float): 攻击力
        battery (float): 电池剩余量
        max_speed (float): 最大速度限制
    """
    name: str
    drone_type: str
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    health: float = 100.0
    attack_power: float = 0.0
    battery: float = 100.0
    max_speed: float = 20.0
    trail: List[List[float]] = field(default_factory=list)
    destroyed: bool = False
    death_timer: float = 0.0
    death_effect_duration: float = 1.5

    def update_position(self, delta_time: float) -> None:
        """根据当前速度更新位置"""
        if self.destroyed:
            return
        self.position = [p + v * delta_time for p, v in zip(self.position, self.velocity)]
        self.trail.append(self.position.copy())
        if len(self.trail) > 40:
            self.trail.pop(0)
        # TODO: 优化轨迹存储，或许使用deque以提高性能

    def set_velocity(self, velocity: List[float]) -> None:
        """设置速度向量，自动限制在最大速度内"""
        self.velocity = [max(min(v, self.max_speed), -self.max_speed) for v in velocity]

    def apply_damage(self, amount: float) -> None:
        """受到伤害，减少血量，血量清零则坠毁。"""
        if self.destroyed:
            return
        self.health = max(0.0, self.health - amount)
        if self.health <= 0.0:
            self.destroyed = True
            self.velocity = [0.0, 0.0, 0.0]
            self.position[2] = 0.0
            self.death_timer = 0.0
            self.trail.clear()

    def update_death_timer(self, delta_time: float) -> None:
        if self.destroyed:
            self.death_timer += delta_time

    def should_remove(self) -> bool:
        return self.destroyed and self.death_timer >= self.death_effect_duration

    def is_alive(self) -> bool:
        """检查是否还活着（血量 > 0）"""
        return self.health > 0

    def __repr__(self) -> str:
        return f"{self.drone_type}('{self.name}', pos={self.position}, hp={self.health})"
