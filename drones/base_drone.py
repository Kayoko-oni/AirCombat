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
            代码中的default_factory使得每个实例都有自己的位置与速度向量, 而不会所有实例共用一个列表
        health (float): 当前血量
        attack_power (float): 攻击力
        battery (float): 电池剩余量
        max_speed (float): 最大速度限制
         trail: 轨迹点列表，是由[x,y,z]组成的列表，=list表示, 如果没有给出一个列表, 那么将会创造一个新的空列表给list
        destroy 将无人机是否坠毁设定为bool值, 初始为没有坠毁, bool值为false, 若判定无人机坠毁, 则bool值为true
        death_timer 记录死亡时间，供播放爆炸动画，与移除无人机残骸使用
        death_effect_duration 从判定无人机坠毁，播放爆炸动画，到清除无人机残骸的时间
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
        """
        根据当前速度更新位置
        如果无人机被判定坠毁, 那么直接return, 不再继续调用此函数更新位置
        如果无人机被判定存活, 那么实现 new_position = old_position + velocity * delta_time
                            实现方式: zip(...), 将里面的两个列表, 每次各取出一个元素, 分别赋值给p和v, 每次循环计算 p + v * delta_time, 作为一个新的列表元素赋值给 self.position
        将当前位置副本添加至轨迹列表 避免后续修改是position时影响到现有的点
        当轨迹点列表的元素数量超过40个时 自动移除最旧的一个列表元素 实现列表的更新
        """
        if self.destroyed:
            return
        self.position = [p + v * delta_time for p, v in zip(self.position, self.velocity)]
        self.trail.append(self.position.copy())
        if len(self.trail) > 40:
            self.trail.pop(0)
        # TODO: 优化轨迹存储，或许使用deque以提高性能

    def set_velocity(self, velocity: List[float]) -> None:
        """
        设置速度向量，自动限制在最大速度内
        此处限制速度的逻辑是: xyz三个方向上的速度分别不超过最大值, 这个逻辑可能有些问题，讨论过后可以更改
        """
        self.velocity = [max(min(v, self.max_speed), -self.max_speed) for v in velocity]

    def apply_damage(self, amount: float) -> None:
        """
        无人机碰撞后的生命值削减/坠毁判定程序
        如果无人机已经坠毁(self.destroyed返回初始值True, 那么不重复执行坠毁程序)
        执行生命值削减, 生命值变为 min(0.0, 血量-造成的伤害(对方基础伤害*相对速度)), 因为血量不小于0
        如果无人机生命值归零, 那么执行坠毁程序
            修改destroyed状态为True
            将无人机速度判定为0
            将无人机z坐标直接设定为0 (可能不是很合适, 讨论过后决定是否修改)
            将坠毁计时初始化为0, 供爆炸动画使用
            清除轨迹列表
        """
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
        """
        无人机坠毁后, 对坠毁计时进行更新, 每帧的坠毁计时更新为+= delta_time
        """
        if self.destroyed:
            self.death_timer += delta_time

    def should_remove(self) -> bool:
        """
        判断坠毁后的无人机是否已经可以从场地中清除
        当同时满足以下两个条件时, 认为无人机可以被清除
            无人机坠毁( self.destroy == True)
            坠毁计时 >= 坠毁爆炸特效所需时间 (坠毁爆炸特效已经完成)
        """
        return self.destroyed and self.death_timer >= self.death_effect_duration

    def is_alive(self) -> bool:
        """
        返回无人机存活状态 返回True则存活 返回False则坠毁
        判断依据为此时自身血量是否>0
        """
        return self.health > 0

    def __repr__(self) -> str:
        """
        定义这个类被转换成字符串时应该如何显示
        比如新建了一个对象drone
        可以通过print(drone)来获取这个对象的字符串形式
        """
        return f"{self.drone_type}('{self.name}', pos={self.position}, hp={self.health})"
