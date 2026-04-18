from typing import List, Tuple

from drones.base_drone import BaseDrone
from utils.geometry import distance


def detect_collisions(drones: List[BaseDrone], min_distance: float = 8.0) -> List[Tuple[BaseDrone, BaseDrone]]:
    """
    无人机碰撞检测
    参数:
        drones列表, 其元素为一个BaseeDrone类成员
        碰撞距离阈值: 8.0  
        返回一个列表(list), 其元素为一个包含两个无人机对象的元组(tuple)
        也就是说, 这个函数的输入是所有无人机的列表, 输出是发生了碰撞的无人机的组合的列表
    """
    collisions = []
    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            if not drones[i].is_alive() or not drones[j].is_alive():
                continue
            if distance(drones[i].position, drones[j].position) < min_distance:
                collisions.append((drones[i], drones[j]))
    """
    新建一个命名为collisions的空列表
    枚举所有无人机的两两组合
        若两架无人机有一架已经坠毁, 则continue跳过本次循环, 实现: 已经坠毁的无人机不参与碰撞检测
        若两架无人机都存活, 且它们的距离小于碰撞阈值距离, 则认为它们发生了碰撞, 将它们添加到collision碰撞列表中
    返回碰撞列表
    """
    return collisions



def resolve_collisions(collisions: List[Tuple[BaseDrone, BaseDrone]]) -> None:
    """处理发生碰撞的无人机元组"""
    for drone_a, drone_b in collisions:
        relative_speed = sum(abs(a - b) for a, b in zip(drone_a.velocity, drone_b.velocity))
        damage = max(35.0, min(130.0, relative_speed * 3.0))
        drone_a.apply_damage(damage)
        drone_b.apply_damage(damage)
        if drone_a.is_alive():
            drone_a.set_velocity([-v for v in drone_a.velocity])
        if drone_b.is_alive():
            drone_b.set_velocity([-v for v in drone_b.velocity])
    """
    每次取出collisions列表中的一个元组, 将它们的作为循环变量drone_a和drone_b参与循环
        计算两架无人机的相对速度: 近似为 vxa+vya+vza-vxb-vyb-vzb, 做的是标量的计算, 可能需要讨论过后更改
        damege为碰撞造成的伤害, 现在的版本是, 伤害限定在35~130之间, 等于相对速度*3, 这个方法肯定是有问题的, 需要讨论后修改
        调用drones —— base_drone —— apply_damge函数, 让两架无人机受到数值为damege的伤害
        若碰撞后无人机仍然存活, 则设置其速度为碰撞前速度的反向
    """
