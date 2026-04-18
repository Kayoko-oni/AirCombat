from typing import List, Tuple


def distance(a: List[float], b: List[float]) -> float:
    """ 传入两个代表坐标点的列表, 计算两点之间的距离 """
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def clamp_position(position: List[float], min_bounds: Tuple[float, float, float], max_bounds: Tuple[float, float, float]) -> List[float]:
    """ 
    传入一个需要判断的坐标点、地图范围最小值, 地图范围最大值,根据场地范围判断无人机是否超界, 不允许坐标点超出设定的地图范围
    """
    #在 controller —— single_control —— move_drone 里面被调用, 具体的地图范围也在被调用处给出
    return [
        max(min(position[i], max_bounds[i]), min_bounds[i])
        for i in range(len(position))
    ]
