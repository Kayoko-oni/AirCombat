import math
from typing import List, Tuple

_EARTH_RADIUS_M = 6_371_000.0


def distance(a: List[float], b: List[float]) -> float:
    """ 传入两个代表坐标点的列表, 计算两点之间的距离 """
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def geodetic_to_local_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
    origin_alt_m: float = 0.0,
) -> List[float]:
    """将经纬度(度)与高度(米)转换为局部 ENU 平面坐标(米)。

    采用等距矩形近似，适用于小范围任务区域。
    """
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0 = math.radians(origin_lat_deg)
    lon0 = math.radians(origin_lon_deg)
    d_lat = lat - lat0
    d_lon = lon - lon0

    x = _EARTH_RADIUS_M * d_lon * math.cos(lat0)
    y = _EARTH_RADIUS_M * d_lat
    z = alt_m - origin_alt_m
    return [x, y, z]


def clamp_position(position: List[float], min_bounds: Tuple[float, float, float], max_bounds: Tuple[float, float, float]) -> List[float]:
    """ 
    传入一个需要判断的坐标点、地图范围最小值, 地图范围最大值,根据场地范围判断无人机是否超界, 不允许坐标点超出设定的地图范围
    """
    #在 controller —— single_control —— move_drone 里面被调用, 具体的地图范围也在被调用处给出
    return [
        max(min(position[i], max_bounds[i]), min_bounds[i])
        for i in range(len(position))
    ]
