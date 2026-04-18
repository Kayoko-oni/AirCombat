from typing import List

from drones.base_drone import BaseDrone
from utils.geometry import distance


class RadarSensor:
    def __init__(self, range_m: float, pulse_interval: float = 1.0):
        """当一个雷达实例被创建时, 自动调用此init构造函数, 初始化成员变量:range_m, pulse_interval"""
        self.range_m = range_m
        self.pulse_interval = pulse_interval
    #   .range_m的值为传入的range_m的值, 为雷达的探测半径, 单位为米
    #   .pulse_interval的值为传入的pulse_interval的值, 为雷达的探测周期, 经过多少时间探测一次

    def scan(self, drones: List[BaseDrone]) -> List[dict]:
        """ 输入一个无人机列表, 返回这个无人机列表中被探测到的无人机的信息的字典列表 """
        detections = []
        for drone in drones:
              #遍历传入的无人机列表, 如果雷达与无人机的距离小于雷达的探测半径, 则认为此无人机被探测到了, 将其信息加入输出列表 
              #此处雷达的坐标放置在[0, 0, 0], 也就是基地处, 如果之后想设置不同地点的雷达, 可以在构造函数中加入雷达的坐标, 再在此处调用
    
            if distance([0, 0, 0], drone.position) <= self.range_m:
                detections.append(
                    {
                        "name": drone.name,
                        "type": drone.drone_type,
                        "position": drone.position,
                        "health": drone.health,
                    }
                )
                #返回输出列表
        return detections
