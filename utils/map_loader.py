#读取.obj文件，或用文件夹内随机生成地图的方法生成一个地图，是工具类函数

import random
import open3d as o3d
import numpy as np

def generate_buildings():  
    """生成随机城市建筑（不依赖外部文件）"""
    buildings = []
    # 地图范围 -380 到 380
    min_xy, max_xy = -380, 380
    # 建筑数量适中（120~180）
    num = random.randint(180, 240)

    for _ in range(num):
        x = random.uniform(min_xy, max_xy)
        y = random.uniform(min_xy, max_xy)
        # 避开基地附近（原点半径 50 米）
        if abs(x) < 50 and abs(y) < 50:
            continue
        if (x - 75)**2 + (y - 75)**2 < 2500:
            continue
        w = random.uniform(12, 28)
        d = random.uniform(12, 28)
        h = random.uniform(45, 120)
        center = (x, y, h/2)
        size = (w, d, h)
        buildings.append((center, size))

    # 添加四个地标建筑
    landmarks = [(-80, -80, 100), (80, -80, 110), (-80, 80, 95)]
    for x, y, h in landmarks:
        buildings.append(((x, y, h/2), (18, 18, h)))

    print(f"Generated {len(buildings)} buildings.")
    return buildings