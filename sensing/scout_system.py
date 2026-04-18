from typing import List

from drones.base_drone import BaseDrone


class ScoutSystem:
    def __init__(self):
        self.detected = []

    def classify(self, drones: List[BaseDrone]) -> List[dict]:
        """
        接收一个无人机列表, 返回一个字典列表, 每个字典包括一架存活的无人机的信息(名称、类型、是否存活)
            但是此处存在逻辑冗余, 我们只会记录生命值大于0的无人机的数据, 返回值也中再返回是否存活就没有意义了, 必然都是存活的
            此处也要考虑一下, 是否需要顺带返回无人机的position, 便于读取数据
        """
        result = []
        for drone in drones:
            """遍历 作为参数传入的无人机列表 的每一个成员, 记录所有存活的无人机的状态"""
            if drone.health > 0:
                result.append({
                    "name": drone.name,
                    "type": drone.drone_type,
                    "is_alive": drone.is_alive(),
                })
        """
        将result赋值到实例变量self.detected中
        此时接收的无人机列表中, 存活的无人机的信息储存在字典列表self.detected中, 这是此次探测的结果

        目前不知道这段代码能在什么地方被使用, 而且提供了两种方式来输出同一个列表, 可以讨论之后修改
        """
        self.detected = result
        return result
    
    """
    如果要在外部使用此处的代码, 提供以下示例

    from sensing.scout_system import ScoutSystem

    # 创建实例
    scout = ScoutSystem()

    # 假设已有 drones 列表
    drones = [...]  # 一些无人机对象

    # 调用 classify 方法，此时内部会设置 self.detected
    result = scout.classify(drones)

    # 外部可以通过实例变量访问
    cached_result = scout.detected

    # result 和 cached_result 指向同一个列表
    print(result is cached_result)  # True

    """
        
