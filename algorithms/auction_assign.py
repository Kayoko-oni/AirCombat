from typing import List, Dict, Optional
from drones.base_drone import BaseDrone
from utils.geometry import distance


def auction_assign(drones: List[BaseDrone], tasks: List[dict]) -> dict:
    assignment = {}
    for index, drone in enumerate(drones):
        assignment[drone.name] = tasks[index % len(tasks)] if tasks else None
    return assignment

def greedy_assignment(defenders: List[BaseDrone], attackers: List[BaseDrone]) -> Dict[str, Optional[BaseDrone]]:
    """
    贪婪分配：返回字典 {防守方.name: 进攻方对象或None}
    """
    remaining_def = defenders.copy()
    remaining_att = attackers.copy()
    assignment = {}

    while remaining_def and remaining_att:
        min_dist = float('inf')
        best_def = None
        best_att = None
        for d in remaining_def:
            for a in remaining_att:
                dist = distance(d.position, a.position)
                if dist < min_dist:
                    min_dist = dist
                    best_def = d
                    best_att = a
        assignment[best_def.name] = best_att
        remaining_def.remove(best_def)
        remaining_att.remove(best_att)

    for d in remaining_def:
        assignment[d.name] = None
    return assignment