from typing import List, Tuple


def distance(a: List[float], b: List[float]) -> float:
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def clamp_position(position: List[float], min_bounds: Tuple[float, float, float], max_bounds: Tuple[float, float, float]) -> List[float]:
    return [
        max(min(position[i], max_bounds[i]), min_bounds[i])
        for i in range(len(position))
    ]
