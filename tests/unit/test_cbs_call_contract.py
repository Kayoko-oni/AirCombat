"""main.py 与 cbs_pathplan 之间调用契约的测试。

背景：``cbs_plan_paths`` 在 ``len(agents) > max_agents`` 时直接返回空列表。
main.py 若按全量 pairs 调用，CBS 会整体被丢弃（实测 55% 的调用如此），
因此必须先把 pairs 截断到 ``_CBS_MAX_AGENTS``。本测试锁定该不变量。
"""

import inspect
import unittest

import main
from algorithms.cbs_pathplan import cbs_plan_paths


class CbsAgentCapContractTest(unittest.TestCase):
    """截断上限必须与规划器的 max_agents 语义保持一致。"""

    def test_cap_does_not_exceed_planner_default(self) -> None:
        """截断值超过规划器默认值时，CBS 仍会整体返回空列表。"""
        planner_default = inspect.signature(cbs_plan_paths).parameters["max_agents"].default

        self.assertLessEqual(main._CBS_MAX_AGENTS, planner_default)

    def test_cap_is_positive(self) -> None:
        """截断值必须为正，否则不会规划任何 agent。"""
        self.assertGreater(main._CBS_MAX_AGENTS, 0)

    def test_planner_returns_padded_partial_for_oversized_batch(self) -> None:
        """新 CBS 对超限 batch 返回对齐的部分解，尾部为空路径。"""
        class _FakeMapGrid:
            width = 10
            height = 10
            cell_size = 1.0

            @staticmethod
            def is_grid_occupied(x: int, y: int) -> bool:
                return False

            @staticmethod
            def world_to_grid(x: float, y: float):
                return int(x), int(y)

            @staticmethod
            def grid_to_world(gx: int, gy: int):
                return float(gx), float(gy)

        agents = [([0, 0], [5, 5]) for _ in range(main._CBS_MAX_AGENTS + 1)]

        result = cbs_plan_paths(agents, map_grid=_FakeMapGrid(), max_agents=main._CBS_MAX_AGENTS)

        self.assertEqual(len(result), len(agents))
        self.assertEqual(result[-1], [])


if __name__ == "__main__":
    unittest.main()
