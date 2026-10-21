"""main.py 中任务分配节流逻辑的单元测试。

覆盖点：
- 首次调用（无缓存）必须重算
- 间隔内且集合/存活状态不变时复用缓存
- 间隔到期强制重算
- 目标被摧毁立即重算
- 防守方集合变化立即重算
- 进攻方集合变化立即重算
- 重置接口清空缓存
"""

import unittest

import main


class FakeDrone:
    """最小化无人机替身：只提供节流判定用到的 ``name`` 与 ``is_alive``。"""

    def __init__(self, name: str, alive: bool = True) -> None:
        self.name = name
        self._alive = alive

    def is_alive(self) -> bool:
        """返回存活状态。"""
        return self._alive

    def kill(self) -> None:
        """模拟被摧毁。"""
        self._alive = False


class AssignmentThrottleTest(unittest.TestCase):
    """``_assignment_needs_refresh`` 的行为契约测试。"""

    def setUp(self) -> None:
        main._reset_assignment_throttle()
        self.defenders = [FakeDrone("D-1"), FakeDrone("D-2")]
        self.attackers = [FakeDrone("A-1"), FakeDrone("A-2")]

    def _prime(self, now: float) -> None:
        """写入一份有效缓存，模拟刚完成一次重算。

        Args:
            now: 伪造的重算完成时刻。
        """
        main._assignment_cache["assignment"] = {
            "D-1": self.attackers[0],
            "D-2": self.attackers[1],
        }
        main._assignment_cache["time"] = now
        main._assignment_cache["defender_names"] = frozenset(d.name for d in self.defenders)
        main._assignment_cache["attacker_names"] = frozenset(a.name for a in self.attackers)

    def test_first_call_requires_refresh(self) -> None:
        """无缓存时应重算。"""
        needed = main._assignment_needs_refresh(self.defenders, self.attackers, now=100.0)

        self.assertTrue(needed)

    def test_reuses_cache_within_interval(self) -> None:
        """间隔内且无事件时复用缓存。"""
        self._prime(now=100.0)

        needed = main._assignment_needs_refresh(
            self.defenders, self.attackers, now=100.1, min_interval=0.25
        )

        self.assertFalse(needed)

    def test_refresh_after_interval_elapsed(self) -> None:
        """超过最小间隔后强制重算（时间兜底）。"""
        self._prime(now=100.0)

        needed = main._assignment_needs_refresh(
            self.defenders, self.attackers, now=100.26, min_interval=0.25
        )

        self.assertTrue(needed)

    def test_refresh_when_cached_target_dies(self) -> None:
        """缓存中的目标被摧毁时立即重算。"""
        self._prime(now=100.0)
        self.attackers[0].kill()

        needed = main._assignment_needs_refresh(
            self.defenders, self.attackers, now=100.05, min_interval=0.25
        )

        self.assertTrue(needed)

    def test_refresh_when_defender_set_changes(self) -> None:
        """防守方集合变化（新补充）时立即重算。"""
        self._prime(now=100.0)
        defenders = self.defenders + [FakeDrone("D-3")]

        needed = main._assignment_needs_refresh(
            defenders, self.attackers, now=100.05, min_interval=0.25
        )

        self.assertTrue(needed)

    def test_refresh_when_attacker_set_changes(self) -> None:
        """进攻方集合变化（新增目标）时立即重算。"""
        self._prime(now=100.0)
        attackers = self.attackers + [FakeDrone("A-3")]

        needed = main._assignment_needs_refresh(
            self.defenders, attackers, now=100.05, min_interval=0.25
        )

        self.assertTrue(needed)

    def test_reset_clears_cache(self) -> None:
        """重置后应回到「无缓存」状态。"""
        self._prime(now=100.0)

        main._reset_assignment_throttle()
        needed = main._assignment_needs_refresh(
            self.defenders, self.attackers, now=100.05, min_interval=0.25
        )

        self.assertTrue(needed)


if __name__ == "__main__":
    unittest.main()
