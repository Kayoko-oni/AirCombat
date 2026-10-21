"""CBS 失败熔断与轨迹线向量化的单元测试。"""

import unittest

import numpy as np

import main

try:  # open3d 在无显示环境下也可能可用，缺失时跳过相关用例
    import open3d  # noqa: F401

    _HAS_OPEN3D = True
except ImportError:  # pragma: no cover - 取决于运行环境
    _HAS_OPEN3D = False


class CbsFailureBackoffTest(unittest.TestCase):
    """``_note_cbs_result`` / ``_cbs_plan_allowed`` 的熔断契约。"""

    def setUp(self) -> None:
        main._reset_cbs_throttle()

    def test_backoff_blocks_planning_after_consecutive_failures(self) -> None:
        """连续失败达到阈值后，退避期内不允许再次规划。"""
        for _ in range(main._CBS_MAX_CONSECUTIVE_FAILURES):
            main._note_cbs_result(False, now=100.0)

        allowed = main._cbs_plan_allowed(now=100.0, min_interval=0.0)

        self.assertFalse(allowed)

    def test_backoff_expires_after_window(self) -> None:
        """退避窗口结束后恢复允许规划。"""
        for _ in range(main._CBS_MAX_CONSECUTIVE_FAILURES):
            main._note_cbs_result(False, now=100.0)

        allowed = main._cbs_plan_allowed(
            now=100.0 + main._CBS_FAILURE_BACKOFF_SECONDS + 0.01,
            min_interval=0.0,
        )

        self.assertTrue(allowed)

    def test_success_resets_failure_counter(self) -> None:
        """中途成功一次会清零计数，避免误熔断。"""
        main._note_cbs_result(False, now=100.0)
        main._note_cbs_result(False, now=100.0)
        main._note_cbs_result(True, now=100.0)
        main._note_cbs_result(False, now=100.0)

        allowed = main._cbs_plan_allowed(now=100.0, min_interval=0.0)

        self.assertTrue(allowed)

    def test_reset_clears_backoff(self) -> None:
        """重置接口清空计数与退避窗口。"""
        for _ in range(main._CBS_MAX_CONSECUTIVE_FAILURES):
            main._note_cbs_result(False, now=100.0)

        main._reset_cbs_throttle()
        allowed = main._cbs_plan_allowed(now=100.0, min_interval=0.0)

        self.assertTrue(allowed)


@unittest.skipUnless(_HAS_OPEN3D, "open3d 未安装，跳过渲染几何测试")
class CreatePathLineVectorizationTest(unittest.TestCase):
    """向量化后的 ``create_path_line`` 与旧实现的语义等价性。"""

    def test_consecutive_duplicates_are_removed(self) -> None:
        """相邻重复点应被剔除，仅保留有效顶点。"""
        from Visual.render_utils import create_path_line

        points = [[0, 0, 0], [0, 0, 0], [10, 0, 0], [10, 0, 0], [20, 0, 0]]

        line_set = create_path_line(points)

        self.assertEqual(len(line_set.points), 3)

    def test_color_fade_matches_original_formula(self) -> None:
        """颜色渐隐仍按 0.3 + 0.7*(i/total) 计算。"""
        from Visual.render_utils import create_path_line

        points = [[0, 0, 0], [10, 0, 0], [20, 0, 0], [30, 0, 0]]
        color = (1.0, 0.0, 0.0)

        line_set = create_path_line(points, color=color)
        colors = np.asarray(line_set.colors)

        self.assertEqual(len(colors), 3)
        expected_first = 0.3 + 0.7 * (1 / 3)
        expected_last = 1.0
        self.assertAlmostEqual(colors[0][0], expected_first, places=6)
        self.assertAlmostEqual(colors[-1][0], expected_last, places=6)

    def test_too_few_points_returns_none(self) -> None:
        """点数不足 2 时返回 None（边界分支）。"""
        from Visual.render_utils import create_path_line

        self.assertIsNone(create_path_line([[0, 0, 0]]))
        self.assertIsNone(create_path_line([[0, 0, 0], [0, 0, 0]]))


if __name__ == "__main__":
    unittest.main()
