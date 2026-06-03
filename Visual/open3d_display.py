from typing import List
import numpy as np

import yaml
import os
from utils.obj_voxelizer import voxelize_obj
from utils.map_grid import MapGrid

from utils.map_loader import generate_buildings
from drones.base_drone import BaseDrone
from Visual.render_utils import (
    create_drone_mesh,
    create_path_line,
    create_explosion_mesh,
    create_drone_model_mesh,
    create_guts_base_mesh,
    create_dashed_line,
)


class Open3DDisplay:
    def __init__(self, map_size=(1000, 1000), obj_model_path=None):
        try:
            import open3d as o3d
            from open3d.visualization import gui, rendering
        except ImportError as exc:
            raise ImportError(
                "Open3D is required for visualization. Install it with `pip install open3d`."
            ) from exc

        self.o3d = o3d
        self.gui = gui
        self.rendering = rendering
        self.map_size = map_size
        self.app = self.gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("AirCombat", 1024, 768)
        self.scene_widget = self.gui.SceneWidget()
        self.scene_widget.scene = self.rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.scene_widget)

        # ── 美化 HUD：多标签堆叠面板（纯 ASCII，无 Unicode 方框）──
        # 颜色常量
        self._hud_bg   = self.gui.Color(0.03, 0.03, 0.08, 0.80)  # 深色面板背景
        self._hud_white   = self.gui.Color(0.95, 0.95, 0.97, 1.0)  # 主文字
        self._hud_dim     = self.gui.Color(0.50, 0.50, 0.55, 1.0)  # 弱化文字
        self._hud_green   = self.gui.Color(0.15, 0.90, 0.25, 1.0)  # 血量高
        self._hud_orange  = self.gui.Color(1.00, 0.65, 0.10, 1.0)  # 血量中 / EW 警告
        self._hud_red     = self.gui.Color(0.95, 0.20, 0.20, 1.0)  # 血量低 / 危险
        self._hud_purple  = self.gui.Color(0.65, 0.20, 0.85, 1.0)  # Attack
        self._hud_yellow  = self.gui.Color(1.00, 0.85, 0.10, 1.0)  # Tank
        self._hud_blue    = self.gui.Color(0.15, 0.60, 1.00, 1.0)  # EW
        self._hud_darkred = self.gui.Color(0.90, 0.10, 0.25, 1.0)  # ARD
        self._hud_emerald = self.gui.Color(0.15, 0.80, 0.20, 1.0)  # Scout
        self._hud_cyan    = self.gui.Color(0.15, 0.80, 0.95, 1.0)  # Interceptor
        self._hud_brown   = self.gui.Color(1.00, 0.40, 0.10, 1.0)  # AntiEW

        # HUD 布局常量
        self._hud_x = 16
        self._hud_y0 = 12
        self._hud_w = 440
        self._hud_row_h = 22
        self._hud_gap = 10

        # 所有 HUD 标签行
        self._hud_labels = {}
        self._hud_rows = {
            "title":    0,
            "offense":  2,
            "defense":  3,
            "status":   5,
            "hp":       7,
            "legend_off": 9,
            "legend_def": 10,
            "controls": 12,
        }
        for key in self._hud_rows:
            lbl = self.gui.Label("")
            lbl.background_color = self._hud_bg
            self.window.add_child(lbl)
            self._hud_labels[key] = lbl

        # FPS 追踪
        self._fps_frame_count = 0
        self._fps_accum_time = 0.0
        self._fps_current = 0.0
        self._sim_elapsed = 0.0

        # Set scene widget to fill the window
        self.scene_widget.frame = self.gui.Rect(0, 0, self.window.content_rect.width, self.window.content_rect.height)

        self.is_open = True
        self.paused = False

        self.show_paths = False   # True=显示避障路径, False=不显示
        self.static_objects_added = False      # 静态物体是否已添加
        self.dynamic_geometries = set()        # 存储动态几何体的名称
        self.map_initialized = False           # 地图是否已初始化（用于首次更新时添加建筑）

        self.obj_model_path = obj_model_path
        self.city_mesh = None

        self.map_data_initialized = False   # 地图数据是否已生成
        self.buildings = None               # 建筑列表
        self.map_grid = None                # 障碍物网格对象

        self.offensive_types = {"AttackDrone", "TankDrone", "AntiRadiationDrone", "EWDrone"}
        self.drone_color_map = {
            "AttackDrone": (0.65, 0.2, 0.85),
            "TankDrone": (1.0, 0.85, 0.1),
            "AntiRadiationDrone": (0.9, 0.15, 0.3),
            "EWDrone": (0.1, 0.6, 1.0),
            "ScoutDrone": (0.2, 0.8, 0.2),
            "InterceptorDrone": (0.2, 0.8, 0.95),
            "AntiEWDrone": (1.0, 0.4, 0.1),
        }
        self.drone_color_names = {
            "AttackDrone": "purple",
            "TankDrone": "yellow",
            "AntiRadiationDrone": "deep-red",
            "EWDrone": "blue",
            "ScoutDrone": "green",
            "InterceptorDrone": "cyan",
            "AntiEWDrone": "orange",
        }

        # Set up camera
        bounds = self.o3d.geometry.AxisAlignedBoundingBox(
            np.array([-1000, -1000, -100], dtype=np.float64),
            np.array([1000, 1000, 100], dtype=np.float64),
        )
        center = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.scene_widget.setup_camera(45.0, bounds, center)
        self.scene_widget.set_view_controls(self.gui.SceneWidget.Controls.ROTATE_CAMERA)

    def _mouse_callback(self, event):
        if event.type == self.gui.MouseEvent.Type.BUTTON_DOWN and event.is_button_down(self.gui.MouseButton.RIGHT):
            bounds = self.o3d.geometry.AxisAlignedBoundingBox(
                np.array([-1000, -1000, -100], dtype=np.float64),
                np.array([1000, 1000, 100], dtype=np.float64),
            )
            center = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            self.scene_widget.setup_camera(45.0, bounds, center)
            return self.gui.SceneWidget.EventCallbackResult.CONSUMED
        return self.gui.SceneWidget.EventCallbackResult.IGNORED

    def _key_callback(self, event):
        """键盘快捷键：R=重置视角 P=暂停 T=切换路径显示"""
        if event.type == self.gui.KeyEvent.Type.DOWN:
            key = event.key
            if key == ord('R') or key == ord('r'):
                bounds = self.o3d.geometry.AxisAlignedBoundingBox(
                    np.array([-1000, -1000, -100], dtype=np.float64),
                    np.array([1000, 1000, 100], dtype=np.float64),
                )
                center = np.array([0.0, 0.0, 0.0], dtype=np.float32)
                self.scene_widget.setup_camera(45.0, bounds, center)
                return self.gui.SceneWidget.EventCallbackResult.CONSUMED
            elif key == ord('P') or key == ord('p'):
                self.paused = not self.paused
                print(f"[HUD] Paused: {self.paused}")
                return self.gui.SceneWidget.EventCallbackResult.CONSUMED
            elif key == ord('T') or key == ord('t'):
                self.show_paths = not self.show_paths
                print(f"[HUD] Show paths: {self.show_paths}")
                return self.gui.SceneWidget.EventCallbackResult.CONSUMED
        return self.gui.SceneWidget.EventCallbackResult.IGNORED

    def open_window(self):
        self.scene_widget.set_on_mouse(self._mouse_callback)
        self.scene_widget.set_on_key(self._key_callback)
        self.window.set_on_layout(self._on_layout)
        self.window.show(True)

    def _add_ground_plane(self):
        ground = self.o3d.geometry.TriangleMesh.create_box(
            width=self.map_size[0],
            height=self.map_size[1],
            depth=0.1,
        )
        ground.translate([-self.map_size[0] / 2, -self.map_size[1] / 2, -0.1])
        ground.compute_vertex_normals()
        ground.paint_uniform_color((0.92, 0.92, 0.92))
        material = self.rendering.MaterialRecord()
        material.base_color = (0.92, 0.92, 0.92, 0.95)
        self.scene_widget.scene.add_geometry("ground_plane", ground, material)

    def _add_static_objects(self, base_position):  #只在首次更新时添加静态物体（坐标轴、地面、基地模型、建筑）
        # 坐标轴
        coord_frame = self.o3d.geometry.TriangleMesh.create_coordinate_frame(size=60.0)
        coord_frame.compute_vertex_normals()
        self.scene_widget.scene.add_geometry("coord_frame", coord_frame, self.rendering.MaterialRecord())
        # 地面
        self._add_ground_plane()
        # 基地模型
        self._add_base_model(base_position)
        #添加建筑物
        self._add_buildings()

        self.static_objects_added = True

    def _add_base_model(self, base_position):
        """在原点添加基地模型（灰色长方体+绿色球体）"""
        # 使用我们之前定义的 create_guts_base_mesh 函数
        from Visual.render_utils import create_guts_base_mesh

        base_mesh = create_guts_base_mesh(base_position)
        material = self.rendering.MaterialRecord()
        material.shader = 'defaultLit'
        self.scene_widget.scene.add_geometry("base_model", base_mesh, material)
        

    def _add_buildings(self):
        if self.city_mesh is not None:
            material = self.rendering.MaterialRecord()
            material.shader = 'defaultLit'
            material.base_color = (0.7, 0.7, 0.7, 1.0)
            self.scene_widget.scene.add_geometry("city_buildings", self.city_mesh, material)
            print("[Map] 显示 OBJ 城市模型")
        elif self.buildings is not None:
            combined = self.o3d.geometry.TriangleMesh()
            for center, size in self.buildings:
                cx, cy, cz = center
                sx, sy, sz = size
                box = self.o3d.geometry.TriangleMesh.create_box(width=sx, height=sy, depth=sz)
                box.translate([cx - sx/2, cy - sy/2, cz - sz/2])
                box.paint_uniform_color((0.8, 0.8, 0.8))
                combined += box
            combined.compute_vertex_normals()
            material = self.rendering.MaterialRecord()
            material.shader = 'defaultLit'
            material.base_color = (0.8, 0.8, 0.8, 0.92)
            self.scene_widget.scene.add_geometry("city_buildings", combined, material)
            print(f"[Map] 显示随机建筑: {len(self.buildings)} 个立方体")
            
    def _init_map_data(self):
        if self.map_data_initialized:
            return
        if self.obj_model_path and os.path.exists(self.obj_model_path):
            mesh, obs_grid, obs_height, cell_size, bounds = voxelize_obj(self.obj_model_path)
            self.city_mesh = mesh
            self.map_grid = MapGrid.from_arrays(obs_grid, obs_height, cell_size, bounds)
            print(f"[Map] 使用 OBJ 模型体素化，网格 {obs_grid.shape}")
        else:
            from utils.map_loader import generate_buildings
            self.buildings = generate_buildings()
            self.map_grid = MapGrid(self.buildings, cell_size=5.0)
            print("[Map] 使用随机生成建筑")
        self.map_data_initialized = True

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.scene_widget.frame = self.gui.Rect(0, 0, r.width, r.height)
        return self.gui.Widget.EventCallbackResult.HANDLED

    def _color_for_type(self, drone_type: str):
        return self.drone_color_map.get(drone_type, (0.7, 0.7, 0.7))

    def _drone_color(self, drone: BaseDrone):
        return self._color_for_type(drone.drone_type)

    def _label_text(self, drone: BaseDrone, nearest_dist: float):
        return f"{drone.name} ({drone.drone_type})\nD={nearest_dist:.1f}"

    def _nearest_enemy_distance(self, drone: BaseDrone, others: List[BaseDrone]) -> float:
        best = float("inf")
        for other in others:
            if other is drone or not other.is_alive():
                continue
            dist = sum((p - q) ** 2 for p, q in zip(drone.position, other.position)) ** 0.5
            if dist < best:
                best = dist
        return best if best != float("inf") else 0.0

    # ── 美化版 HUD：多标签行，纯 ASCII ──

    @staticmethod
    def _bar_ascii(pct: float, width: int = 16) -> str:
        """ASCII 进度条：使用 # 和 - 字符，例如 [############----]"""
        pct = max(0.0, min(100.0, pct))
        filled = max(0, min(width, int(round(pct / 100.0 * width))))
        return "[" + "#" * filled + "-" * (width - filled) + "]"

    def _refresh_hud(self, drones: list, base_health: float) -> None:
        """每帧刷新所有 HUD 行标签的文本与颜色。"""
        active = [d for d in drones if not d.destroyed]
        tc = {t: sum(1 for d in active if d.drone_type == t) for t in self.drone_color_map}

        # EW 状态
        ew_active = any(getattr(d, "jamming_active", False) for d in drones
                        if d.drone_type == "EWDrone" and d.is_alive())
        jammed_count = sum(1 for d in drones
                           if d.drone_type not in self.offensive_types
                           and getattr(d, "_ew_jammed", False) and d.is_alive())

        # FPS + 时间
        fps_str = f"{self._fps_current:5.1f}"
        mins = int(self._sim_elapsed // 60)
        secs = int(self._sim_elapsed % 60)
        time_str = f"{mins:02d}:{secs:02d}"

        # ── 设置每行文字与颜色 ──
        labels = self._hud_labels

        # 标题行
        labels["title"].text = "===  AIR  COMBAT  SIMULATION  ==="
        labels["title"].text_color = self._hud_white

        # 进攻方行
        off_parts = f"Attack: {tc['AttackDrone']}    Tank: {tc['TankDrone']}    ARD: {tc['AntiRadiationDrone']}    EW: {tc['EWDrone']}"
        labels["offense"].text = off_parts
        labels["offense"].text_color = self._hud_orange

        # 防守方行
        def_parts = f"Scout: {tc['ScoutDrone']}    Inter: {tc['InterceptorDrone']}    AEW: {tc['AntiEWDrone']}"
        labels["defense"].text = def_parts
        labels["defense"].text_color = self._hud_cyan

        # 状态行
        ew_text = "ACTIVE" if ew_active else "none"
        jam_text = str(jammed_count) if jammed_count > 0 else "none"
        status_text = f"Offense: {sum(tc[t] for t in ['AttackDrone','TankDrone','AntiRadiationDrone','EWDrone'])}    Defense: {sum(tc[t] for t in ['ScoutDrone','InterceptorDrone','AntiEWDrone'])}    EW: {ew_text}    Jammed: {jam_text}"
        labels["status"].text = status_text
        labels["status"].text_color = self._hud_dim

        # 血量条行
        bar = self._bar_ascii(base_health)
        hp_text = f"Base HP  {bar}  {base_health:5.1f}%"
        labels["hp"].text = hp_text
        if base_health > 60:
            labels["hp"].text_color = self._hud_green
        elif base_health > 30:
            labels["hp"].text_color = self._hud_orange
        else:
            labels["hp"].text_color = self._hud_red

        # 图例行 - 进攻方
        labels["legend_off"].text = "* Attack = purple    Tank = yellow    ARD = red    EW = blue"
        labels["legend_off"].text_color = self._hud_dim

        # 图例行 - 防守方
        labels["legend_def"].text = "+ Scout = green    Inter = cyan    AEW = orange"
        labels["legend_def"].text_color = self._hud_dim

        # 控制行
        ctrl_text = f"FPS: {fps_str}    Time: {time_str}    [R] Reset    [P] Pause    [T] Paths"
        labels["controls"].text = ctrl_text
        labels["controls"].text_color = self._hud_dim

        # ── 更新每行的 frame 位置 ──
        row_h = self._hud_row_h
        w = self._hud_w
        x = self._hud_x
        for key, row_idx in self._hud_rows.items():
            y = self._hud_y0 + row_idx * (row_h + self._hud_gap)
            self._hud_labels[key].frame = self.gui.Rect(x, y, w, row_h)

    # 保留旧名兼容
    def _status_text(self, drones, base_health):
        return " "  # 不再使用单一大标签

    def _safe_add_dynamic_geometry(self, name: str, geometry, material) -> bool:
        """Add dynamic geometry defensively to avoid GUI crashes from invalid shapes."""
        if geometry is None:
            return False
        try:
            self.scene_widget.scene.add_geometry(name, geometry, material)
            self.dynamic_geometries.add(name)
            return True
        except Exception as exc:
            print(f"[WARN] Skip geometry {name}: {exc}")
            return False

    def update(self, drones: List[BaseDrone], detections, base_health: float, base_position):
        if not self.is_open:
            return

        if not self.map_initialized:
            self._init_map_data()   # 只执行一次，生成 self.buildings 和 self.map_grid

        if not self.static_objects_added:  # 只在首次更新时添加静态物体（坐标轴、地面、基地模型）
            self._add_static_objects(base_position)

        for name in list(self.dynamic_geometries):
            try:
                self.scene_widget.scene.remove_geometry(name)
            except Exception:
                pass
        self.dynamic_geometries.clear()

        offensive = [d for d in drones if d.drone_type in self.offensive_types and not d.destroyed]
        defensive = [d for d in drones if d.drone_type not in self.offensive_types and not d.destroyed]

        for drone in drones:
            if drone.destroyed:
                if drone.impact:
                    explosion_color = self._color_for_type(drone.drone_type)
                    debris = create_explosion_mesh(drone.position, drone.death_timer, drone.death_effect_duration, color=explosion_color)
                    material = self.rendering.MaterialRecord()
                    material.base_color = explosion_color + (1.0,)
                    self._safe_add_dynamic_geometry(f"explosion_{drone.name}", debris, material)
                else:
                    # 若为碰撞导致的坠毁，使用灰色无人机模型和灰色轨迹；否则沿用原有球体与颜色
                    if getattr(drone, "death_cause", None) == "collision":
                        fall_color = (0.8, 0.8, 0.8)
                        mesh = create_drone_model_mesh(drone.position, color=fall_color)
                    else:
                        fall_color = self._color_for_type(drone.drone_type)
                        mesh = create_drone_mesh(drone.position, color=fall_color)

                    material = self.rendering.MaterialRecord()
                    material.base_color = fall_color + (1.0,)
                    self._safe_add_dynamic_geometry(f"falling_{drone.name}", mesh, material)
                    if len(drone.trail) > 1:
                        path = create_path_line(drone.trail, color=fall_color)
                        if path is not None:
                            material_trail = self.rendering.MaterialRecord()
                            material_trail.shader = 'unlitLine'
                            material_trail.base_color = fall_color + (1.0,)
                            self._safe_add_dynamic_geometry(f"trail_{drone.name}", path, material_trail)
                continue

            color = self._drone_color(drone)
            mesh = create_drone_model_mesh(drone.position, color=color)
            material = self.rendering.MaterialRecord()
            material.base_color = color + (1.0,)
            # TODO: 增加视觉效果
            self._safe_add_dynamic_geometry(f"drone_{drone.name}", mesh, material)

            if len(drone.trail) > 1:
                trail_color = self._color_for_type(drone.drone_type)
                path = create_path_line(drone.trail, color=trail_color)
                if path is not None:
                    material_trail = self.rendering.MaterialRecord()
                    material_trail.shader = 'unlitLine'
                    material_trail.base_color = trail_color + (1.0,)
                    self._safe_add_dynamic_geometry(f"trail_{drone.name}", path, material_trail)
            # 如果存在避障路径缓存（来自 PathTracker 或 CBS），可视化显示
            # 优先显示防守方的 CBS 路径 (_cbs_path)（红色），其次显示个人避障路径 (_avoid_path)（绿色）
            if self.show_paths:
                try:
                    if hasattr(drone, "_cbs_path") and drone._cbs_path:
                        # 使用紫色来区分于进攻方轨迹（红色）
                        p = create_path_line(drone._cbs_path, color=(0.6, 0.2, 0.8))
                        if p is not None:
                            material_p = self.rendering.MaterialRecord()
                            material_p.shader = 'unlitLine'
                            material_p.base_color = (0.6, 0.2, 0.8, 1.0)
                            self._safe_add_dynamic_geometry(f"path_{drone.name}", p, material_p)
                    elif hasattr(drone, "_avoid_path") and drone._avoid_path:
                        p = create_path_line(drone._avoid_path, color=(0.2, 0.8, 0.2))
                        if p is not None:
                            material_p = self.rendering.MaterialRecord()
                            material_p.shader = 'unlitLine'
                            material_p.base_color = (0.2, 0.8, 0.2, 1.0)
                            self._safe_add_dynamic_geometry(f"path_{drone.name}", p, material_p)
                except Exception:
                    pass
            # 显示防守分配连线：若 defender 有 `_assigned_target` 属性，则用虚线显示到目标
            try:
                assigned = getattr(drone, "_assigned_target", None)
                if assigned is not None and hasattr(assigned, "position") and getattr(assigned, "is_alive", lambda: True)():
                    dash = create_dashed_line(drone.position, assigned.position, color=(1.0, 0.8, 0.0))
                    if dash is not None:
                        material_assign = self.rendering.MaterialRecord()
                        material_assign.shader = 'unlitLine'
                        material_assign.base_color = (1.0, 0.8, 0.0, 1.0)
                        self._safe_add_dynamic_geometry(f"assign_{drone.name}", dash, material_assign)
            except Exception:
                pass

            enemies = defensive if drone in offensive else offensive
            nearest_dist = self._nearest_enemy_distance(drone, enemies)

        # ── FPS 跟踪 ──
        import time as _time
        now = _time.perf_counter()
        if not hasattr(self, '_last_fps_tick'):
            self._last_fps_tick = now
        dt = now - self._last_fps_tick
        self._last_fps_tick = now
        if dt > 0:
            self._fps_accum_time += dt
            self._fps_frame_count += 1
            if self._fps_accum_time >= 0.5:
                self._fps_current = self._fps_frame_count / self._fps_accum_time
                self._fps_frame_count = 0
                self._fps_accum_time = 0.0

        # ── HUD 更新 ──
        if self._fps_current > 0:
            self._sim_elapsed += 1.0 / self._fps_current
        else:
            self._sim_elapsed += 0.016  # ~60fps fallback
        self._refresh_hud(drones, base_health)

        self.scene_widget.force_redraw()
        self.window.post_redraw()

    def close_window(self):
        if not self.is_open:
            return
        try:
            self.window.close()
        finally:
            self.is_open = False
        try:
            # Ensure the GUI thread shuts down before interpreter finalizes.
            self.app.quit()
        except Exception:
            pass
