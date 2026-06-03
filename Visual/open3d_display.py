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
    def __init__(self, map_size=(1000, 1000), obj_model_path=None, enable_trails=True, enable_dashed_lines=True):
        try:
            import open3d as o3d
            from open3d.visualization import gui, rendering
        except ImportError as exc:
            raise ImportError(
                "Open3D is required for visualization. Install it with `pip install open3d`."
            ) from exc

        self.enable_trails = enable_trails          # 是否生成轨迹线
        self.enable_dashed_lines = enable_dashed_lines  # 是否生成虚线连线

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

        self.drone_geoms = {}          # 存活无人机几何体名称 -> True
        self.active_effects = {}       # 特效信息: key -> dict
        self._effect_handled = set()   # 已经创建过特效的无人机名称（避免重复创建）

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

    def _add_static_objects(self, base_positions):  # 只在首次更新时添加静态物体（坐标轴、地面、基地模型、建筑）
        # 坐标轴
        coord_frame = self.o3d.geometry.TriangleMesh.create_coordinate_frame(size=60.0)
        coord_frame.compute_vertex_normals()
        self.scene_widget.scene.add_geometry("coord_frame", coord_frame, self.rendering.MaterialRecord())
        # 地面
        self._add_ground_plane()
        # 基地模型（支持多个基地）
        if isinstance(base_positions, list) and base_positions:
            for i, bp in enumerate(base_positions):
                self._add_base_model(i, bp)
        else:
            # 兼容旧的 single-base 情形
            self._add_base_model(0, base_positions or [0.0, 0.0, 0.0])
        #添加建筑物
        self._add_buildings()

        self.static_objects_added = True

    def _add_base_model(self, idx: int, base_position):
        """根据索引为指定基地添加模型，名称为 base_model_{idx}。"""
        from Visual.render_utils import create_guts_base_mesh

        base_mesh = create_guts_base_mesh(base_position)
        material = self.rendering.MaterialRecord()
        material.shader = 'defaultLit'
        name = f"base_model_{idx}"
        self.scene_widget.scene.add_geometry(name, base_mesh, material)
        

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

    def update(self, drones: List[BaseDrone], detections, base_health: float, base_positions):
        if not self.is_open:
            return

        if not self.map_initialized:
            self._init_map_data()
        if not self.static_objects_added:
            self._add_static_objects(base_positions)

        # 清理每帧重建的动态几何体（轨迹、虚线等）
        for name in list(self.dynamic_geometries):
            try:
                self.scene_widget.scene.remove_geometry(name)
            except Exception:
                pass
        self.dynamic_geometries.clear()

        offensive = [d for d in drones if d.drone_type in self.offensive_types and not d.destroyed]
        defensive = [d for d in drones if d.drone_type not in self.offensive_types and not d.destroyed]

        # 记录当前存活无人机的名称，用于清理无人机几何体
        current_drone_names = set()

        # ===== 1. 处理存活无人机的模型移动和动态几何体 =====
        for drone in drones:
            if drone.destroyed:
                # 无人机被摧毁：清除其无人机几何体（如果还在）
                name = f"drone_{drone.name}"
                if name in self.drone_geoms:
                    self.scene_widget.scene.remove_geometry(name)
                    del self.drone_geoms[name]
                continue

            # 存活无人机
            name = f"drone_{drone.name}"
            current_drone_names.add(name)
            color = self._drone_color(drone)

            # 无人机模型（矩阵移动）
            if name not in self.drone_geoms:
                mesh = create_drone_model_mesh(color=color)  # 中心在原点
                material = self.rendering.MaterialRecord()
                material.base_color = color + (1.0,)
                self.scene_widget.scene.add_geometry(name, mesh, material)
                self.drone_geoms[name] = True
            mat = np.eye(4, dtype=np.float64)
            mat[:3, 3] = drone.position[:3]
            self.scene_widget.scene.set_geometry_transform(name, mat)

            # 轨迹线（每帧重建）
            if self.enable_trails and len(drone.trail) > 1:
                trail_name = f"trail_{drone.name}"
                # 先移除旧的几何体（忽略异常）
                try:
                    self.scene_widget.scene.remove_geometry(trail_name)
                except Exception:
                    pass
                trail_color = self._color_for_type(drone.drone_type)
                path = create_path_line(drone.trail, color=trail_color)
                if path is not None:
                    material_trail = self.rendering.MaterialRecord()
                    material_trail.shader = 'unlitLine'
                    material_trail.base_color = trail_color + (1.0,)
                    self._safe_add_dynamic_geometry(trail_name, path, material_trail)

            # 虚线连线（仅防守方）
            if self.enable_dashed_lines and drone in defensive:
                target = getattr(drone, "_assigned_target", None)
                if target is not None and hasattr(target, "position") and target.is_alive():
                    dash = create_dashed_line(drone.position, target.position, color=(1.0, 0.8, 0.0))
                    if dash is not None:
                        material_assign = self.rendering.MaterialRecord()
                        material_assign.shader = 'unlitLine'
                        material_assign.base_color = (1.0, 0.8, 0.0, 1.0)
                        self._safe_add_dynamic_geometry(f"assign_{drone.name}", dash, material_assign)

            # 可选：计算最近敌人距离（用于 HUD）
            enemies = defensive if drone in offensive else offensive
            self._nearest_enemy_distance(drone, enemies)

        # ===== 2. 清理不再存活的无人机几何体 =====
        for geom_name in list(self.drone_geoms.keys()):
            if geom_name not in current_drone_names:
                self.scene_widget.scene.remove_geometry(geom_name)
                del self.drone_geoms[geom_name]

        # ===== 3. 处理持久化的坠毁/爆炸特效 =====
        import time
        now = time.time()

        # 第一步：为刚被摧毁且尚未处理特效的无人机创建特效
        for drone in drones:
            if drone.destroyed:
                if drone.name in self._effect_handled:
                    continue
                effect_key = f"effect_{drone.name}"
                if drone.impact:
                    # 爆炸：球体从小到大
                    explosion_mesh = self.o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
                    explosion_mesh.compute_vertex_normals()
                    explosion_mesh.paint_uniform_color(self._color_for_type(drone.drone_type))
                    self.scene_widget.scene.add_geometry(effect_key, explosion_mesh, self.rendering.MaterialRecord())
                    self.active_effects[effect_key] = {
                        'type': 'explosion',
                        'start_time': now,
                        'duration': drone.death_effect_duration,
                        'start_pos': drone.position.copy(),
                        'mesh': explosion_mesh,
                        'initial_radius': 0.5,
                        'final_radius': 6.0
                    }
                else:
                    # 坠落：简单球体，匀加速下落
                    fall_color = (0.4, 0.4, 0.4)
                    fall_mesh = self.o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
                    fall_mesh.compute_vertex_normals()
                    fall_mesh.paint_uniform_color(fall_color)
                    self.scene_widget.scene.add_geometry(effect_key, fall_mesh, self.rendering.MaterialRecord())
                    self.active_effects[effect_key] = {
                        'type': 'falling',
                        'start_time': now,
                        'last_time': now,           # 添加
                        'start_pos': drone.position.copy(),
                        'mesh': fall_mesh,
                        'ground_height': 0.0,
                        'velocity': 0.0,            # 初速度0
                        'gravity': -9.8
                    }
                self._effect_handled.add(drone.name)

        # 第二步：更新所有已存在的特效
        for effect_key, info in list(self.active_effects.items()):
            elapsed = now - info['start_time']
            if info['type'] == 'explosion':
                if elapsed >= info['duration']:
                    self.scene_widget.scene.remove_geometry(effect_key)
                    del self.active_effects[effect_key]
                    continue
                t = elapsed / info['duration']
                radius = info['initial_radius'] + (info['final_radius'] - info['initial_radius']) * t
                scale = radius / info['initial_radius']
                mat = np.eye(4, dtype=np.float64)
                mat[0,0] = scale
                mat[1,1] = scale
                mat[2,2] = scale
                mat[:3, 3] = info['start_pos']
                self.scene_widget.scene.set_geometry_transform(effect_key, mat)
            elif info['type'] == 'falling':
                # 获取上次更新时间（如果没有，则用 start_time）
                if 'last_time' not in info:
                    info['last_time'] = info['start_time']
                    info['velocity'] = 0.0
                dt = now - info['last_time']
                if dt > 0.05:  # 限制最大时间步长，避免跳跃过大
                    dt = 0.05
                # 更新速度：v = v + g * dt，限制最大速度 -15.0 m/s
                info['velocity'] += info['gravity'] * dt
                if info['velocity'] < -15.0:
                    info['velocity'] = -15.0
                # 更新位置
                new_z = info['start_pos'][2] + info['velocity'] * dt
                info['start_pos'][2] = new_z   # 更新当前高度，用于下一帧
                if new_z <= info['ground_height']:
                    new_z = info['ground_height']
                    self.scene_widget.scene.remove_geometry(effect_key)
                    del self.active_effects[effect_key]
                    continue
                mat = np.eye(4, dtype=np.float64)
                mat[:3, 3] = [info['start_pos'][0], info['start_pos'][1], new_z]
                self.scene_widget.scene.set_geometry_transform(effect_key, mat)
                info['last_time'] = now

        # 第三步：清理已经完全移除的无人机的特效记录
        existing_drone_names = {d.name for d in drones}
        for name in list(self._effect_handled):
            if name not in existing_drone_names:
                self._effect_handled.discard(name)
        for effect_key in list(self.active_effects.keys()):
            drone_name = effect_key[7:]  # 去掉 "effect_"
            if drone_name not in existing_drone_names:
                self.scene_widget.scene.remove_geometry(effect_key)
                del self.active_effects[effect_key]
                self._effect_handled.discard(drone_name)

        # ===== 4. FPS 跟踪和 HUD 更新 =====
        import time as _time
        now_ = _time.perf_counter()
        if not hasattr(self, '_last_fps_tick'):
            self._last_fps_tick = now_
        dt = now_ - self._last_fps_tick
        self._last_fps_tick = now_
        if dt > 0:
            self._fps_accum_time += dt
            self._fps_frame_count += 1
            if self._fps_accum_time >= 0.5:
                self._fps_current = self._fps_frame_count / self._fps_accum_time
                self._fps_frame_count = 0
                self._fps_accum_time = 0.0

        if self._fps_current > 0:
            self._sim_elapsed += 1.0 / self._fps_current
        else:
            self._sim_elapsed += 0.016
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

    @staticmethod
    def _dashed_line_points(start, end, dash_length=8.0, gap_length=6.0):
        start = np.array(start)
        end = np.array(end)
        vec = end - start
        dist = np.linalg.norm(vec)
        if dist < 1e-6:
            return None
        dirv = vec / dist
        segment = dash_length + gap_length
        count = int(dist // segment)
        points = []
        for i in range(count):
            s = start + dirv * (i * segment)
            e = start + dirv * (i * segment + dash_length)
            points.append(s)
            points.append(e)
        tail_start = count * segment
        if tail_start < dist:
            s = start + dirv * tail_start
            e = start + dirv * min(tail_start + dash_length, dist)
            points.append(s)
            points.append(e)
        if not points:
            return None
        return np.array(points)