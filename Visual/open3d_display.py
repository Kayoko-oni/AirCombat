from typing import List
import numpy as np

import yaml

from utils.map_loader import generate_buildings
from drones.base_drone import BaseDrone
from Visual.render_utils import create_drone_mesh, create_path_line, create_explosion_mesh, create_drone_model_mesh, create_guts_base_mesh, create_dashed_line


class Open3DDisplay:
    def __init__(self, map_size=(1000, 1000)):
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
        
        # Status label as HUD overlay
        self.status_label = self.gui.Label("")
        self.status_label.text_color = self.gui.Color(1.0, 1.0, 1.0, 1.0)
        self.status_label.background_color = self.gui.Color(0.0, 0.0, 0.0, 0.5)  # Semi-transparent background
        self.window.add_child(self.status_label)
        # Position the label at top-left
        self.status_label.frame = self.gui.Rect(10, 10, 350, 100)
        
        # Set scene widget to fill the window
        self.scene_widget.frame = self.gui.Rect(0, 0, self.window.content_rect.width, self.window.content_rect.height)
        
        self.is_open = True
        self.paused = False

        self.static_objects_added = False      # 静态物体是否已添加
        self.dynamic_geometries = set()        # 存储动态几何体的名称
        self.map_initialized = False           # 地图是否已初始化（用于首次更新时添加建筑）

        self.map_data_initialized = False   # 地图数据是否已生成
        self.buildings = None               # 建筑列表
        self.map_grid = None                # 障碍物网格对象
        
        # Set up camera
        import numpy as np
        bounds = self.o3d.geometry.AxisAlignedBoundingBox(np.array([-1000, -1000, -100], dtype=np.float64), np.array([1000, 1000, 100], dtype=np.float64))
        center = np.array([0., 0., 0.], dtype=np.float32)
        self.scene_widget.setup_camera(45.0, bounds, center)
        self.scene_widget.set_view_controls(self.gui.SceneWidget.Controls.ROTATE_CAMERA)

    def _mouse_callback(self, event):
        if event.type == self.gui.MouseEvent.Type.BUTTON_DOWN and event.is_button_down(self.gui.MouseButton.RIGHT):
            import numpy as np
            bounds = self.o3d.geometry.AxisAlignedBoundingBox(np.array([-1000, -1000, -100], dtype=np.float64), np.array([1000, 1000, 100], dtype=np.float64))
            center = np.array([0., 0., 0.], dtype=np.float32)
            self.scene_widget.setup_camera(45.0, bounds, center)
            return self.gui.SceneWidget.EventCallbackResult.CONSUMED
        return self.gui.SceneWidget.EventCallbackResult.IGNORED

    def open_window(self):
        self.scene_widget.set_on_mouse(self._mouse_callback)
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
        

    def _init_map_data(self):
        """生成建筑列表和障碍物网格（只执行一次）"""
        if self.map_data_initialized:
            return
        from utils.map_loader import generate_buildings
        from utils.map_grid import MapGrid
        self.buildings = generate_buildings()
        self.map_grid = MapGrid(self.buildings, cell_size=5.0)
        self.map_data_initialized = True
        print(f"Map data initialized: {len(self.buildings)} buildings, grid size {self.map_grid.width}x{self.map_grid.height}")
        # ===================================测试代码，验证网格是否被正确标记，路径规划确认读取无误后可以删除======================================================
        if self.map_grid is not None and self.buildings:
            # 1. 取第一个建筑的中心点，应被标记为障碍物
            cx, cy, _ = self.buildings[0][0]
            occupied = self.map_grid.is_occupied(cx, cy)
            print(f"验证1: 第一个建筑中心 ({cx:.1f},{cy:.1f}) 是否被障碍物网格标记？ {occupied} (预期 True)")

            # 2. 取一个明显空闲的点，例如 (400, 400)
            free_x, free_y = 400, 400
            occupied_free = self.map_grid.is_occupied(free_x, free_y)
            print(f"验证2: 空闲点 ({free_x},{free_y}) 是否被障碍物网格标记？ {occupied_free} (预期 False)")

            # 3. 打印网格中障碍物比例
            total_cells = self.map_grid.width * self.map_grid.height
            occupied_cells = np.sum(self.map_grid.obstacle_grid)
            print(f"网格尺寸: {self.map_grid.width}x{self.map_grid.height}, 障碍物比例: {occupied_cells}/{total_cells} ({100*occupied_cells/total_cells:.1f}%)")
        #======================================================================================================================================================

    def _add_buildings(self):
        """将建筑列表转换为单个网格并添加到场景"""
        if self.buildings is None:
            return
        combined = self.o3d.geometry.TriangleMesh()
        for center, size in self.buildings:
            cx, cy, cz = center
            sx, sy, sz = size
            box = self.o3d.geometry.TriangleMesh.create_box(width=sx, height=sy, depth=sz)
            box.translate([cx - sx/2, cy - sy/2, cz - sz/2])
            box.paint_uniform_color((0.5, 0.5, 0.5))
            combined += box
        combined.compute_vertex_normals()
        material = self.rendering.MaterialRecord()
        material.shader = 'defaultLit'
        material.base_color = (0.5, 0.5, 0.5, 0.8)
        self.scene_widget.scene.add_geometry("city_buildings", combined, material)
        print(f"Buildings displayed: {len(self.buildings)}")

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.scene_widget.frame = self.gui.Rect(0, 0, r.width, r.height)
        return self.gui.Widget.EventCallbackResult.HANDLED

    def _drone_color(self, drone: BaseDrone):
        if drone.drone_type in {"AttackDrone", "TankDrone"}:
            return (1.0, 0.2, 0.2)
        return (0.2, 0.4, 1.0)

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

    def _status_text(self, drones: List[BaseDrone], base_health: float) -> str:
        active = [d for d in drones if not d.destroyed]
        red = [d for d in active if d.drone_type in {"AttackDrone", "TankDrone"}]
        blue = [d for d in active if d.drone_type not in {"AttackDrone", "TankDrone"}]
        type_counts = {
            "AttackDrone": sum(1 for d in active if d.drone_type == "AttackDrone"),
            "TankDrone": sum(1 for d in active if d.drone_type == "TankDrone"),
            "ScoutDrone": sum(1 for d in active if d.drone_type == "ScoutDrone"),
            "InterceptorDrone": sum(1 for d in active if d.drone_type == "InterceptorDrone"),
        }
        status_lines = [
            f"Red Team: {len(red)}  Blue Team: {len(blue)}",
            f"Attack: {type_counts['AttackDrone']}  Tank: {type_counts['TankDrone']}",
            f"Scout: {type_counts['ScoutDrone']}  Interceptor: {type_counts['InterceptorDrone']}",
            f"Base Health: {base_health:.1f}",
            "Right-click reset view",
        ]
        return "\n".join(status_lines)

    def update(self, drones: List[BaseDrone], detections, base_health: float, base_position):
        if not self.is_open:
            return
        
        if not self.map_initialized:
            self._init_map_data()   # 只执行一次，生成 self.buildings 和 self.map_grid

        if not self.static_objects_added: #只在首次更新时添加静态物体（坐标轴、地面、基地模型）
            self._add_static_objects(base_position)

        for name in self.dynamic_geometries:
            self.scene_widget.scene.remove_geometry(name)
        self.dynamic_geometries.clear()

        offensive = [d for d in drones if d.drone_type in {"AttackDrone", "TankDrone"} and not d.destroyed]
        defensive = [d for d in drones if d.drone_type not in {"AttackDrone", "TankDrone"} and not d.destroyed]

        for drone in drones:
            if drone.destroyed:
                if drone.impact:
                    explosion_color = (0.8, 0.2, 0.0) if drone.drone_type in {"AttackDrone", "TankDrone"} else (0.0, 0.2, 0.8)
                    debris = create_explosion_mesh(drone.position, drone.death_timer, drone.death_effect_duration, color=explosion_color)
                    material = self.rendering.MaterialRecord()
                    material.base_color = explosion_color + (1.0,)
                    self.scene_widget.scene.add_geometry(f"explosion_{drone.name}", debris, material)
                    self.dynamic_geometries.add(f"explosion_{drone.name}")
                else:
                    fall_color = (0.6, 0.1, 0.1) if drone.drone_type in {"AttackDrone", "TankDrone"} else (0.1, 0.1, 0.6)
                    mesh = create_drone_mesh(drone.position, color=fall_color)
                    material = self.rendering.MaterialRecord()
                    material.base_color = fall_color + (1.0,)
                    self.scene_widget.scene.add_geometry(f"falling_{drone.name}", mesh, material)
                    self.dynamic_geometries.add(f"falling_{drone.name}")
                    if len(drone.trail) > 1:
                        path = create_path_line(drone.trail, color=fall_color)
                        if path is not None:
                            material_trail = self.rendering.MaterialRecord()
                            material_trail.shader = 'unlitLine'
                            material_trail.base_color = fall_color + (1.0,)
                            self.scene_widget.scene.add_geometry(f"trail_{drone.name}", path, material_trail)
                            self.dynamic_geometries.add(f"trail_{drone.name}")
                continue

            color = self._drone_color(drone)
            mesh = create_drone_model_mesh(drone.position, color=color)
            material = self.rendering.MaterialRecord()
            material.base_color = color + (1.0,)
            # TODO: 增加视觉效果
            self.scene_widget.scene.add_geometry(f"drone_{drone.name}", mesh, material)
            self.dynamic_geometries.add(f"drone_{drone.name}")

            if len(drone.trail) > 1:
                trail_color = (1.0, 0.2, 0.2) if drone.drone_type in {"AttackDrone", "TankDrone"} else (0.2, 0.4, 1.0)
                path = create_path_line(drone.trail, color=trail_color)
                if path is not None:
                    material_trail = self.rendering.MaterialRecord()
                    material_trail.shader = 'unlitLine'
                    material_trail.base_color = trail_color + (1.0,)
                    self.scene_widget.scene.add_geometry(f"trail_{drone.name}", path, material_trail)
                    self.dynamic_geometries.add(f"trail_{drone.name}")
            # 如果存在避障路径缓存（来自 PathTracker 或 CBS），可视化显示
            # 优先显示防守方的 CBS 路径 (_cbs_path)（红色），其次显示个人避障路径 (_avoid_path)（绿色）
            try:
                if hasattr(drone, "_cbs_path") and drone._cbs_path:
                    # 使用不那么刺眼的紫色来区分于进攻方轨迹（红色）
                    p = create_path_line(drone._cbs_path, color=(0.6, 0.2, 0.8))
                    if p is not None:
                        material_p = self.rendering.MaterialRecord()
                        material_p.shader = 'unlitLine'
                        material_p.base_color = (0.6, 0.2, 0.8, 1.0)
                        self.scene_widget.scene.add_geometry(f"path_{drone.name}", p, material_p)
                        self.dynamic_geometries.add(f"path_{drone.name}")
                elif hasattr(drone, "_avoid_path") and drone._avoid_path:
                    p = create_path_line(drone._avoid_path, color=(0.2, 0.8, 0.2))
                    if p is not None:
                        material_p = self.rendering.MaterialRecord()
                        material_p.shader = 'unlitLine'
                        material_p.base_color = (0.2, 0.8, 0.2, 1.0)
                        self.scene_widget.scene.add_geometry(f"path_{drone.name}", p, material_p)
                        self.dynamic_geometries.add(f"path_{drone.name}")
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
                        self.scene_widget.scene.add_geometry(f"assign_{drone.name}", dash, material_assign)
                        self.dynamic_geometries.add(f"assign_{drone.name}")
            except Exception:
                pass
            enemies = defensive if drone in offensive else offensive
            nearest_dist = self._nearest_enemy_distance(drone, enemies)
            # Removed 3D label for each drone to reduce clutter
            # label_text = self._label_text(drone, nearest_dist)
            # self.scene_widget.add_3d_label(drone.position, label_text)

        # Update status label as HUD
        status_text = self._status_text(drones, base_health)
        self.status_label.text = status_text

        # Force redraw
        self.scene_widget.force_redraw()
        self.window.post_redraw()

    def close_window(self):
        if self.is_open:
            self.window.close()
            self.is_open = False
