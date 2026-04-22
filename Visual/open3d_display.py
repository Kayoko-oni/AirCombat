from typing import List

import yaml

from drones.base_drone import BaseDrone
from Visual.render_utils import create_drone_mesh, create_path_line, create_explosion_mesh, create_drone_model_mesh, create_guts_base_mesh


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
        # Add coordinate frame
        coord_frame = self.o3d.geometry.TriangleMesh.create_coordinate_frame(size=20.0)
        coord_frame.compute_vertex_normals()
        self.scene_widget.scene.add_geometry("coord_frame", coord_frame, self.rendering.MaterialRecord())
        self._add_ground_plane()

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

    def _add_base_model(self, base_position):
        """在原点添加基地模型（灰色长方体+绿色球体）"""
        # 使用我们之前定义的 create_guts_base_mesh 函数
        from Visual.render_utils import create_guts_base_mesh
        base_mesh = create_guts_base_mesh(base_position)
        material = self.rendering.MaterialRecord()
        material.shader = 'defaultLit'
        self.scene_widget.scene.add_geometry("base_model", base_mesh, material)



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
        
        # Clear previous geometries except coordinate frame
        self.scene_widget.scene.clear_geometry()
        # Re-add coordinate frame and ground plane
        coord_frame = self.o3d.geometry.TriangleMesh.create_coordinate_frame(size=60.0)
        coord_frame.compute_vertex_normals()
        self.scene_widget.scene.add_geometry("coord_frame", coord_frame, self.rendering.MaterialRecord())
        self._add_ground_plane()  #参考地面
        self._add_base_model(base_position)  #基地模型

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
                else:
                    fall_color = (0.6, 0.1, 0.1) if drone.drone_type in {"AttackDrone", "TankDrone"} else (0.1, 0.1, 0.6)
                    mesh = create_drone_mesh(drone.position, color=fall_color)
                    material = self.rendering.MaterialRecord()
                    material.base_color = fall_color + (1.0,)
                    self.scene_widget.scene.add_geometry(f"falling_{drone.name}", mesh, material)
                    if len(drone.trail) > 1:
                        path = create_path_line(drone.trail, color=fall_color)
                        if path is not None:
                            material_trail = self.rendering.MaterialRecord()
                            material_trail.shader = 'unlitLine'
                            material_trail.base_color = fall_color + (1.0,)
                            self.scene_widget.scene.add_geometry(f"trail_{drone.name}", path, material_trail)
                continue

            color = self._drone_color(drone)
            mesh = create_drone_model_mesh(drone.position, color=color)
            material = self.rendering.MaterialRecord()
            material.base_color = color + (1.0,)
            # TODO: 增加视觉效果
            self.scene_widget.scene.add_geometry(f"drone_{drone.name}", mesh, material)

            if len(drone.trail) > 1:
                trail_color = (1.0, 0.2, 0.2) if drone.drone_type in {"AttackDrone", "TankDrone"} else (0.2, 0.4, 1.0)
                path = create_path_line(drone.trail, color=trail_color)
                if path is not None:
                    material_trail = self.rendering.MaterialRecord()
                    material_trail.shader = 'unlitLine'
                    material_trail.base_color = trail_color + (1.0,)
                    self.scene_widget.scene.add_geometry(f"trail_{drone.name}", path, material_trail)

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
