import open3d as o3d
import numpy as np


def create_drone_mesh(position, color=(1.0, 0.0, 0.0)):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color(color)
    sphere.translate(position)
    return sphere

def create_drone_model_mesh(position, color=(1.0, 0.0, 0.0)):
    import open3d as o3d
    import numpy as np

    # ------------------ 基座 ------------------
    base_size = 2.5
    base_thick = 1.5
    base = o3d.geometry.TriangleMesh.create_box(width=base_size, height=base_size, depth=base_thick)
    base.translate([-base_size/2, -base_size/2, -base_thick/2])
    base.paint_uniform_color((0.5, 0.5, 0.5))

    corner_z = base_thick / 2
    corners = [
        ( base_size/2,  base_size/2, corner_z),
        ( base_size/2, -base_size/2, corner_z),
        (-base_size/2,  base_size/2, corner_z),
        (-base_size/2, -base_size/2, corner_z),
    ]

    # ------------------ 旋翼中心 ------------------
    arm_extension = 1.8
    rotor_z = corner_z + 0.2
    rotor_centers = [
        ( corners[0][0] + arm_extension,  corners[0][1] + arm_extension, rotor_z),
        ( corners[1][0] + arm_extension,  corners[1][1] - arm_extension, rotor_z),
        ( corners[2][0] - arm_extension,  corners[2][1] + arm_extension, rotor_z),
        ( corners[3][0] - arm_extension,  corners[3][1] - arm_extension, rotor_z),
    ]

    # ------------------ 旋翼（压扁球体，水平圆盘） ------------------
    rotor_diameter = 1.6
    rotor_thick = 0.1
    rotors = []
    motors = []
    for pos in rotor_centers:
        rotor = o3d.geometry.TriangleMesh.create_sphere(radius=rotor_diameter/2)
        vertices = np.asarray(rotor.vertices)
        vertices[:, 2] *= (rotor_thick / rotor_diameter)
        rotor.vertices = o3d.utility.Vector3dVector(vertices)
        rotor.compute_vertex_normals()
        rotor.translate(pos)
        rotor.paint_uniform_color(color)
        rotors.append(rotor)

        motor = o3d.geometry.TriangleMesh.create_sphere(radius=0.25)
        motor.translate(pos)
        motor.paint_uniform_color((0.2, 0.2, 0.2))
        motors.append(motor)

    # ------------------ 合并 ------------------
    combined = base
    for rotor in rotors:
        combined += rotor
    for motor in motors:
        combined += motor

    combined.compute_vertex_normals()
    combined.translate(position)
    return combined

def create_path_line(points, color=(1.0, 0.0, 0.0)):
    if len(points) < 2:
        return None
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(points) - 1)]),
    )
    total = len(points) - 1
    colors = []
    for index in range(total):
        fade = 0.3 + 0.7 * ((index + 1) / total)
        colors.append([min(1.0, c * fade) for c in color])
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def create_explosion_mesh(position, timer, duration, color=(1.0, 0.5, 0.0)):
    radius = 3.0 + 6.0 * min(1.0, timer / max(duration, 1e-3))
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color(color)
    sphere.translate(position)
    return sphere
