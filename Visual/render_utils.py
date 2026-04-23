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

    # ------------------ 十字悬臂（X轴和Y轴方向的扁立方体） ------------------
    arm_length = 8.0
    arm_width = 0.4
    arm_height = 0.15
    # X方向臂
    arm_x = o3d.geometry.TriangleMesh.create_box(width=arm_length, height=arm_width, depth=arm_height)
    arm_x.translate([-arm_length/2, -arm_width/2, -arm_height/2])
    arm_x.paint_uniform_color((0.6, 0.6, 0.6))
    # Y方向臂
    arm_y = o3d.geometry.TriangleMesh.create_box(width=arm_width, height=arm_length, depth=arm_height)
    arm_y.translate([-arm_width/2, -arm_length/2, -arm_height/2])
    arm_y.paint_uniform_color((0.6, 0.6, 0.6))

    # ------------------ 机身（扁立方体，放在十字臂中央上方） ------------------
    body = o3d.geometry.TriangleMesh.create_box(width=2.5, height=2.5, depth=1.5)
    body.translate([-1.25, -1.25, arm_height])  # 放在臂的上面
    body.paint_uniform_color((0.4, 0.4, 0.4))

    # ------------------ 旋翼：圆盘（压扁球体）位于四个臂的末端 ------------------
    endpoints = [
        ( arm_length/2,  0, arm_height),
        (-arm_length/2,  0, arm_height),
        ( 0,  arm_length/2, arm_height),
        ( 0, -arm_length/2, arm_height)
    ]
    rotors = []
    motors = []
    rotor_diameter = 1.6
    rotor_thick = 0.1
    for ex, ey, ez in endpoints:
        rotor = o3d.geometry.TriangleMesh.create_sphere(radius=rotor_diameter/2)
        vertices = np.asarray(rotor.vertices)
        vertices[:, 2] *= (rotor_thick / rotor_diameter)  # 压扁成圆盘
        rotor.vertices = o3d.utility.Vector3dVector(vertices)
        rotor.compute_vertex_normals()
        rotor.translate([ex, ey, ez])
        rotor.paint_uniform_color(color)
        rotors.append(rotor)

        motor = o3d.geometry.TriangleMesh.create_sphere(radius=0.25)
        motor.translate([ex, ey, ez])
        motor.paint_uniform_color((0.2, 0.2, 0.2))
        motors.append(motor)

    # ------------------ 合并 ------------------
    combined = arm_x + arm_y + body
    for rotor in rotors:
        combined += rotor
    for motor in motors:
        combined += motor

    # ------------------ 整体上下翻转（绕 X 轴旋转 180 度） ------------------
    R = combined.get_rotation_matrix_from_xyz((np.pi, 0, 0))  # 绕 X 轴转 180 度
    combined.rotate(R, center=(0, 0, 0))

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

def create_guts_base_mesh(position): #添加基地模型
    import open3d as o3d
    import numpy as np

    # 基座：长方体 (长40, 高5, 宽40) 注意：height 沿 Y，depth 沿 Z
    base = o3d.geometry.TriangleMesh.create_box(width=40.0, height=5.0, depth=40.0)
    base.paint_uniform_color((0.5, 0.5, 0.5))
    # 将基座中心移至原点，底部位于 y = -2.5
    base.translate([-20.0, -2.5, -20.0])

    # 穹顶：球体半径 2.5，位于基座顶面中央 (y = 2.5 处)
    dome = o3d.geometry.TriangleMesh.create_sphere(radius=12.5)
    dome.paint_uniform_color((0.2, 0.6, 0.2))
    dome.translate([0, 2.5, 0])

    # 合并
    combined = base + dome

    # 旋转：绕 X 轴转 +90 度，使原 Y 轴转向 Z 轴（适配 Z-up 世界）
    R = combined.get_rotation_matrix_from_xyz((np.pi/2, 0, 0))
    combined.rotate(R, center=(0, 0, 0))


    combined.compute_vertex_normals()
    combined.translate(position)
    return combined



