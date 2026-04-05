import open3d as o3d


def create_drone_mesh(position, color=(1.0, 0.0, 0.0)):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color(color)
    sphere.translate(position)
    return sphere


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
