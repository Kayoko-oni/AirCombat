import open3d as o3d
import numpy as np

def voxelize_obj(obj_path, target_extent=1000.0, voxel_size=20.0,
                 map_bounds=(-500, -500, 500, 500), z_up_offset=120.0):
    mesh = o3d.io.read_triangle_mesh(obj_path)
    mesh.compute_vertex_normals()
    # 旋转 Y->Z
    R = mesh.get_rotation_matrix_from_xyz((np.pi/2, 0, 0))
    mesh.rotate(R, center=mesh.get_center())
    mesh.compute_vertex_normals()
    # 缩放最长边到 target_extent
    bbox = mesh.get_axis_aligned_bounding_box()
    size = bbox.max_bound - bbox.min_bound
    scale = target_extent / max(size[0], size[1], size[2])
    mesh.scale(scale, center=bbox.get_center())
    mesh.compute_vertex_normals()
    # 平移到原点，再向上偏移
    mesh.translate(-mesh.get_center())
    mesh.translate([0, 0, z_up_offset])

    # 体素化
    voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size)
    voxels = voxel_grid.get_voxels()

    # 生成二维障碍网格
    min_x, max_x = map_bounds[0], map_bounds[2]
    min_y, max_y = map_bounds[1], map_bounds[3]
    cell_size = voxel_size
    w = int((max_x - min_x) / cell_size) + 1
    h = int((max_y - min_y) / cell_size) + 1
    obstacle_grid = np.zeros((w, h), dtype=bool)
    obstacle_height = np.zeros((w, h), dtype=float)

    origin = voxel_grid.origin
    vs = voxel_size
    for v in voxels:
        cx, cy, cz = origin + v.grid_index * vs + vs/2
        gx = int((cx - min_x) / cell_size)
        gy = int((cy - min_y) / cell_size)
        if 0 <= gx < w and 0 <= gy < h:
            obstacle_grid[gx, gy] = True
            if cz > obstacle_height[gx, gy]:
                obstacle_height[gx, gy] = cz

    # 低高度过滤：高度低于20米的格子视为空地
    low_threshold = 20.0
    for gx in range(w):
        for gy in range(h):
            if obstacle_height[gx, gy] < low_threshold:
                obstacle_grid[gx, gy] = False
                obstacle_height[gx, gy] = 0.0

    return mesh, obstacle_grid, obstacle_height, cell_size, (min_x, min_y, max_x, max_y)