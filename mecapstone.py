import numpy as np
import open3d as o3d
import copy
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt

def internal_point_3d(p1, p2, m, n):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x = (m * x2 + n * x1) / (m + n)
    y = (m * y2 + n * y1) / (m + n)
    z = (m * z2 + n * z1) / (m + n)
    return x, y, z

def getNineCenter(leftDown, rightDown, leftUp, rightUp):
    E = internal_point_3d(leftUp, leftDown, 1, 4)
    center1 = internal_point_3d(E, rightUp, 1, 5)
    H = internal_point_3d(leftUp, rightUp, 4, 1)
    center3 = internal_point_3d(H, rightDown, 1, 5)
    F = internal_point_3d(leftDown, rightDown, 1, 4)
    center7 = internal_point_3d(leftUp, F, 5, 1)
    G = internal_point_3d(rightUp, rightDown, 4, 1)
    center9 = internal_point_3d(G, leftDown, 1, 5)
    center2 = internal_point_3d(center1, center3, 1, 1)
    center4 = internal_point_3d(center1, center7, 1, 1)
    center6 = internal_point_3d(center3, center9, 1, 1)
    center8 = internal_point_3d(center7, center9, 1, 1)
    center5 = internal_point_3d(center4, center6, 1, 1)
    return [center1, center2, center3, center4, center5, center6, center7, center8, center9]

def get_nearest_neighbors_avg_color(center, point_cloud, n_neighbors=4):
    points = np.asarray(point_cloud.points)
    colors = np.asarray(point_cloud.colors)
    center_point = np.array(center)
    nbrs = NearestNeighbors(n_neighbors=n_neighbors).fit(points)
    distances, indices = nbrs.kneighbors([center_point])
    nearest_neighbors_colors = colors[indices[0]]
    avg_color = np.mean(nearest_neighbors_colors, axis=0)
    return avg_color

def get_color_name_hsv(hsv):
    color_names = {
        "DRed":  (358, 76),
        "Dred": (3, 88),
        "BYellow": (67, 99),
        "LGreen": (104, 90),
        "RBlue": (190, 85),
        "UOrange": (23, 81),
        "FWhite": (41, 19),
    }
    closest_color = None
    min_distance = float('inf')
    for color, color_hsv in color_names.items():
        distance = sum((a - b) ** 2 for a, b in zip(hsv[:2], color_hsv))
        if distance < min_distance:
            min_distance = distance
            closest_color = color
    return closest_color

def find_farthest_point2(point_cloud, input_point):
    arr = np.asarray(point_cloud)
    distances = np.linalg.norm(arr - input_point, axis=1)
    farthest_point_index = np.argmax(distances)
    farthest_point = arr[farthest_point_index]
    return farthest_point

def rgb_to_hsv(rgb):
    r, g, b = [x / 255.0 for x in rgb]
    max_val = max(r, g, b)
    min_val = min(r, g, b)
    v = max_val
    s = 0 if max_val == 0 else (max_val - min_val) / max_val
    if max_val == min_val:
        h = 0
    else:
        delta = max_val - min_val
        if max_val == r:
            h = ((g - b) / delta) % 6
        elif max_val == g:
            h = (b - r) / delta + 2
        else:
            h = (r - g) / delta + 4
        h *= 60
    return round(h), round(s * 100), round(v * 100)

def save_point_cloud_to_ply(points, filename='cube_with_interior.ply'):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    #o3d.io.write_point_cloud(filename, point_cloud)
    return point_cloud

def create_cube_with_interior(cube_size, num_interior_cubes):
    exterior_cube_points = np.array([
        [0, 0, 0],
        [cube_size, 0, 0],
        [cube_size, cube_size, 0],
        [0, cube_size, 0],
        [0, 0, cube_size],
        [cube_size, 0, cube_size],
        [cube_size, cube_size, cube_size],
        [0, cube_size, cube_size],
    ])
    interior_cube_points = np.random.rand(num_interior_cubes, 3) * cube_size
    all_points = np.vstack([exterior_cube_points, interior_cube_points])
    return all_points

def plot_point_cloud(pcd, filename, title=''):
    pts = np.asarray(pcd.points)
    if pts.shape[0] == 0:
        print(f"{filename}: No points to plot.")
        return
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    colors = None
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        if colors.max() <= 1.0:
            colors = (colors * 255).astype(int)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=colors/255.0, s=1)
    else:
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='gray', s=1)
    ax.set_title(title)
    plt.axis('off')
    plt.savefig(filename, bbox_inches='tight', pad_inches=0)
    plt.close()

# -------------------- Main Processing --------------------

pcd_Flo = o3d.io.read_point_cloud("./input_bottom.ply")
pcd_down_Flo = pcd_Flo.voxel_down_sample(voxel_size=0.005)

plane_model_Flo, inliers_Flo = pcd_down_Flo.segment_plane(
    distance_threshold=0.02, ransac_n=400, num_iterations=5000)
[a, b, c, d] = plane_model_Flo
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud_Flo = pcd_down_Flo.select_by_index(inliers_Flo)
inlier_cloud_Flo.paint_uniform_color([1.0, 0, 0])
outlier_cloud_Flo = pcd_down_Flo.select_by_index(inliers_Flo, invert=True)
# o3d.visualization.draw_geometries([inlier_cloud_Flo, outlier_cloud_Flo])
# o3d.visualization.draw_geometries([outlier_cloud_Flo])

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels_Flo = np.array(
        outlier_cloud_Flo.cluster_dbscan(eps=0.07, min_points=1000, print_progress=True))

pcd_cube_Flo = o3d.geometry.PointCloud()
for i in range(len(outlier_cloud_Flo.points)):
    if labels_Flo[i] == 0:
        pcd_cube_Flo.points.append(outlier_cloud_Flo.points[i])
        pcd_cube_Flo.colors.append(outlier_cloud_Flo.colors[i])

# o3d.visualization.draw_geometries([pcd_cube_Flo])
#o3d.io.write_point_cloud('gcloud3.ply', pcd_cube_Flo)

pcd_Flo2 = o3d.io.read_point_cloud("./input_top.ply")
pcd_down_Flo2 = pcd_Flo2.voxel_down_sample(voxel_size=0.005)

plane_model_Flo2, inliers_Flo2 = pcd_down_Flo2.segment_plane(
    distance_threshold=0.007, ransac_n=400, num_iterations=5000)
[a2, b2, c2, d2] = plane_model_Flo2
print(f"Plane equation: {a2:.2f}x + {b2:.2f}y + {c2:.2f}z + {d2:.2f} = 0")

inlier_cloud_Flo2 = pcd_down_Flo2.select_by_index(inliers_Flo2)
inlier_cloud_Flo2.paint_uniform_color([1.0, 0, 0])
outlier_cloud_Flo2 = pcd_down_Flo2.select_by_index(inliers_Flo2, invert=True)
# o3d.visualization.draw_geometries([inlier_cloud_Flo2, outlier_cloud_Flo2])
# o3d.visualization.draw_geometries([outlier_cloud_Flo2])

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels_Flo2 = np.array(
        outlier_cloud_Flo2.cluster_dbscan(eps=0.07, min_points=1000, print_progress=True))

pcd_cube_Flo2 = o3d.geometry.PointCloud()
for i in range(len(outlier_cloud_Flo2.points)):
    if labels_Flo2[i] == 0:
        pcd_cube_Flo2.points.append(outlier_cloud_Flo2.points[i])
        pcd_cube_Flo2.colors.append(outlier_cloud_Flo2.colors[i])
# o3d.visualization.draw_geometries([pcd_cube_Flo2])

# 오리엔티드 바운딩 박스 생성 및 변환
oriented_bounding_box = pcd_cube_Flo.get_oriented_bounding_box()
center = np.array(oriented_bounding_box.center)
extent = np.array(oriented_bounding_box.extent)
R = np.array(oriented_bounding_box.R)
# new_oriented_bounding_box = o3d.geometry.OrientedBoundingBox(center, R, extent)

oriented_bounding_box2 = pcd_cube_Flo2.get_oriented_bounding_box()
center2 = np.array(oriented_bounding_box2.center)
extent2 = np.array(oriented_bounding_box2.extent)
R2 = np.array(oriented_bounding_box2.R)
# new_oriented_bounding_box2 = o3d.geometry.OrientedBoundingBox(center2, R2, extent2)

# mesh_frame6 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4, origin=[0, 0, 0])

inverse = [
    [1, 0, 0, -center[0]],
    [0, 1, 0, -center[1]],
    [0, 0, 1, -center[2]],
    [0, 0, 0, 1]
]
pcd_Ori = copy.deepcopy(pcd_cube_Flo).transform(inverse)
pcd_Ori_Bound = pcd_Ori.get_oriented_bounding_box()
center_Ori = np.array(pcd_Ori_Bound.center)
extent_Ori = np.array(pcd_Ori_Bound.extent)
R_Ori = np.array(pcd_Ori_Bound.R)

inverse2 = [
    [1, 0, 0, -center2[0]],
    [0, 1, 0, -center2[1]],
    [0, 0, 1, -center2[2]],
    [0, 0, 0, 1]
]
pcd_Ori2 = copy.deepcopy(pcd_cube_Flo2).transform(inverse2)
pcd_Ori_Bound2 = pcd_Ori2.get_oriented_bounding_box()
center_Ori2 = np.array(pcd_Ori_Bound2.center)
extent_Ori2 = np.array(pcd_Ori_Bound2.extent)
R_Ori2 = np.array(pcd_Ori_Bound2.R)

# 절단 평면 정의 및 포인트 추출
plane_normal = np.array([a, b, c])
plane_origin = np.array([0.025, 0.002, -0.035])
distances = np.dot(np.asarray(pcd_Ori.points) - plane_origin, plane_normal)
threshold = 0.0035
above_plane = pcd_Ori.select_by_index(np.where(distances > threshold)[0])
# o3d.visualization.draw_geometries([above_plane])
o3d.io.write_point_cloud('top_cube.ply', above_plane)

plane_normal2 = np.array([a2, b2, c2])
plane_origin2 = np.array([0.007, 0.005, -0.037])
distances2 = np.dot(np.asarray(pcd_Ori2.points) - plane_origin2, plane_normal2)
above_plane2 = pcd_Ori2.select_by_index(np.where(distances2 > threshold)[0])
# o3d.visualization.draw_geometries([above_plane2])
o3d.io.write_point_cloud('bottom_cube.ply', above_plane2)

above_plane = above_plane.voxel_down_sample(voxel_size=0.0002)
above_plane2 = above_plane2.voxel_down_sample(voxel_size=0.0002)

colors = np.asarray(above_plane.colors)
colors2 = np.asarray(above_plane2.colors)

red_lower = np.array([0, 0.2, 0])
red_upper = np.array([1.0, 0.4, 0.8])

red_indices = np.logical_and(
    np.all(colors >= red_lower, axis=1),
    np.all(colors <= red_upper, axis=1)
)
red_indices2 = np.logical_and(
    np.all(colors2 >= red_lower, axis=1),
    np.all(colors2 <= red_upper, axis=1)
)

red_point_indices = np.where(red_indices)[0]
red_point_indices2 = np.where(red_indices2)[0]

source = above_plane.select_by_index(red_point_indices)
target = above_plane2.select_by_index(red_point_indices2)

# o3d.visualization.draw_geometries([source])
# o3d.visualization.draw_geometries([target])

sourceraw = pcd_Ori.voxel_down_sample(voxel_size=0.0002)
targetraw = pcd_Ori2.voxel_down_sample(voxel_size=0.0002)

radius_normal = 0.06 * 2
radius_feature = 0.06 * 3
source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
          source,
          o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=50))

target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
          target,
          o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=50))

distance_threshold = 0.0085

result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
          source, target, source_fpfh, target_fpfh, True,
          distance_threshold,
          o3d.pipelines.registration.TransformationEstimationPointToPoint(True),
          3, [
              o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                  0.3),
              o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                  distance_threshold)
          ], o3d.pipelines.registration.RANSACConvergenceCriteria(10000000, 0.9999))

T = result.transformation
source2 = copy.deepcopy(sourceraw).transform(T)
target2 = copy.deepcopy(targetraw)
P = source2 + target2
source_color = copy.deepcopy(source).transform(T)
result = o3d.pipelines.registration.registration_icp(
    source2, target2, 0.01, np.identity(4),
    o3d.pipelines.registration.TransformationEstimationPointToPoint(True),
    o3d.pipelines.registration.ICPConvergenceCriteria(1e-6, 1e-6))

T2 = result.transformation
source_color2 = copy.deepcopy(source_color).transform(T2)
source23 = copy.deepcopy(source2).transform(T2)
target23 = copy.deepcopy(target2)
Reconst_cube = source23 + target23
P4 = source_color + target
# o3d.visualization.draw_geometries([P])
# o3d.visualization.draw_geometries([P4])
# o3d.visualization.draw_geometries([Reconst_cube])
o3d.io.write_point_cloud('Reconst_cube.ply', Reconst_cube)

Jab_cube = save_point_cloud_to_ply(create_cube_with_interior(0.08072673, 10000))
Jab_pcd2 = Jab_cube.voxel_down_sample(voxel_size=0.01)
# o3d.visualization.draw_geometries([Jab_pcd2])
Jab_T = np.eye(4)
Jab_source = Jab_pcd2
Jab_target = Reconst_cube
Jab_threshold = 0.01
Jab_trans_init = np.asarray([[1.0, 0.0, 0.0, -0.04145812],
                         [0.0, 1.0, 0.0, -0.04212163],
                         [0.0, 0.0, 1.0, -0.0395336], [0.0, 0.0, 0.0, 1.0]])

Jab_reg_p2p_1 = o3d.pipelines.registration.registration_icp(
    Jab_source, Jab_target, Jab_threshold, Jab_trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
Jab_source2 = copy.deepcopy(Jab_source).transform(Jab_reg_p2p_1.transformation)
Jab_target2 = copy.deepcopy(Jab_target)
Jab_trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
Jab_reg_p2p_2 = o3d.pipelines.registration.registration_icp(
    Jab_source2, Jab_target2, Jab_threshold, Jab_trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
Jab_source3 = copy.deepcopy(Jab_source2).transform(Jab_reg_p2p_2.transformation)
Jab_target3 = copy.deepcopy(Jab_target2)

Jab_reg_p2p_3 = o3d.pipelines.registration.registration_icp(
    Jab_source3, Jab_target3, Jab_threshold, Jab_trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
Jab_source4 = copy.deepcopy(Jab_source3).transform(Jab_reg_p2p_3.transformation)
Jab_target4 = copy.deepcopy(Jab_target3)

Jab_reg_p2p_4 = o3d.pipelines.registration.registration_icp(
    Jab_source4, Jab_target4, Jab_threshold, Jab_trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
Jab_source5 = copy.deepcopy(Jab_source4).transform(Jab_reg_p2p_4.transformation)
Jab_target5 = copy.deepcopy(Jab_target4)

Jab_reg_p2p_5 = o3d.pipelines.registration.registration_icp(
    Jab_source5, Jab_target5, Jab_threshold, Jab_trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
Jab_source6 = copy.deepcopy(Jab_source5).transform(Jab_reg_p2p_4.transformation)
Jab_target6 = copy.deepcopy(Jab_target5)

Jab_source6.paint_uniform_color([1, 0.706, 0])
Jab_target6.paint_uniform_color([0, 0.651, 0.929])
# o3d.visualization.draw_geometries([Jab_source6, Jab_target6])

myInitCorner = [[0, 0, 0, 1], [0.07866566, 0.001697562, 0.001612913, 1], [0.002390593, 0.07747418, 0.0006809889, 1], [0.07794371, 0.07907291, 0.002228768, 1], [0.002032894, 0.001728895, 0.07850936, 1], [0.07698265, 0.002238543, 0.07739687, 1], [0.002006141, 0.07796794, 0.07915138, 1], [0.07912368, 0.07847866, 0.07810578, 1]]

initCorner = []
initCorners = []

Jab_reg_p2p = Jab_reg_p2p_1.transformation @ Jab_reg_p2p_2.transformation @ Jab_reg_p2p_3.transformation @ Jab_reg_p2p_4.transformation @ Jab_reg_p2p_5.transformation

for i in myInitCorner:
    i = np.array(i)
    initCorner.append(Jab_reg_p2p @ i)

for i in initCorner:
    arr = []
    for j in range(3):
        arr.append(i[j])
    initCorners.append(arr)

cubecornerV = []
initCorner_np = np.array(initCorners)
for i in range(len(initCorner_np)):
    cubecornerV.append(find_farthest_point2(Reconst_cube.points, initCorner_np[7 - i]))

# 6면의 9개 센터 좌표 반복문으로 HSV 정보 출력
face_names = ['U', 'R', 'F', 'D', 'L', 'B']
face_corners = [
    (cubecornerV[4], cubecornerV[5], cubecornerV[6], cubecornerV[7]),  # U
    (cubecornerV[1], cubecornerV[3], cubecornerV[5], cubecornerV[7]),  # R
    (cubecornerV[0], cubecornerV[1], cubecornerV[4], cubecornerV[5]),  # F
    (cubecornerV[2], cubecornerV[3], cubecornerV[0], cubecornerV[1]),  # D
    (cubecornerV[2], cubecornerV[0], cubecornerV[6], cubecornerV[4]),  # L
    (cubecornerV[3], cubecornerV[2], cubecornerV[7], cubecornerV[6]),  # B
]
Cubestring = ""
for face, corners in zip(face_names, face_corners):
    centers = getNineCenter(*corners)
    print(f"--- Face {face} ---")
    for idx, center in enumerate(centers):
        avg_color = get_nearest_neighbors_avg_color(center, Reconst_cube) * 255
        hsv = rgb_to_hsv(avg_color)
        color_name = get_color_name_hsv(hsv)
        print(f"Center {idx+1}: HSV={hsv}, Name={color_name}")
        Cubestring += color_name[0]
print("Cubestring:", Cubestring)


