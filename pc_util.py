from open3d import *
import numpy as np
import copy
import time

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
#     source_temp.paint_uniform_color([1, 0.706, 0])
#     target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

def preprocess_point_cloud(pcd, voxel_size):
#     print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
#     print(":: Estimate normal with search radius %.3f." % radius_normal)
    estimate_normals(pcd_down, KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 30))

    radius_feature = voxel_size * 5
#     print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = compute_fpfh_feature(pcd_down,
            KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
    return pcd_down, pcd_fpfh, radius_normal

def prepare_dataset(voxel_size, source, target):
#     print(":: Load two point clouds and disturb initial pose.")
#     source = read_point_cloud("../wcom_estool/rgb0.pcd")
#     target = read_point_cloud("../wcom_estool/rgb1.pcd")
#     trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0],
#                             [1.0, 0.0, 0.0, 0.0],
#                             [0.0, 1.0, 0.0, 0.0],
#                             [0.0, 0.0, 0.0, 1.0]])
#     source.transform(trans_init)
#     draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh, radius = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh, _ = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh, radius

def execute_global_registration(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
#     print(":: RANSAC registration on downsampled point clouds.")
#     print("   Since the downsampling voxel size is %.3f," % voxel_size)
#     print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            distance_threshold,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            RANSACConvergenceCriteria(4000000, 500))
    return result

def execute_fast_global_registration(source_down, target_down,
        source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
#     print(":: Apply fast global registration with distance threshold %.3f" \
#             % distance_threshold)
    result = registration_fast_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            FastGlobalRegistrationOption(
            maximum_correspondence_distance = distance_threshold))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size,radius,roughTrans):
    distance_threshold = voxel_size * 0.4
#     print(":: Point-to-plane ICP registration is applied on original point")
#     print("   clouds to refine the alignment. This time we use a strict")
#     print("   distance threshold %.3f." % distance_threshold)
    result = registration_icp(source, target, distance_threshold,
            roughTrans,
            TransformationEstimationPointToPlane())
#     result = registration_colored_icp(source, target,
#                 radius, result_ransac.transformation,
#                 ICPConvergenceCriteria(relative_fitness = 1e-6,
#                 relative_rmse = 1e-6, max_iteration = 5))
    return result