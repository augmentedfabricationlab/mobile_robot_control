import open3d as o3d


def downsample_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    return pcd_down

def preprocess_point_cloud_global(pcd, search_radius):
    radius_normal = search_radius
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=300))

    radius_feature = search_radius * 3
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=1000))
    return pcd, pcd_fpfh

def execute_global_registration(source, target, source_fpfh,
                                target_fpfh, distance_threshold=0.1):
    print(":: RANSAC registration on downsampled point clouds.")
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(10**5, 0.999))
    return result

def execute_global_registration2(source, target, source_fpfh,
                                target_fpfh, distance_threshold=0.1):
    print(":: RANSAC registration on downsampled point clouds.")
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_correspondence(
        source, target, 
        o3d.pipelines.registration.correspondences_from_features(source_fpfh, target_fpfh),
        distance_threshold)
    return result