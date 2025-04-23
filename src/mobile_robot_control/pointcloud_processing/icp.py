import open3d.pipelines.registration as reg
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_rhino.conversions import plane_to_compas_frame
import numpy as np
import open3d as o3d


def preprocess_point_cloud_icp(pcd, voxel_size=0.05, max_neighbors=30):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=max_neighbors))

    return pcd_down

def evaluate_local_registration(source, target, threshold, estimation_plane):
    # Initial alignment or source to target transform.
    T = Transformation.from_frame_to_frame(Frame.worldXY(), plane_to_compas_frame(estimation_plane))
    trans_init = np.asarray(T.matrix)

    # Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
    evaluation = reg.evaluate_registration(source, target, threshold, trans_init)
    return trans_init, evaluation

def execute_icp_local_registration(source, target, threshold, initial_transformation):
    mu, sigma = 0, 0.1 # mean and standard deviation
    loss = reg.TukeyLoss(k=sigma)
    p2l = reg.TransformationEstimationPointToPoint()

    registration_icp = reg.registration_icp(source, target, threshold, initial_transformation, p2l,
                                            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000))

    return registration_icp