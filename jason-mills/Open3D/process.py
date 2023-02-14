import open3d as o3d
import numpy as np

def load_point_clouds(voxel_size=0.0):
    pcds = []
    for i in range(9):
        pcd = o3d.io.read_point_cloud("PLY Files/bowl_1_%d.ply" %i)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds

def pairwise_registration(source, target, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id], max_correspondence_distance_coarse, max_correspondence_distance_fine)
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph

# def main():
# voxel_size = 0.02
# pcds_down = load_point_clouds(voxel_size)
ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud("PLY Files/bowl_1_0.ply")
downpcd = pcd.voxel_down_sample(voxel_size=0.05)
downpcd.estimate_normals(
search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
# o3d.visualization.draw_geometries([downpcd], point_show_normal=True)
normals = (np.asarray(downpcd.normals))
print(normals[0])
# pcd.estimate_normals(downpcd, search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 1.0, max_nn = 5))
# o3d.draw_geometries([downpcd])
# print("Full registration ...")
# max_correspondence_distance_coarse = voxel_size * 15
# max_correspondence_distance_fine = voxel_size * 1.5
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     pose_graph = full_registration(pcds_down,
#                                    max_correspondence_distance_coarse,
#                                    max_correspondence_distance_fine)
# print("Optimizing PoseGraph ...")
# option = o3d.pipelines.registration.GlobalOptimizationOption(
#     max_correspondence_distance=max_correspondence_distance_fine,
#     edge_prune_threshold=0.25,
#     reference_node=0)
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     o3d.pipelines.registration.global_optimization(
#         pose_graph,
#         o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
#         o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
#         option)
# print("Transform points and display")
# for point_id in range(len(pcds_down)):
#     print(pose_graph.nodes[point_id].pose)
#     pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
# pcds = load_point_clouds(voxel_size)
# pcd_combined = o3d.geometry.PointCloud()
# for point_id in range(len(pcds)):
#     pcds[point_id].transform(pose_graph.nodes[point_id].pose)
#     pcd_combined += pcds[point_id]
# pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
# o3d.io.write_point_cloud("multiway_registration.pcd", pcd_combined_down)
# o3d.visualization.draw_geometries([pcd_combined_down],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

# if __name__ == '__main__':
#     main()