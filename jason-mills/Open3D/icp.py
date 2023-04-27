import open3d as o3d
import numpy as np
import copy
import os
import sys
from Editor import Editor
from PointCloudStruct import PointCloudStruct

def all_registration(clouds, coarse_dist, fine_dist, voxelSize):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    num_clouds = len(clouds)
    
    for source_id in range(num_clouds):
        for target_id in range(source_id + 1, num_clouds):
            transformation_icp, information_icp = pairwise_registration(
                clouds[source_id], clouds[target_id],
                coarse_dist, fine_dist, voxelSize)
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
            else:
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph

def pairwise_registration(source, target, coarse_dist, fine_dist, voxelSize):
    print("Applying pairwise registration with point to plane icp...")
    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))
    icp_coarse = o3d.pipelines.registration.registration_icp(source,
                                                             target,
                                                             coarse_dist,
                                                             np.identity(4),
                                                             o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(source,
                                                             target,
                                                             fine_dist,
                                                             icp_coarse.transformation,
                                                             o3d.pipelines.registration.TransformationEstimationPointToPlane())
    trans_icp = icp_fine.transformation
    info_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(source,
                                                                                   target,
                                                                                   fine_dist,
                                                                                   icp_fine.transformation)
    return trans_icp, info_icp

def multiway_registration(source, downed_source, voxel_size):
    coarse_dist = voxel_size * 15
    fine_dist = voxel_size * 1.5

    pose_graph = all_registration(downed_source, coarse_dist, fine_dist, voxel_size)

    option = o3d.pipelines.registration.GlobalOptimizationOption(max_correspondence_distance=fine_dist,
                                                                 edge_prune_threshold=0.25,
                                                                 reference_node=0)
    
    o3d.pipelines.registration.global_optimization(pose_graph,
                                                   o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                                                   o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                                                   option)
    
    source[0].estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
    totalCloud = copy.deepcopy(source[0])

    for source_id in range(1, len(downed_source)):
        test = pose_graph.nodes[source_id].pose
        source[source_id].estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
        reg_p2p = o3d.pipelines.registration.registration_icp(source[source_id], 
                                                                  totalCloud, 
                                                                  voxel_size * 0.4, 
                                                                  pose_graph.nodes[source_id].pose, 
                                                                  o3d.pipelines.registration.TransformationEstimationPointToPlane())
        source[source_id].transform(reg_p2p.transformation)
        totalCloud += source[source_id]

    return totalCloud

# Read a supported file type
def read_file(file_path):
    if file_path.endswith(".xyz"):
        points = []
        fileStream = open(file_path, 'r')

        firstLine = fileStream.readline()
        if(firstLine == 'X Y Z\n'):
            for line in fileStream:
                line = line.rstrip('\n')
                x, y, z = line.split(' ')
                if x == '-0' or y == '-0' or z == '-0':
                    continue
                points.append([float(x), float(y), float(z)])

        new_cloud = o3d.geometry.PointCloud()
        new_cloud.points.extend(points)

        return new_cloud
    
    elif file_path.endswith((".xyzn", ".xyzrgb", ".pts", ".ply", ".pcd")):
        new_cloud = o3d.io.read_point_cloud(file_path)
        return new_cloud


# remove points below a certain y value to remove the platform from the scan
def remove_platform (cloud):
    points = np.asarray(cloud.points)
    mask = points[:,1] < 0.048
    cloud.points = o3d.utility.Vector3dVector(points[mask])

    return

# This function has to downsample the point cloud
def remove_outliers(pcd, voxel_size, iterations, numberOfPoints, radius):
    downPcd = pcd.voxel_down_sample(voxel_size)

    for i in range(iterations):
        cur_cloud, indices = downPcd.remove_radius_outlier(nb_points=numberOfPoints, radius=radius, print_progress=True)
        downPcd = downPcd.select_by_index(indices)
        cl, indices = downPcd.remove_statistical_outlier(nb_neighbors=numberOfPoints, std_ratio=1.0)
        downPcd = downPcd.select_by_index(indices)

    return downPcd

def process_scan(input_directory_path, input_file_base_name, input_file_extension, output_file_base_name, file_order, is_user_scan):
    files = os.listdir(input_directory_path)
    cloud_structs = []

    for i in range(len(files)):
        print("Reading " + files[i])
        file_path = input_directory_path + "/" + files[i]
        
        current_cloud = read_file(file_path)

        if(is_user_scan == "True"):
            remove_platform(current_cloud)

        voxel_size = round(max(current_cloud.get_max_bound() - current_cloud.get_min_bound()) * 0.01, 4)
        down_sample_voxel_size = round(max(current_cloud.get_max_bound() - current_cloud.get_min_bound()) * 0.0041, 4)

        if(is_user_scan == "True"):
            current_cloud = remove_outliers(current_cloud, voxel_size, 3, 200, voxel_size * 4)

        current_cloud_struct = PointCloudStruct(file_path, 
                                                current_cloud, 
                                                current_cloud.voxel_down_sample(voxel_size), 
                                                voxel_size, down_sample_voxel_size)
        
        cloud_structs.append(current_cloud_struct)

    return cloud_structs

def process_non_user_scan(input_directory_path, input_file_base_name, input_file_extension, output_file_base_name, file_order):
    print("something")

def main():
    if (len(sys.argv) - 1) != 6:
        print("Please provide command line arguments when running")
        print("Example: python icp.py input_directory_path input_file_base_name input_file_extension output_file_base_name file_order is_user_scan")
        return 1
    
    input_directory_path = sys.argv[1].replace("\\", "/")
    input_file_base_name = sys.argv[2]
    input_file_extension = sys.argv[3]
    output_file_base_name = sys.argv[4]
    file_order = sys.argv[5].split(",")
    is_user_scan = sys.argv[6]

    if not os.path.isdir(input_directory_path):
        print("Input directory is not valid")
        return 1
    
    cloud_structs = []

    cloud_structs = process_scan(input_directory_path, 
                                        input_file_base_name, 
                                        input_file_extension, 
                                        output_file_base_name, 
                                        file_order, is_user_scan)

    
    editor = Editor(cloud_structs)
    editor.update_visualizer(cloud_structs[0].cloud)
    editor.run_visualizer()
    editor.destroy_visualizer()

if __name__ == '__main__':
    main()