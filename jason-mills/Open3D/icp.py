import open3d as o3d
import numpy as np
import copy
import random
from time import sleep
import time
import os
import sys
import msvcrt
import Editor
import PointCloudStruct

# paint source cloud and target cloud for easier visualization
def paint_registration_result(source_cloud, target_cloud):
    source_cloud.paint_uniform_color([0.54, 0, 0])
    target_cloud.paint_uniform_color([0.5, 0.5, 0.5])

# display outliers given cloud and indices of inliers
def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

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

#may modify in the future to take different icp iterations and different threshold (voxelSize * 1.5)
def pointToPointMerge(cloudsAndSamples, voxelSize):
    totalCloud = o3d.geometry.PointCloud()
    downSampleTotal = o3d.geometry.PointCloud()

    base = cloudsAndSamples[0]
    print("Adding base points")
    totalCloud.points.extend(base[0].points)
    totalCloud.colors.extend(base[0].colors)
    downSampleTotal.points.extend(base[1].points)

    for cloudSamplePair in cloudsAndSamples:
        if cloudSamplePair != base:
            print("Adding: " + cloudSamplePair[2])
            sourceDown, sourceFpfh = preprocess_point_cloud(cloudSamplePair[1], voxel_size=voxelSize)
            targetDown, targetFpfh = preprocess_point_cloud(downSampleTotal, voxel_size=voxelSize)
            ransacResult = execute_global_registration(sourceDown, targetDown, sourceFpfh, targetFpfh, voxelSize)
            # sourceDown.transform(ransacResult.transformation)
            downSampleTotal.points.extend(sourceDown.points)
            reg_p2p = o3d.pipelines.registration.registration_icp(cloudSamplePair[0], 
                                                                  totalCloud, 
                                                                  voxelSize * 1.5,
                                                                  ransacResult.transformation,
                                                                    o3d.pipelines.registration.TransformationEstimationPointToPoint(), o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
            # reg_p2p = o3d.pipelins.registration.registration_icp(cloudSamplePair[0], totalCloud, voxelSize * 1.5, ransacResult.transformation)
            cloudSamplePair[0].transform(reg_p2p.transformation)
            totalCloud.points.extend(cloudSamplePair[0].points)
            totalCloud.colors.extend(cloudSamplePair[0].colors)


            # o3d.visualization.draw_geometries([downSampleTotal])
            # o3d.visualization.draw_geometries([totalCloud])
            downSampleTotal = totalCloud.voxel_down_sample(voxelSize)
            # downSampleTotal = totalCloud
            # thing = input("Stop here if you want traveller")

    
    return totalCloud

# perform point to plane merging
def point_to_plane_merge(cloud_structs, voxelSize):
    totalCloud = o3d.geometry.PointCloud()
    downSampleTotal = o3d.geometry.PointCloud()

    base = cloud_structs[0]
    print("Adding: " + base.name)
    cloud_structs.pop(0)

    totalCloud = base.cloud
    downSampleTotal = base.downsampled_cloud

    key = "n"
    source_down = None
    sourceFpfh = None
    target_down = None
    targetFpfh = None

    while len(cloud_structs) > 0:
        if key == "n":
            source_down, sourceFpfh = preprocess_point_cloud(cloud_structs[0].downsampled_cloud, voxel_size=voxelSize)
            target_down, targetFpfh = preprocess_point_cloud(downSampleTotal, voxel_size=voxelSize)
            print("Adding: " + cloud_structs[0].name)
            ransacResult = execute_global_registration(source_down, target_down, sourceFpfh, targetFpfh, voxelSize)
            sleep(1)

            cloud_structs[0].cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))
            totalCloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))

            transformation = execute_point_to_plane_refinement(cloud_structs[0].cloud, totalCloud, voxelSize * 1.5, ransacResult.transformation)
            key = ""

        elif key == "y":
            cloud_structs[0].cloud.transform(transformation)
            totalCloud += cloud_structs[0].cloud
            totalCloud = totalCloud.voxel_down_sample(cloud_structs[0].down_sample_voxel_size)
            downSampleTotal = totalCloud.voxel_down_sample(voxelSize)
            cloud_structs.pop(0)
            key = "n"

        elif key == "s":
            cloud_structs.append(cloud_structs[0])
            cloud_structs.pop(0)
            key = "n"

        elif key == "z":
            return totalCloud

        if(msvcrt.kbhit()):
            print("gotcha")
            key = msvcrt.getch().decode("ASCII")

    return totalCloud

def colorPointToPoint(cloudsAndSamples, voxelSize):
    totalCloud = o3d.geometry.PointCloud()
    downSampleTotal = o3d.geometry.PointCloud()

    base = cloudsAndSamples[0]
    print("Adding base points")
    totalCloud = base[0]
    downSampleTotal = base[1]

    for cloudSamplePair in cloudsAndSamples:
        if cloudSamplePair != base:
            print("Adding: " + cloudSamplePair[2])
            sourceDown, sourceFpfh = preprocess_point_cloud(cloudSamplePair[1], voxel_size=voxelSize)
            targetDown, targetFpfh = preprocess_point_cloud(downSampleTotal, voxel_size=voxelSize)
            entry = ""
            while(entry != "y"):
                ransacResult = execute_global_registration(sourceDown, targetDown, sourceFpfh, targetFpfh, voxelSize)
                entry = input("Good(y/n): ")

            sourceDown.transform(ransacResult.transformation)
            downSampleTotal.points.extend(sourceDown.points)
            cloudSamplePair[0].estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))
            totalCloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))
            reg_p2p = o3d.pipelines.registration.registration_colored_icp(cloudSamplePair[0], totalCloud, voxelSize*0.05, ransacResult.transformation,
                                                                           o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                                                                           o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
            cloudSamplePair[0].transform(reg_p2p.transformation)
            totalCloud.points.extend(cloudSamplePair[0].points)
            totalCloud.colors.extend(cloudSamplePair[0].colors)

            # o3d.visualization.draw_geometries([downSampleTotal])
            # o3d.visualization.draw_geometries([totalCloud])
            downSampleTotal = totalCloud.voxel_down_sample(voxelSize)
            # downSampleTotal = totalCloud
            # thing = input("Stop here if you want traveller")

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
        # display_inlier_outlier(cloud, indices)
    

    return downPcd

# estimate normals and compute 33 dimensional FPHP vector
def preprocess_point_cloud(point_cloud, voxel_size):
    radius_normal = voxel_size * 2
    point_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(point_cloud, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return point_cloud, pcd_fpfh

# get a general transformation matrix to apply to align points
def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print("Global Registration Started")
    registration_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(source_down,
                                                                                        target_down,
                                                                                        source_fpfh, 
                                                                                        target_fpfh, 
                                                                                        True, 
                                                                                        distance_threshold, 
                                                                                        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 
                                                                                        3, 
                                                                                        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                                                                                        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], 
                                                                                        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    
    transformed_cloud = copy.deepcopy(source_down)
    transformed_cloud.transform(registration_result.transformation)
    target_copy = copy.deepcopy(target_down)
    print("updating cloud")

    return registration_result

def execute_point_to_point_refinement(source, target, voxel_size, initial_transformation):
    cloud_to_transform = o3d.geometry.PointCloud()
    combined_cloud = o3d.geometry.PointCloud()

    print("Local Point to Point Refinement Started")
    registration_result = o3d.pipelines.registration.registration_icp(source, 
                                                            target, 
                                                            voxel_size * 1.5,
                                                            initial_transformation,
                                                            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
    
    cloud_to_transform = copy.deepcopy(source)
    combined_cloud = cloud_to_transform.transform(registration_result.transformation) + target

    return combined_cloud, registration_result.fitness
    

def execute_point_to_plane_refinement(source, target, voxel_size, initial_transformation):
    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2,
                                                                max_nn=30))
    
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2,
                                                                max_nn=30))

    print("Local Point to Plane Refinement Started")
    registration_result = o3d.pipelines.registration.registration_icp(source,
                                                                       target,
                                                                        voxel_size * 1.5,
                                                                        initial_transformation,
                                                                        o3d.pipelines.registration.TransformationEstimationPointToPlane())

    transformed_cloud = copy.deepcopy(source)
    transformed_cloud.transform(registration_result.transformation)

    return registration_result.transformation

def execute_color_point_to_point_refinement(source, target, voxel_size, initial_transformation):
    cloud_to_transform = o3d.geometry.PointCloud()
    combined_cloud = o3d.geometry.PointCloud()

    print("Local Color Point to Point Refinement Started")
    registration_result = o3d.pipelines.registration.registration_colored_icp(source,
                                                                   target,
                                                                    voxel_size*0.05,
                                                                    initial_transformation,
                                                                    o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                                                                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))

    cloud_to_transform = copy.deepcopy(source)
    combined_cloud = cloud_to_transform.transform(registration_result.transformation) + target

    return combined_cloud, registration_result.fitness

def display_point_cloud(pcd): 
    o3d.visualization.draw_geometries([pcd])

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
    editor.add_base_points()
    editor.update_visualizer(cloud_structs[0].cloud)


    # previous_t = time.time()
    # dt = 0.01

    # keep_running = True

    # while keep_running:
    #     if time.time() - previous_t > dt:
    #         previous_t = time.time()

    #     keep_running = editor.poll()

if __name__ == '__main__':
    main()