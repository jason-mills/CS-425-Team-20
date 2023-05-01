import open3d as o3d
import numpy as np
import os
import sys
from Editor import Editor
from Structs import PointCloudStruct
import open3d.visualization.gui as gui
import json
import argparse

def parseArgs():
    usage = '''python %(prog)s [-h] --dir DIR --prefix PREFIX --out OUT'''
    parser = argparse.ArgumentParser(usage=usage, add_help=False)
    required = parser.add_argument_group("Required")
    required.add_argument("--run_interactive_mode", 
                        type=str,  
                        help="Choose to run in interactive mode or not",
                        required=True)
    required.add_argument("--input_directory_path", 
                        type=str,  
                        help="The directory containing the files to work with",
                        required=True)
    required.add_argument("--output_directory_path", 
                        type=str,  
                        help="The output directory",
                        required=True)
    required.add_argument("--output_file_base_name", 
                        type=str,  
                        help="The output file base name",
                        required=True)
    required.add_argument("--is_user_scan", 
                        type=str,  
                        help="If the data is from user scan",
                        required=True)
    
    optional = parser.add_argument_group("optional")

    required.add_argument("--input_file_base_name", 
                        type=str,  
                        default="",
                        help="The base file name being read from",
                        required=False)
    required.add_argument("--file_order", 
                        type=str,  
                        default=" ",
                        help="The base file name being read from",
                        required=False)
    optional.add_argument("--merge_method", 
                        type=str, 
                        default="mutliway registration", 
                        help="The registration algorithm to use",
                        required=False)
    optional.add_argument("--output_file_type", 
                        type=str, 
                        default=".stl", 
                        help="Object file type being created",
                        required=False)
    required.add_argument("--input_file_extension", 
                        type=str,  
                        help="The extension being used, example: .xyz, .ply",
                        required=False)
    
    args = parser.parse_known_args()

    return args[0]

# Write metadata to json file
def write_to_json(file_path, key_value_pairs):
    if not (os.path.isfile(file_path)):
        file_data = {}

        for key_value_pair in key_value_pairs:
            file_data[key_value_pair[0]] = key_value_pair[1]

        with open(file_path, 'w') as file:
            json.dump(file_data, file, indent=4)

    else:
        with open(file_path, 'r+') as file:
            file_data = json.load(file)
            
        for key_value_pair in key_value_pairs:
            file_data[key_value_pair[0]] = key_value_pair[1]

        with open(file_path, 'w') as file:
            json.dump(file_data, file, indent=4)

    return

# Read a supported file type
def read_file(file_path):    
    if file_path.endswith((".xyz", ".xyzn", ".xyzrgb", ".pts", ".ply", ".pcd")):
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
        # cl, indices = downPcd.remove_statistical_outlier(nb_neighbors=numberOfPoints, std_ratio=1.0, print_progress=True)
        # downPcd = downPcd.select_by_index(indices)

    return downPcd

def process_scan(input_directory_path, input_file_base_name, input_file_extension, file_order, is_user_scan):
    file_paths = []
    if not input_file_base_name == "":
        for number in file_order:
            file_paths.append(input_directory_path + "/" + input_file_base_name + number + input_file_extension)
    else:
        files = os.listdir(input_directory_path)
        for file in files:
            file_paths.append(input_directory_path + "/" + file)

    cloud_structs = []

    for file_path in file_paths:
        print("Reading " + file_path)
        
        current_cloud = read_file(file_path)
        print()

        if is_user_scan:
            remove_platform(current_cloud)

        voxel_size = round(max(current_cloud.get_max_bound() - current_cloud.get_min_bound()) * 0.01, 4)
        down_sample_voxel_size = round(max(current_cloud.get_max_bound() - current_cloud.get_min_bound()) * 0.0041, 4)

        if is_user_scan:
            current_cloud = remove_outliers(current_cloud, down_sample_voxel_size, 3, 200, voxel_size * 4)

        current_cloud_struct = PointCloudStruct(file_path, 
                                                current_cloud, 
                                                current_cloud.voxel_down_sample(voxel_size), 
                                                voxel_size, down_sample_voxel_size)
        
        cloud_structs.append(current_cloud_struct)

    return cloud_structs

def main():
    args = parseArgs()

    run_interactive_mode = args.run_interactive_mode.lower() == "true"
    input_directory_path = args.input_directory_path.replace("\\", "/")
    input_file_base_name = args.input_file_base_name
    print("input file base name: " + input_file_base_name)
    input_file_extension = args.input_file_extension
    file_order = args.file_order.split(",")
    output_directory_path = args.output_directory_path.replace("\\", "/")
    output_file_base_name = args.output_file_base_name
    is_user_scan = args.is_user_scan.lower() == "true"
    output_file_type = args.output_file_type

    print(is_user_scan)
    if is_user_scan:
        print("this is a user scan")

    if not os.path.isdir(input_directory_path):
        print("Input directory is not valid")
        return 1
    
    cloud_structs = []

    cloud_structs = process_scan(input_directory_path, 
                                 input_file_base_name, 
                                 input_file_extension, 
                                 file_order, 
                                 is_user_scan)

    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(cloud_structs[0].cloud, cloud_structs[0].voxel_size * 3)
    all_voxels = voxel_grid.get_voxels()
    all_centers = []

    for voxel in all_voxels:
        voxel_center = voxel_grid.get_voxel_center_coordinate(voxel.grid_index)
        all_centers.append(voxel_center)

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(all_centers)

    cloud.estimate_normals()
    cloud.orient_normals_consistent_tangent_plane(100)
    distances = cloud.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist 

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        cloud,
        o3d.utility.DoubleVector([radius, radius * 2]))
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cloud, depth=15, width=0, scale=1.1, linear_fit=False)[0]
    # mesh = mesh.filter_smooth_simple(number_of_iterations=5)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Bunny Visualize', width=800, height=600)
    vis.add_geometry(mesh)
    vis.add_geometry(cloud)
    vis.run()


    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh("test.stl", mesh)
    # if run_interactive_mode:
    #     gui.Application.instance.initialize()
    #     editor = Editor(cloud_structs, output_directory_path, output_file_base_name, output_file_type, run_interactive_mode)
    #     gui.Application.instance.run()
    #     gui.Application.instance.quit()
    # else:
    #     editor = Editor(cloud_structs, output_directory_path, output_file_base_name, output_file_type, run_interactive_mode)
    
    # write_to_json(output_directory_path + "/metadata.json", editor.metadata)  
    print("done here")
    
if __name__ == '__main__':
    main()