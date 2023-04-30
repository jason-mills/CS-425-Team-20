import open3d as o3d
import numpy as np
import os
import sys
from Editor import Editor
from Structs import PointCloudStruct
import open3d.visualization.gui as gui
import json

# Write metadata to json file
def write_to_json(file_path, key_value_pairs):
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
        cl, indices = downPcd.remove_statistical_outlier(nb_neighbors=numberOfPoints, std_ratio=1.0, print_progress=True)
        downPcd = downPcd.select_by_index(indices)

    return downPcd

def process_scan(input_directory_path, input_file_base_name, input_file_extension, file_order, is_user_scan):
    file_paths = []
    if not input_file_base_name == " ":
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
    if (len(sys.argv) - 1) != 7:
        print("Please provide command line arguments when running")
        print("Example: python EditorDriver.py input_directory_path input_file_base_name input_file_extension file_order output_directory_path output_file_base_name is_user_scan")
        return 1
    
    input_directory_path = sys.argv[1].replace("\\", "/")
    input_file_base_name = sys.argv[2]
    input_file_extension = sys.argv[3]
    file_order = sys.argv[4].split(",")
    output_directory_path = sys.argv[5].replace("\\", "/")
    output_file_base_name = sys.argv[6]
    is_user_scan = sys.argv[7].lower() == "true"

    if not os.path.isdir(input_directory_path):
        print("Input directory is not valid")
        return 1
    
    cloud_structs = []

    cloud_structs = process_scan(input_directory_path, 
                                 input_file_base_name, 
                                 input_file_extension, 
                                 file_order, 
                                 is_user_scan)

    gui.Application.instance.initialize()
    editor = Editor(cloud_structs, output_directory_path, output_file_base_name)
    gui.Application.instance.run()
    gui.Application.instance.quit()
    
    # write_to_json(output_directory_path + "/metadata.json", editor.metadata)  
    
if __name__ == '__main__':
    main()