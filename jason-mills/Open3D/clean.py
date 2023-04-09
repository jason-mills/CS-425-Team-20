import numpy as np
import open3d as o3d
import sys
import os

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def removeOutliers(inputFile, outputFilename, outputFileType):
    pcd = o3d.io.read_point_cloud(inputFile)
    print(len(pcd.points))
    pcd = pcd.voxel_down_sample(voxel_size=0.002)
    print(len(pcd.points))
    # pcd.uniform_down_sample(every_k_points=5)
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=0.0001)
    # cl, ind = pcd.remove_radius_outlier(nb_points=30, radius=0.01)

    # plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    # inlier_cloud = pcd.select_by_index(inliers)

    display_inlier_outlier(pcd, ind)


def main():
    if (len(sys.argv) - 1) != 3:
        print("Please provide command line arguments when running.\nExample: python renconstruct.py directory/inputFilename directory/outputFilename outputFileType")
        return 1

    inputFile = sys.argv[1]
    outputFilename = sys.argv[2]
    outputFileType = sys.argv[3]

    if not os.path.isfile(inputFile):
        print("Not a valid path")
        return 1

    removeOutliers(inputFile, outputFilename, outputFileType)


if __name__ == '__main__':
    main()