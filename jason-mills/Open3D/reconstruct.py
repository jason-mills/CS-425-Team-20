import numpy as np
import open3d as o3d
import sys
import os

def reconstruct(inputFile, outputFilename, outputFileType):
    pcd = o3d.io.read_point_cloud(inputFile)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,max_nn=30))

    o3d.visualization.draw_geometries([pcd])


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

    reconstruct(inputFile, outputFilename, outputFileType)


if __name__ == '__main__':
    main()