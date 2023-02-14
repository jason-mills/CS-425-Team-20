import open3d as o3d
import numpy as np
import sys
import os.path

def render(filePath):
    splitPath = filePath.split(".")
    extension = splitPath[len(splitPath) - 1]
    print(extension)

    if extension == "pcd" or extension == "ply":
        fileData = o3d.io.read_point_cloud(filePath)
        o3d.visualization.draw_geometries([fileData])

    elif extension == "stl":
        fileData = o3d.io.read_triangle_mesh(filePath)
        # can change the color for the stl render
        fileData.paint_uniform_color([0.9, 0.1, 0.1])
        o3d.visualization.draw_geometries([fileData])
    

def main():
    if (len(sys.argv) - 1) != 1:
        print("Please provide command line arguments when running.\nExample: python render.py directory/filename.ply")
        return 1

    filePath = sys.argv[1]

    if not os.path.isfile(filePath):
        print("Not a valid path")
        return 1

    render(filePath)

if __name__ == '__main__':
    main()