import open3d as o3d
import numpy as np
import math

class XYZReader:
    def readFile(self, filePath):
        points = []
        print("reading")
        i = 0
        fileStream = open(filePath, 'r')
        firstLine = fileStream.readline()
        if(firstLine == 'X Y Z\n'):
            for line in fileStream:
                line = line.rstrip('\n')
                x, y, z = line.split(', ')
                if x == '-0' or y == '-0' or z == '-0':
                    continue
                # print(x, y, z)
                points.append([x, y, z])

        return points
            

def main():
    rotationAngle = 0
    radianConverter = (math.pi / 180)
    zRotator = np.matrix([[math.cos(rotationAngle * radianConverter), math.sin(rotationAngle * radianConverter), 0, 0],
                         [-math.sin(rotationAngle * radianConverter), math.cos(rotationAngle * radianConverter), 0, 0], 
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    sample = np.matrix([1, 0, 0, 0])
    
    # print(zRotator)
    # print(sample)
    print(sample * zRotator)
    # myReader = XYZReader()

    # points = np.asarray(myReader.readFile('XYZ/data.xyz'))
    # filePath = 'XYZ/data.xyz'

    # fileStream = open(filePath, 'r')
    # print(fileStream.readline())

    # points = np.array([[0, 0, 0]])
    # pcd = o3d.geometry.PointCloud()
    # pcd.points.extend(points)
    # pcd.points = o3d.utility.Vector3dVector(points)
    # filePath = 'PLY Files/bowl_1_0.ply'
    # pcd = o3d.io.read_point_cloud(filePath)

    # o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    main()