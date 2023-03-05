import open3d as o3d
import numpy as np
import sys
import os

# class for reading x y z files and function for creating .ply from them
class XYZReader:
    def readFile(self, filePath):
        points = []
        fileStream = open(filePath, 'r')

        firstLine = fileStream.readline()
        if(firstLine == 'X Y Z\n'):
            for line in fileStream:
                line = line.rstrip('\n')
                x, y, z = line.split(' ')
                if x == '-0' or y == '-0' or z == '-0':
                    continue
                points.append([x, y, z])

        return points

    def createPly(self, filePath, plyFilepath):
        if not (filePath.endswith('.xyz')):
            return 1

        pcd = o3d.geometry.PointCloud()
        points = np.asarray(self.readFile(filePath))
        pcd.points.extend(points)
        o3d.io.write_point_cloud(plyFilepath, pcd)
        
        return 0

# this script takes command line arguments so it can be called by other programs
def main():
    if (len(sys.argv) - 1) != 2:
        print("Please provide command line arguments when running.\nExample: python xyzToPly.py filepath filepathForNewPly.ply")
        return 1

    filePath = sys.argv[1]
    plyFilepath = sys.argv[2]

    if not os.path.isfile(filePath):
        print("Not a valid file")
        return 1
    
    myReader = XYZReader()
    return myReader.createPly(filePath, plyFilepath)

if __name__ == '__main__':
    main()