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

    def createPly(self, inputDir, xyzBaseName, outputDir, plyBaseName):
        files = os.listdir(inputDir)
        filesToConvert = []

        for file in files: 
            if(file.startswith(xyzBaseName) and file.endswith(".xyz")):
                filesToConvert.append(file)

        for i in range(len(filesToConvert)):
            pcd = o3d.geometry.PointCloud()
            points = np.asarray(self.readFile(inputDir + '/' + filesToConvert[i]))
            pcd.points.extend(points)
            downpcd = pcd.voxel_down_sample(voxel_size=0.0005)

            o3d.io.write_point_cloud(outputDir + '/' + plyBaseName + str(i) + ".ply", downpcd, write_ascii=True)

            rewrite = open(outputDir + '/' + plyBaseName + str(i) + ".ply", "r")
            content = rewrite.read()
            content = content.replace("double", "float")
            rewrite.close()

            rewrite = open(outputDir + '/' + plyBaseName + str(i) + ".ply", "w")
            rewrite.write(content)
            rewrite.close()
        
        return 0

# this script takes command line arguments so it can be called by other programs
def main():
    if (len(sys.argv) - 1) != 4:
        print("Please provide command line arguments when running.\nExample: python xyzToPly.py inputDir xyzBaseName outputDirectory plyBaseName")
        return 1

    inputDir = sys.argv[1]
    xyzBaseName = sys.argv[2]
    outputDir = sys.argv[3]
    plyBaseName = sys.argv[4]

    if not os.path.isdir(inputDir):
        print("Input direcory is not valid")
        return 1
    
    if not os.path.isdir(outputDir):
        print("Output directory is not valid")
        return 1

    myReader = XYZReader()

    return myReader.createPly(inputDir, xyzBaseName, outputDir, plyBaseName)

if __name__ == '__main__':
    main()