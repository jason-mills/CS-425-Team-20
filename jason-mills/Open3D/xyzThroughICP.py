import open3d as o3d
import numpy as np
import sys
import os

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

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
                points.append([float(x), float(y), float(z)])

        return points

    def removePlatform (self, points):
        newPoints = []
        for point in points:
            if point[1] < 0.041:
                newPoints.append(point)

        return newPoints
            


    # def createPly(self, inputDir, xyzBaseName, outputDir, plyBaseName):
    #     files = os.listdir(inputDir)
    #     filesToConvert = []

    #     for file in files: 
    #         if(file.startswith(xyzBaseName) and file.endswith(".xyz")):
    #             filesToConvert.append(file)

    #     for i in range(len(filesToConvert)):
    #         pcd = o3d.geometry.PointCloud()
    #         points = np.asarray(self.readFile(inputDir + '/' + filesToConvert[i]))
    #         pcd.points.extend(points)
    #         downpcd = pcd.voxel_down_sample(voxel_size=0.0005)

    #         o3d.io.write_point_cloud(outputDir + '/' + plyBaseName + str(i) + ".ply", downpcd, write_ascii=True)

    #         rewrite = open(outputDir + '/' + plyBaseName + str(i) + ".ply", "r")
    #         content = rewrite.read()
    #         content = content.replace("double", "float")
    #         rewrite.close()

    #         rewrite = open(outputDir + '/' + plyBaseName + str(i) + ".ply", "w")
    #         rewrite.write(content)
    #         rewrite.close()
        
        return 0



# this script takes command line arguments so it can be called by other programs
def main():
    if (len(sys.argv) - 1) != 3:
        print("Please provide command line arguments when running.\nExample: python xyzToPly.py inputDir xyzBaseName outputDirectory plyBaseName")
        return 1

    inputDir = sys.argv[1]
    xyzBaseName = sys.argv[2]
    outputDir = sys.argv[3]
    # plyBaseName = sys.argv[4]

    if not os.path.isdir(inputDir):
        print("Input direcory is not valid")
        return 1
    
    if not os.path.isdir(outputDir):
        print("Output directory is not valid")
        return 1

    myReader = XYZReader()
    pcd0 = o3d.geometry.PointCloud()
    pcd1 = o3d.geometry.PointCloud()
    pcd2 = o3d.geometry.PointCloud()
    pcd3 = o3d.geometry.PointCloud()
    pcd4 = o3d.geometry.PointCloud()
    pcd5 = o3d.geometry.PointCloud()
    pcd6 = o3d.geometry.PointCloud()
    pcd7 = o3d.geometry.PointCloud()
    pcd8 = o3d.geometry.PointCloud()
    pcd9 = o3d.geometry.PointCloud()
    pcd10 = o3d.geometry.PointCloud()
    pcd11 = o3d.geometry.PointCloud()

    
    # myReader.readFile(inputDir + "/" + xyzBaseName + "0" + ".xyz")
    # points = myReader.readFile(inputDir + "/" + xyzBaseName + "0" + ".xyz")
    # myReader.readFile(inputDir + "/" + xyzBaseName + "0" + ".xyz")
    # points = myReader.readFile(inputDir + "/" + xyzBaseName + "0" + ".xyz")
    points = myReader.readFile(inputDir + "/" + xyzBaseName + "0" + ".xyz")
    points = myReader.removePlatform(points)
    pcd0.points.extend(points)
    pcd0 = pcd0.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd0.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd0
        pcd0 = cloud.select_by_index(ind)
    
    points = myReader.readFile(inputDir + "/" + xyzBaseName + "1" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd1 = cloud.select_by_index(ind)

    points = myReader.readFile(inputDir + "/" + xyzBaseName + "2" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd2 = cloud.select_by_index(ind)

    points = myReader.readFile(inputDir + "/" + xyzBaseName + "3" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd3 = cloud.select_by_index(ind)

    points = myReader.readFile(inputDir + "/" + xyzBaseName + "4" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd4 = cloud.select_by_index(ind)

    points = myReader.readFile(inputDir + "/" + xyzBaseName + "5" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd5 = cloud.select_by_index(ind)

    points = myReader.readFile(inputDir + "/" + xyzBaseName + "6" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd6 = cloud.select_by_index(ind)

    points = myReader.readFile(inputDir + "/" + xyzBaseName + "7" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd7 = cloud.select_by_index(ind)

    points = myReader.readFile(inputDir + "/" + xyzBaseName + "8" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd8 = cloud.select_by_index(ind)

    points = myReader.readFile(inputDir + "/" + xyzBaseName + "9" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd9 = cloud.select_by_index(ind)

    points = myReader.readFile(inputDir + "/" + xyzBaseName + "10" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd10 = cloud.select_by_index(ind)
    
    points = myReader.readFile(inputDir + "/" + xyzBaseName + "11" + ".xyz")
    points = myReader.removePlatform(points)
    pcd1.points.extend(points)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.0009)
    for i in range(3):
        cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
        cloud = pcd1
        pcd11 = cloud.select_by_index(ind)

    # cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
    # cloud = pcd1
    # pcd1 = cloud.select_by_index(ind)
    # pcd2 = cloud.select_by_index(ind, invert=True)
    # cl, ind = pcd1.remove_radius_outlier(nb_points=55, radius=0.004)
    # cloud = pcd1
    # pcd1 = cloud.select_by_index(ind)
    # pcd2 = cloud.select_by_index(ind, invert=True)
    # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=0.001)
    # display_inlier_outlier(pcd, ind)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "1" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "2" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "3" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "4" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "5" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "6" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "7" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "8" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "9" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "10" + ".xyz"))
    # pcd.points.extend(points)
    # points = myReader.removePlatform(myReader.readFile(inputDir + "/" + xyzBaseName + "11" + ".xyz"))
    # pcd.points.extend(points)


    
    # points = np.asarray(myReader.readFile('blank.xyz'))
    # print(len(points))
    # pcd2.paint_uniform_color([1, 0, 0])
    # pcd1.paint_uniform_color([0.8, 0.8, 0.8])
    # o3d.visualization.draw_geometries([pcd0, pcd1, pcd2, pcd3, pcd4, pcd5, pcd6, pcd7, pcd8, pcd9, pcd10, pcd11])

    o3d.io.write_point_cloud("out0.ply", pcd0, write_ascii=True)
    o3d.io.write_point_cloud("out1.ply", pcd1, write_ascii=True)
    o3d.io.write_point_cloud("out2.ply", pcd2, write_ascii=True)
    o3d.io.write_point_cloud("out3.ply", pcd3, write_ascii=True)
    o3d.io.write_point_cloud("out4.ply", pcd4, write_ascii=True)
    o3d.io.write_point_cloud("out5.ply", pcd5, write_ascii=True)
    o3d.io.write_point_cloud("out6.ply", pcd6, write_ascii=True)
    o3d.io.write_point_cloud("out7.ply", pcd7, write_ascii=True)
    o3d.io.write_point_cloud("out8.ply", pcd8, write_ascii=True)
    o3d.io.write_point_cloud("out9.ply", pcd9, write_ascii=True)
    o3d.io.write_point_cloud("out10.ply", pcd10, write_ascii=True)
    o3d.io.write_point_cloud("out11.ply", pcd11, write_ascii=True)

    
    # o3d.visualization.draw_geometries([pcd1])
    # o3d.visualization.draw_geometries([pcd2])


    # points = np.asarray(myReader.removePlatform(myReader.readFile('blank.xyz')))


    # myReader = XYZReader()

    # return myReader.createPly(inputDir, xyzBaseName, outputDir, plyBaseName)

if __name__ == '__main__':
    main()