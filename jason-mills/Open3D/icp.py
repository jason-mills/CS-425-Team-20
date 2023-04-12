import open3d as o3d
import numpy as np
import copy
import random
from time import sleep

# class PointCloud():
#     def __init__(self, name, cloud, downsampledCloud):
#         self.name = name
#         self.cloud = cloud
#         self.downsampledCloud = downsampledCloud


#may modify in the future to take different icp iterations and different threshold (voxelSize * 1.5)
def pointToPointMerge(cloudsAndSamples, voxelSize):
    totalCloud = o3d.geometry.PointCloud()
    downSampleTotal = o3d.geometry.PointCloud()

    base = cloudsAndSamples[0]
    print("Adding base points")
    # resultClouds.append(base[0])
    totalCloud.points.extend(base[0].points)
    totalCloud.colors.extend(base[0].colors)
    downSampleTotal.points.extend(base[1].points)

    # resultClouds.append(base)
    for cloudSamplePair in cloudsAndSamples:
        if cloudSamplePair != base:
            print("Adding: " + cloudSamplePair[2])
            sourceDown, sourceFpfh = preprocess_point_cloud(cloudSamplePair[1], voxel_size=voxelSize)
            targetDown, targetFpfh = preprocess_point_cloud(downSampleTotal, voxel_size=voxelSize)
            ransacResult = execute_global_registration(sourceDown, targetDown, sourceFpfh, targetFpfh, voxelSize)
            # sourceDown.transform(ransacResult.transformation)
            # downSampleTotal.points.extend(sourceDown.points)
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
            sleep(5)
            downSampleTotal = totalCloud.voxel_down_sample(voxelSize)
            # downSampleTotal = totalCloud
            # thing = input("Stop here if you want traveller")

    
    return totalCloud

#may modify in the future to take different icp iterations and different threshold (voxelSize * 1.5)
def pointToPlaneMerge(cloudsAndSamples, voxelSize):
    totalCloud = o3d.geometry.PointCloud()
    downSampleTotal = o3d.geometry.PointCloud()

    base = cloudsAndSamples[0]
    print("Adding base points")
    # resultClouds.append(base[0])
    totalCloud.points.extend(base[0].points)
    totalCloud.colors.extend(base[0].colors)
    downSampleTotal.points.extend(base[1].points)

    # resultClouds.append(base)
    for cloudSamplePair in cloudsAndSamples:
        if cloudSamplePair != base:
            print("Adding: " + cloudSamplePair[2])
            sourceDown, sourceFpfh = preprocess_point_cloud(cloudSamplePair[1], voxel_size=voxelSize)
            targetDown, targetFpfh = preprocess_point_cloud(downSampleTotal, voxel_size=voxelSize)
            ransacResult = execute_global_registration(sourceDown, targetDown, sourceFpfh, targetFpfh, voxelSize)
            sourceDown.transform(ransacResult.transformation)
            downSampleTotal.points.extend(sourceDown.points)
            cloudSamplePair[0].estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))
            # cloudSamplePair[0].orient_normals_consistent_tangent_plane(100)
            totalCloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))
            # totalCloud.orient_normals_consistent_tangent_plane(100)
            reg_p2p = o3d.pipelines.registration.registration_icp(cloudSamplePair[0], totalCloud, voxelSize * 1.5, ransacResult.transformation, o3d.pipelines.registration.TransformationEstimationPointToPlane())
            cloudSamplePair[0].transform(reg_p2p.transformation)
            totalCloud.points.extend(cloudSamplePair[0].points)
            totalCloud.colors.extend(cloudSamplePair[0].colors)

            # o3d.visualization.draw_geometries([downSampleTotal])
            # o3d.visualization.draw_geometries([totalCloud])
            downSampleTotal = totalCloud.voxel_down_sample(voxelSize)
            # downSampleTotal = totalCloud
            # thing = input("Stop here if you want traveller")

    
    return totalCloud

def colorPointToPoint(cloudsAndSamples, voxelSize):
    totalCloud = o3d.geometry.PointCloud()
    downSampleTotal = o3d.geometry.PointCloud()

    base = cloudsAndSamples[0]
    print("Adding base points")
    # resultClouds.append(base[0])
    totalCloud.points.extend(base[0].points)
    totalCloud.colors.extend(base[0].colors)
    downSampleTotal.points.extend(base[1].points)

    # resultClouds.append(base)
    for cloudSamplePair in cloudsAndSamples:
        if cloudSamplePair != base:
            print("Adding: " + cloudSamplePair[2])
            sourceDown, sourceFpfh = preprocess_point_cloud(cloudSamplePair[1], voxel_size=voxelSize)
            targetDown, targetFpfh = preprocess_point_cloud(downSampleTotal, voxel_size=voxelSize)
            ransacResult = execute_global_registration(sourceDown, targetDown, sourceFpfh, targetFpfh, voxelSize)
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

            o3d.visualization.draw_geometries([downSampleTotal])
            o3d.visualization.draw_geometries([totalCloud])
            downSampleTotal = totalCloud.voxel_down_sample(voxelSize)
            # downSampleTotal = totalCloud
            # thing = input("Stop here if you want traveller")

    return totalCloud

def readFile(filePath):
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

def remove_platform (points):
    newPoints = []
    for point in points:
        if point[1] < (0.041):
            newPoints.append(point)

    return newPoints

# This function has to downsample the point cloud
def remove_outliers(pcd, voxelSize, iterations, numberOfPoints, radius):
    # downPcd = pcd.voxel_down_sample(voxel_size=voxelSize)

    for i in range(iterations):
        cl, indices = pcd.remove_radius_outlier(nb_points=numberOfPoints, radius=radius)
        cloud = pcd
        pcd = cloud.select_by_index(indices)

    return pcd


def preprocess_point_cloud(pcd, voxel_size):
    # pcd = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %f," % voxel_size)
    # print("   we use a liberal distance threshold %f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(source_down, target_down, source_fpfh, target_fpfh, True,distance_threshold,o3d.pipelines.registration.TransformationEstimationPointToPoint(False),3, [ o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

# def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
#     distance_threshold = voxel_size * 0.4
#     print(":: Point-to-plane ICP registration is applied on original point")
#     print("   clouds to refine the alignment. This time we use a strict")
#     print("   distance threshold %.3f." % distance_threshold)
#     result = o3d.pipelines.registration.registration_icp(
#         source, target, distance_threshold, result_ransac.transformation,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane())
#     return result

def display_point_cloud(pcd): 
    o3d.visualization.draw_geometries([pcd])

def main():
    # cloudsToCombine = []
    # names = []
    
    # for i in range(4):
    #     print("Reading: " + "In/temp" + str(i) + ".xyz")
    #     currentCloud = o3d.geometry.PointCloud()
    #     points = readFile("In/temp" + str(i) + ".xyz")
    #     names.append("In/temp" + str(i) + ".xyz")
    #     points = remove_platform(points)
    #     currentCloud.points.extend(points)
    #     voxelSize = round(max(currentCloud.get_max_bound()-currentCloud.get_min_bound())*0.01,4)
    #     currentCloud = remove_outliers(pcd=currentCloud, voxelSize=voxelSize, iterations=3, numberOfPoints=55, radius=0.004)
    #     cloudsToCombine.append(currentCloud)

    # voxelSize = round(max(cloudsToCombine[0].get_max_bound()-cloudsToCombine[0].get_min_bound())*0.001, 4)

    bun0 = o3d.io.read_point_cloud("PLY/Stanford Bunny/bun000" + ".ply")
    bun45 = o3d.io.read_point_cloud("PLY/Stanford Bunny/bun045" + ".ply")
    bun90 = o3d.io.read_point_cloud("PLY/Stanford Bunny/bun090" + ".ply")
    bun180 = o3d.io.read_point_cloud("PLY/Stanford Bunny/bun180" + ".ply")
    bun270 = o3d.io.read_point_cloud("PLY/Stanford Bunny/bun270" + ".ply")
    bun315 = o3d.io.read_point_cloud("PLY/Stanford Bunny/bun315" + ".ply")
    chin = o3d.io.read_point_cloud("PLY/Stanford Bunny/chin" + ".ply")
    ear_back = o3d.io.read_point_cloud("PLY/Stanford Bunny/ear_back" + ".ply")
    top2 = o3d.io.read_point_cloud("PLY/Stanford Bunny/top2" + ".ply")
    top3 = o3d.io.read_point_cloud("PLY/Stanford Bunny/top3" + ".ply")
    voxelSize=round(max(bun0.get_max_bound()-bun0.get_min_bound())*0.01,4)
    print(voxelSize)
    stop = input("stop here if you want traveller")

    names = [
        "PLY/Stanford Bunny/bun000" + ".ply",
        "PLY/Stanford Bunny/bun045" + ".ply",
        "PLY/Stanford Bunny/bun090" + ".ply",
        "PLY/Stanford Bunny/bun315" + ".ply",
        "PLY/Stanford Bunny/chin" + ".ply",
        "PLY/Stanford Bunny/top2" + ".ply",
        "PLY/Stanford Bunny/top3" + ".ply",
        "PLY/Stanford Bunny/bun270" + ".ply",
        "PLY/Stanford Bunny/bun180" + ".ply",
        "PLY/Stanford Bunny/ear_back" + ".ply"
    ]

    # cloud0 = o3d.io.read_point_cloud("Color/cloud_bin_00.pcd")
    # cloud1 = o3d.io.read_point_cloud("Color/cloud_bin_01.pcd")
    # cloud2 = o3d.io.read_point_cloud("Color/cloud_bin_02.pcd")
    # voxelSize=round(max(cloud0.get_max_bound()-cloud0.get_min_bound())*0.01,4)

    # cloudsToCombine = [cloud0, cloud1, cloud2]
    # names = ["cloud__bin__00.pcd", "cloud__bin__01.pcd", "cloud__bin__02.pcd"]

    # cloudsToCombine = [bun0, bun45, bun90, bun315, chin,  top2, top3, bun270, bun180, ear_back]
    # cloudsToCombine = [bun0, bun45, bun90, bun315, chin,  top2, top3, bun180, ear_back]
    # cloudsToCombine = [bun0, bun45, bun90, bun315, chin,  top2, top3, bun270, ear_back]
    cloudsToCombine = [bun0, bun45, bun90, bun315, chin,  top2, top3, bun270, ear_back, bun180]

    cloudsAndSamples = []
    
    i = 0
    for cloud in cloudsToCombine:
        cloudsAndSamples.append((cloud, cloud.voxel_down_sample(voxelSize), names[i]))
        # downSample = cloud.voxel_down_sample(voxelSize)
        # display_point_cloud(cloud)
        # display_point_cloud(downSample)
        # cloudsAndSamples.append((cloud, cloud, names[i]))

        i += 1

    # totalCloud = pointToPointMerge(cloudsAndSamples, voxelSize)
    # totalCloud = colorPointToPoint(cloudsAndSamples, voxelSize)
    totalCloud = pointToPlaneMerge(cloudsAndSamples, voxelSize)
    display_point_cloud(totalCloud)
    # totalCloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))
    totalCloud.estimate_normals()
    totalCloud.orient_normals_consistent_tangent_plane(100)
    # totalCloud.orient_normals_towards_camera_location(totalCloud.get_center())
    # totalCloud.normals = o3d.utility.Vector3dVector( - np.asarray(totalCloud.normals))
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(totalCloud, depth=15, width=0, scale=1.1, linear_fit=False)[0]
    o3d.visualization.draw_geometries([mesh]) 
    mesh.compute_vertex_normals()
    stop = input("stop here for no file")
    o3d.io.write_triangle_mesh("test2.stl", mesh)

    # totalCloud = totalCloud.voxel_down_sample(voxelSize)
    # display_point_cloud(totalCloud)
    # totalCloud.estimate_normals()
    # totalCloud.orient_normals_consistent_tangent_plane(100)
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(totalCloud, depth=8, width=0, scale=1.1, linear_fit=False)[0]

    # totalCloud = pointToPlaneMerge(cloudsAndSamples, voxelSize)

    # totalCloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxelSize*2, max_nn=30))
    # totalCloud.orient_normals_towards_camera_location(totalCloud.get_center())
    # totalCloud.normals = o3d.utility.Vector3dVector( - np.asarray(totalCloud.normals))
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(totalCloud, depth=12, width=0, scale=1.1, linear_fit=False)[0]
    
    # display_point_cloud(totalCloud)
    # o3d.visualization.draw_geometries([mesh]) 
    # o3d.io.write_triangle_mesh("test1.stl", mesh)

    # display_point_cloud(totalCloud)


if __name__ == '__main__':
    main()