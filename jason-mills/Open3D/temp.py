import open3d as o3d

fileData0 = o3d.io.read_point_cloud("Color/cloud_bin_00.pcd")
fileData1 = o3d.io.read_point_cloud("Color/cloud_bin_01.pcd")
fileData2 = o3d.io.read_point_cloud("Color/cloud_bin_02.pcd")

o3d.visualization.draw_geometries([fileData0, fileData1, fileData2])