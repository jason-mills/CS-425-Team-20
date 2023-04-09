import open3d as o3d
import numpy as np

class XYZReader:
    def readFile(self, filePath):
        points = []
        fileStream = open(filePath, 'r')

        firstLine = fileStream.readline()
        if(firstLine == 'X Y Z\n'):
            ytotal = 0
            for line in fileStream:
                line = line.rstrip('\n')
                x, y, z = line.split(' ')
                if x == '-0' or y == '-0' or z == '-0':
                    continue
                # 0.047155059153702286
                if(float(y) > 0.044):
                    continue
                points.append([x, y, z])
                # if(i > 10):
                #     break

                # i += 1

        return points


def main():
    myReader = XYZReader()
    pcd1 = o3d.geometry.PointCloud()
    pcd2 = o3d.geometry.PointCloud()
    
    # points = np.asarray(myReader.readFile('data0.xyz'))
    # pcd.points.extend(points)
    # ply_point_cloud = o3d.data.PLYPointCloud()
    # pcd = o3d.io.read_point_cloud("final.ply")
    points = np.asarray(myReader.readFile('XYZ/temp0.xyz'))
    pcd1.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp1.xyz')) 
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp2.xyz'))
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp3.xyz'))
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp4.xyz')) 
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp5.xyz'))
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp6.xyz'))
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp7.xyz')) 
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp8.xyz'))
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp9.xyz'))
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp10.xyz')) 
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/temp11.xyz'))
    # pcd.points.extend(points)
    

    
    # filePath = 'PLY Files/Stanford Bunny/'
    # pcd.points.extend(np.asarray(points.points))

    # points = o3d.io.read_point_cloud(filePath + 'bun045.ply', format='ply')
    # pcd.points.extend(np.asarray(points.points))
    # points = o3d.io.read_point_cloud(filePath + 'bun090.ply', format='ply')
    # pcd.points.extend(np.asarray(points.points))
    # points = o3d.io.read_point_cloud(filePath + 'bun180.ply', format='ply')
    # pcd.points.extend(np.asarray(points.points))
    # points = o3d.io.read_point_cloud(filePath + 'bun270.ply', format='ply')
    # pcd.points.extend(np.asarray(points.points))
    # points = o3d.io.read_point_cloud(filePath + 'bun315.ply', format='ply')
    # pcd.points.extend(np.asarray(points.points))

    # pcd = o3d.io.read_point_cloud('something.ply')

    
    # pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # distances = pcd.compute_nearest_neighbor_distance()
    # avg_dist = np.mean(distances)
    # radius  = 3 * avg_dist

    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector([radius, radius * 2]))

    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth = 10, width = 0, scale = 1.1, linear_fit=False)[0]

    # dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)

    # mesh.remove_degenerate_triangles()
    # mesh.remove_duplicated_triangles()
    # mesh.remove_duplicated_vertices()
    # mesh.remove_non_manifold_edges()

    # o3d.visualization.draw_geometries([mesh])

    # o3d.io.write_point_cloud("")
    o3d.visualization.draw_geometries([pcd1, pcd2])
    # o3d.visualization.draw_geometries([pcd1])


    # o3d.io.write_triangle_mesh("test.off", mesh)

    # mesh = o3d.io.read_triangle_mesh('test.off')
    # o3d.visualization.draw_geometries([mesh])

    

if __name__ == '__main__':
    main()