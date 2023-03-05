import open3d as o3d
import numpy as np

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


def main():
    myReader = XYZReader()
    pcd = o3d.geometry.PointCloud()

    points = np.asarray(myReader.readFile('XYZ/megamind.xyz'))
    pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/Y Rotation/test2.xyz'))
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/Y Rotation/test3.xyz')) 
    # pcd.points.extend(points)
    # points = np.asarray(myReader.readFile('XYZ/Y Rotation/test4.xyz'))
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
    o3d.visualization.draw_geometries([pcd])

    # o3d.io.write_triangle_mesh("test.off", mesh)

    # mesh = o3d.io.read_triangle_mesh('test.off')
    # o3d.visualization.draw_geometries([mesh])

    

if __name__ == '__main__':
    main()