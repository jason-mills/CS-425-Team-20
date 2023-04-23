import open3d as o3d
import PointCloudStruct
import copy

class Editor():
    def __init__(self, cloud_structs):
        self.visualizer = o3d.visualization.VisualizerWithKeyCallback()
        self.visualizer.create_window()
        # self.visualizer.register_key_callback(ord('n'), self.point_to_plane_merge)
        self.visualizer.register_key_callback(ord("Y"), self.test)

        self.cloud_structs = cloud_structs

        self.total_cloud = o3d.geometry.PointCloud()
        self.down_sample_total = o3d.geometry.PointCloud()

        self.mesh = o3d.geometry.TriangleMesh()

        return

    def test(self, another):
        print(self, another)

        return

    def update_visualizer(self, cloud):
        self.visualizer.clear_geometries()
        self.visualizer.add_geometry(cloud)
        self.visualizer.update_renderer()

        self.visualizer.run()

        self.visualizer.destroy_window()

        return

    def run_visualizer(self):
        self.visualizer.run()

        return

    def destroy_visualizer(self):
        self.visualizer.destroy_window()

        return

    def add_base_points(self):
        self.total_cloud = self.cloud_structs[0].cloud
        self.down_sample_total = self.cloud_structs[0].downsampled_cloud

        # get a general transformation matrix to apply to align points
    def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5
        print("Global Registration Started")
        registration_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(source_down,
                                                                                            target_down,
                                                                                            source_fpfh, 
                                                                                            target_fpfh, 
                                                                                            True, 
                                                                                            distance_threshold, 
                                                                                            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 
                                                                                            3, 
                                                                                            [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                                                                                            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)], 
                                                                                            o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        
        transformed_cloud = copy.deepcopy(source_down)
        transformed_cloud.transform(registration_result.transformation)
        target_copy = copy.deepcopy(target_down)
        print("updating cloud")

        return registration_result

    def execute_point_to_point_refinement(source, target, voxel_size, initial_transformation):
        cloud_to_transform = o3d.geometry.PointCloud()
        combined_cloud = o3d.geometry.PointCloud()

        print("Local Point to Point Refinement Started")
        registration_result = o3d.pipelines.registration.registration_icp(source, 
                                                                target, 
                                                                voxel_size * 1.5,
                                                                initial_transformation,
                                                                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
        
        cloud_to_transform = copy.deepcopy(source)
        combined_cloud = cloud_to_transform.transform(registration_result.transformation) + target

        return combined_cloud, registration_result.fitness
        

    def execute_point_to_plane_refinement(source, target, voxel_size, initial_transformation):
        source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2,
                                                                    max_nn=30))
        
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2,
                                                                    max_nn=30))

        print("Local Point to Plane Refinement Started")
        registration_result = o3d.pipelines.registration.registration_icp(source,
                                                                        target,
                                                                            voxel_size * 1.5,
                                                                            initial_transformation,
                                                                            o3d.pipelines.registration.TransformationEstimationPointToPlane())

        transformed_cloud = copy.deepcopy(source)
        transformed_cloud.transform(registration_result.transformation)

        return registration_result.transformation

    def display_registration_result(self, source_cloud, target_cloud, transformation):
        source_cloud_copy = copy.deepcopy(source_cloud)
        target_cloud_copy = copy.deepcopy(target_cloud)

        source_cloud_copy.paint_uniform_color([0.54, 0, 0])
        target_cloud_copy.paint_uniform_color([0.5, 0.5, 0.5])

        combined_cloud = source_cloud_copy + target_cloud_copy

        self.update_visualizer(combined_cloud)

        return

    # def point_to_plane_merge(self):
    #         print("Adding: " + self.cloud_structs[0].name)

    #         source_down, sourceFpfh = preprocess_point_cloud(self.cloud_structs[0].downsampled_cloud, 
    #                                                          voxel_size=self.cloud_structs[0].voxel_size)
            
    #         target_down, targetFpfh = preprocess_point_cloud(self.down_sample_total, 
    #                                                          voxel_size=self.cloud_structs[0].voxel_size)
            
            
    #         ransacResult = execute_global_registration(source_down, 
    #                                                    target_down, 
    #                                                    sourceFpfh, 
    #                                                    targetFpfh, 
    #                                                    self.calculate_voxle_size(self.down_sample_total))

    #         self.cloud_structs[0].cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.cloud_structs * 2,
    #                                                                                            max_nn=30))
    #         self.total_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.cloud_structs[0].voxel_size * 2,
    #                                                                                 max_nn=30))

    #         transformation = execute_point_to_plane_refinement(self.cloud_structs[0].cloud, 
    #                                                            self.total_cloud, 
    #                                                            self.cloud_structs[0].voxel_size * 1.5, 
    #                                                            ransacResult.transformation)

    #         self.display_registration_result(self.cloud_structs[0].cloud, self.total_cloud, transformation)

    #         return

    def make_mesh(self, method, options):
        if method == "poisson":
            self.make_poisson_mesh(options)
        else:
            raise TypeError("Attempt to use unsupported meshing type")
        return

    def make_poisson_mesh(self, options):
        self.total_cloud.estimate_normals()
        self.total_cloud.orient_normals_consistent_tangent_plane(100)
        self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.total_cloud, depth=12, width=0, scale=1.1, linear_fit=False)[0]

        return

    def color_cloud(self, color):
        self.total_cloud.paint_uniform_color([color[0], color[1], color[2]])

        return

    def color_mesh(self, color):
        self.mesh.paint_uniform_color([color[0], color[1], color[2]])

        return

    def calculate_voxle_size(cloud):
        return round(max(cloud.get_max_bound() - cloud.get_min_bound()) * 0.01, 4)

    def get_cloud_struct_len(self):
        return len(self.cloud_structs)

    def poll(self):
        return self.visualizer.poll_events()

    def write_point_cloud_file(self, file_path, file_type):
        o3d.io.write_point_cloud(file_path + file_type, self.total_cloud)

        return

    def write_mesh_file(self, file_path, file_type):
        o3d.io.write_triangle_mesh(file_path + file_type, self.mesh)

        return

    def __del__ (self):
        self.visualizer.destroy_window()

        return
    
