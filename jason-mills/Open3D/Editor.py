import open3d as o3d
import numpy as np
import PointCloudStruct
import copy

class Editor():
    def __init__(self, cloud_structs):
        # Initialize window with callback
        self.visualizer = o3d.visualization.VisualizerWithKeyCallback()
        self.visualizer.create_window()
        
        # self.visualizer.register_key_callback(334, self.point_to_point_merge) # keypad add
        self.visualizer.register_key_callback(334, self.point_to_plane_merge) # keypad add
        self.visualizer.register_key_callback(77, self.make_poisson_mesh) # M key
        

        # Give object cloud structs and cloud index for state management
        self.cloud_structs = cloud_structs
        self.backup_structs = cloud_structs
        self.current_cloud_index = 0

        # Create a total cloud and total down sample cloud for point cloud merging
        self.total_cloud = o3d.geometry.PointCloud()
        self.down_sample_total = o3d.geometry.PointCloud()

        # Give object a mesh object to work with
        self.mesh = o3d.geometry.TriangleMesh()

        # Last transformation that was generated by ICP, start with identity matrix for adding base points
        self.transformation = np.asarray([[1, 0, 0, 0],
                                          [0, 1, 0, 0],
                                          [0, 0, 1, 0],
                                          [0, 0, 0, 1]])

        return

    def initialize_default_callbacks(self):
        # Register key callback functions, uses glfw key number bindings
        # https://www.glfw.org/docs/latest/group__keys.html
        self.visualizer.register_key_callback(257, self.add_transformed_cloud) # enter key
        self.visualizer.register_key_callback(335, self.add_transformed_cloud) # keypad enter
        self.visualizer.register_key_callback(83, self.send_cloud_to_back) # S key
        self.visualizer.register_key_callback(262, self.view_next_cloud) # right arrow
        self.visualizer.register_key_callback(263, self.view_previous_cloud) # left arrow
        self.visualizer.register_key_callback(82, self.revert_changes) # R key

        return

    def set_point_to_plane_callbacks(self, context):
        self.visualizer.register_key_callback(334, self.point_to_plane_merge) # keypad add

        return

    def revert_changes(self, context):
        self.cloud_structs = self.backup_structs
        self.current_cloud_index = 0

        self.total_cloud.clear()
        self.down_sample_total.clear()
        self.mesh.clear()

        self.transformation = np.asarray([[1, 0, 0, 0],
                                          [0, 1, 0, 0],
                                          [0, 0, 1, 0],
                                          [0, 0, 0, 1]])
        
        self.update_visualizer()

        return

    def view_next_cloud(self, context):
        if self.current_cloud_index >= 0 and self.current_cloud_index < len(self.cloud_structs) - 1:
            self.current_cloud_index += 1

        elif len(self.cloud_structs) == 0:
            return

        elif self.current_cloud_index > len(self.cloud_structs) - 1:
            self.current_cloud = len(self.cloud_structs) - 1

        self.update_visualizer(self.cloud_structs[self.current_cloud_index].cloud)

        return

    def view_previous_cloud(self, context):
        if self.current_cloud_index > 0 and len(self.cloud_structs) > 0:
            self.current_cloud_index -= 1

        elif len(self.cloud_structs) == 0:
            return
        
        self.update_visualizer(self.cloud_structs[self.current_cloud_index].cloud)

        return

    def display_registration_result(self, source_cloud, target_cloud, transformation):
        source_cloud_copy = copy.deepcopy(source_cloud)
        target_cloud_copy = copy.deepcopy(target_cloud)

        source_cloud_copy.transform(transformation)
        source_cloud_copy.paint_uniform_color([0.54, 0, 0])
        target_cloud_copy.paint_uniform_color([0.5, 0.5, 0.5])

        combined_cloud = source_cloud_copy + target_cloud_copy

        self.update_visualizer(combined_cloud)

        return

    # display outliers given cloud and indices of inliers
    def display_inlier_outlier(self, cloud, indices):
        cloud_copy = copy.deepcopy(cloud)

        inlier_cloud = cloud_copy.select_by_index(indices)
        outlier_cloud = cloud_copy.select_by_index(indices, invert=True)

        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        
        combined_cloud = inlier_cloud + outlier_cloud

        self.update_visualizer(combined_cloud)

    def update_visualizer(self, geometry):
        self.visualizer.clear_geometries()
        self.visualizer.add_geometry(geometry)
        self.visualizer.update_renderer()

        return

    def run_visualizer(self):
        self.visualizer.run()

        return


    def run_visualizer(self):
        self.visualizer.run()

        return

    def destroy_visualizer(self):
        self.visualizer.destroy_window()

        return

    def add_transformed_cloud(self, context):
        print("Adding: " + self.cloud_structs[0].name)
        if not len(self.cloud_structs) > 0:
            return
        
        print(self.transformation)
        self.total_cloud += self.cloud_structs[0].cloud.transform(self.transformation)
        self.down_sample_total += self.cloud_structs[0].downsampled_cloud.transform(self.transformation)

        self.cloud_structs.pop(0)

        self.update_visualizer(self.total_cloud)

        return

    def send_cloud_to_back(self, context):
        if not len(self.cloud_structs) > 0:
            return
        
        self.cloud_structs.append(self.cloud_structs[0])
        self.cloud_structs.pop(0)

        return

    # estimate normals and compute 33 dimensional FPHP vector
    def preprocess_point_cloud(self, point_cloud, voxel_size):
        radius_normal = voxel_size * 2
        point_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(point_cloud, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

        return point_cloud, pcd_fpfh

        # get a general transformation matrix to apply to align points
    def execute_global_registration(self, source_down, target_down, source_fpfh, target_fpfh, voxel_size):
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
        
        return registration_result

    def execute_point_to_point_refinement(self, source, target, voxel_size, initial_transformation):
        print("Local Point to Point Refinement Started")
        registration_result = o3d.pipelines.registration.registration_icp(source, 
                                                                target, 
                                                                voxel_size * 1.5,
                                                                initial_transformation,
                                                                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
        

        return registration_result.transformation

    def execute_point_to_plane_refinement(self, source, target, voxel_size, initial_transformation):
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

        return registration_result.transformation

    def execute_color_point_to_point_refinement(source, target, voxel_size, initial_transformation):
        print("Local Color Point to Point Refinement Started")
        registration_result = o3d.pipelines.registration.registration_colored_icp(source,
                                                                    target,
                                                                        voxel_size*0.05,
                                                                        initial_transformation,
                                                                        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                                                                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))

        return registration_result.transformation

    def point_to_plane_merge(self, context):
            if len(self.total_cloud.points) == 0:
                self.add_transformed_cloud(context)

            if not len(self.cloud_structs) > 0:
                return

            source_down, sourceFpfh = self.preprocess_point_cloud(point_cloud=self.cloud_structs[0].downsampled_cloud, 
                                                                  voxel_size=self.cloud_structs[0].voxel_size)
            
            target_down, targetFpfh = self.preprocess_point_cloud(point_cloud=self.down_sample_total, 
                                                                  voxel_size=self.cloud_structs[0].voxel_size)
            
            
            ransac_result = self.execute_global_registration(source_down, 
                                                       target_down, 
                                                       sourceFpfh, 
                                                       targetFpfh, 
                                                       self.calculate_voxel_size(self.down_sample_total))

            self.cloud_structs[0].cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.cloud_structs[0].voxel_size * 2,
                                                                                               max_nn=30))
            self.total_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.cloud_structs[0].voxel_size * 2,
                                                                                    max_nn=30))

            transformation = self.execute_point_to_plane_refinement(self.cloud_structs[0].cloud, 
                                                               self.total_cloud, 
                                                               self.cloud_structs[0].voxel_size * 1.5, 
                                                               ransac_result.transformation)

            self.display_registration_result(self.cloud_structs[0].cloud, self.total_cloud, transformation)

            self.transformation = transformation

            return

    def color_point_to_plane_merge(self, context):
            if len(self.total_cloud.points) == 0:
                self.add_transformed_cloud(context)

            if not len(self.cloud_structs) > 0:
                return

            source_down, sourceFpfh = self.preprocess_point_cloud(point_cloud=self.cloud_structs[0].downsampled_cloud, 
                                                                  voxel_size=self.cloud_structs[0].voxel_size)
            
            target_down, targetFpfh = self.preprocess_point_cloud(point_cloud=self.down_sample_total, 
                                                                  voxel_size=self.cloud_structs[0].voxel_size)
            
            
            ransac_result = self.execute_global_registration(source_down, 
                                                       target_down, 
                                                       sourceFpfh, 
                                                       targetFpfh, 
                                                       self.calculate_voxel_size(self.down_sample_total))

            self.cloud_structs[0].cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.cloud_structs[0].voxel_size * 2,
                                                                                               max_nn=30))
            self.total_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.cloud_structs[0].voxel_size * 2,
                                                                                    max_nn=30))
            
            transformation = self.execute_color_point_to_point_refinement(self.cloud_structs[0].cloud, 
                                                                          self.total_cloud,
                                                                          self.cloud_structs[0].voxel_size * 1.5, 
                                                                          ransac_result.transformation)

            self.display_registration_result(self.cloud_structs[0].cloud, self.total_cloud, transformation)

            self.transformation = transformation

            return
    
    def point_to_point_merge(self, context):
            if len(self.total_cloud.points) == 0:
                self.add_transformed_cloud(context)

            if not len(self.cloud_structs) > 0:
                return

            source_down, sourceFpfh = self.preprocess_point_cloud(point_cloud=self.cloud_structs[0].downsampled_cloud, 
                                                                  voxel_size=self.cloud_structs[0].voxel_size)
            
            target_down, targetFpfh = self.preprocess_point_cloud(point_cloud=self.down_sample_total, 
                                                                  voxel_size=self.cloud_structs[0].voxel_size)
            
            
            ransac_result = self.execute_global_registration(source_down, 
                                                       target_down, 
                                                       sourceFpfh, 
                                                       targetFpfh, 
                                                       self.calculate_voxel_size(self.down_sample_total))

            self.cloud_structs[0].cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.cloud_structs[0].voxel_size * 2,
                                                                                               max_nn=30))
            self.total_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.cloud_structs[0].voxel_size * 2,
                                                                                    max_nn=30))
            
            transformation = self.execute_point_to_point_refinement(self.cloud_structs[0].cloud, 
                                                                    self.total_cloud,
                                                                    self.cloud_structs[0].voxel_size * 1.5, 
                                                                    ransac_result.transformation)

            self.display_registration_result(self.cloud_structs[0].cloud, self.total_cloud, transformation)

            self.transformation = transformation

            return

    def make_mesh(self, method, options):
        if method == "poisson":
            self.make_poisson_mesh(options)
        else:
            raise TypeError("Attempt to use unsupported meshing type")
        return

    def make_poisson_mesh(self, options):
        if len(self.total_cloud.points) == 0:
            return

        self.total_cloud.estimate_normals()
        self.total_cloud.orient_normals_consistent_tangent_plane(100)
        self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.total_cloud, depth=12, width=0, scale=1.1, linear_fit=False)[0]

        self.color_mesh([0.5, 0.5, 0.5])
        self.mesh.compute_vertex_normals()

        self.update_visualizer(self.mesh)

        return

    def color_cloud(self, color):
        self.total_cloud.paint_uniform_color([color[0], color[1], color[2]])

        return

    def color_mesh(self, color):
        self.mesh.paint_uniform_color([color[0], color[1], color[2]])

        return

    def calculate_voxel_size(self, cloud):
        return round(max(cloud.get_max_bound() - cloud.get_min_bound()) * 0.01, 4)

    def get_cloud_struct_len(self):
        return len(self.cloud_structs)

    def write_point_cloud_file(self, file_path, file_type):
        o3d.io.write_point_cloud(file_path + file_type, self.total_cloud)

        return

    def write_mesh_file(self, file_path, file_type):
        o3d.io.write_triangle_mesh(file_path + file_type, self.mesh)

        return

    def __del__ (self):
        self.visualizer.destroy_window()

        return