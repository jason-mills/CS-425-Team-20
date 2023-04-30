import open3d as o3d
import numpy as np
from Structs import PointCloudStruct
from Structs import MeshStruct
from Structs import CircularArray
import copy
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from win32api import GetSystemMetrics

class Editor():
    def __init__(self, cloud_structs):
        print(GetSystemMetrics(0), GetSystemMetrics(1))

        # Give object cloud structs and cloud index for state management
        self.cloud_structs = copy.deepcopy(cloud_structs)
        self.backup_structs = copy.deepcopy(cloud_structs)

        # Create a total cloud and total down sample cloud for point cloud merging
        self.total_cloud = o3d.geometry.PointCloud()
        self.down_sample_total = o3d.geometry.PointCloud()

        # Give object a mesh object to work with
        self.mesh = o3d.geometry.TriangleMesh()

        # Last transformation that was generated by ICP, start with identity matrix for adding base points
        self.transformation = np.identity(4)

        # Load in circular array that contains structs for meshes and clouds
        self.geometries = self.initialize_circular_array(cloud_structs)
        self.circular_array_index = 1

        # keep track of current geometry name and lable so that updating the visualizer is possible
        self.current_geometry_name = ""
        self.current_3d_label = None

        # Keep track of what mode is being used for editing
        self.current_mode = "multiway_registration"
        self.meshing_method = "poisson"

        # Initialize the gui
        self.initialize_gui(cloud_structs)

        # Give the gui something to load with
        self.update_visualizer(self.geometries[1].geometry, self.geometries[1].name)

        self.metadata = []

        return
    
    # Initialize a circular array for use in moving through geometries in the scene viewer
    def initialize_circular_array(self, cloud_structs):
        geometries = []

        geometries.append(PointCloudStruct("Total Cloud", None, None, None, None))

        for cloud_struct in cloud_structs:
            geometries.append(cloud_struct)

        geometries.append(MeshStruct("Mesh", None))

        circular_array = CircularArray(np.asarray(geometries))

        return circular_array

    # Initialize the gui
    def initialize_gui(self, cloud_structs):
        # Numbers that are useful to track for initilization
        self.initial_screen_width = int(GetSystemMetrics(0) * 0.75)
        self.initial_screen_heigth = int(GetSystemMetrics(1) * 0.75)

        # Create window adn declare function for resizing widgets
        self.window = gui.Application.instance.create_window("C3P0", self.initial_screen_width, self.initial_screen_heigth)
        self.window.set_on_layout(self.on_layout)
        self.em = self.window.theme.font_size

        # Call functions that add editor and scene widgets to the window
        self.initialize_editor()
        self.initialize_scene_widget(cloud_structs)

        return
    
    # Initialize the editor with desire buttons/functions
    def initialize_editor(self):
        self.layout = gui.Vert(0, gui.Margins(2 * self.em, 2 * self.em, 2 * self.em, 2 * self.em))
        self.layout.frame = gui.Rect(0, self.window.content_rect.y, self.initial_screen_width * 0.25, self.window.content_rect.height)
        self.layout.add_stretch()

        next_button = gui.Button("Next")
        next_button.set_on_clicked(self.view_next_cloud)
        
        previous_buttion = gui.Button("Previous")
        previous_buttion.set_on_clicked(self.view_previous_cloud)

        mesh_button = gui.Button("Make Mesh")
        mesh_button.set_on_clicked(self.make_mesh)

        self.progress_bar = gui.ProgressBar()
        self.progress_bar.visible = False

        self.layout.add_child(previous_buttion)
        self.layout.add_child(next_button)
        self.layout.add_child(self.progress_bar)

        self.window.add_child(self.layout)

        return

    # Initialize the scene widget with desired functionality and geometries
    def initialize_scene_widget(self, cloud_structs):
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.enable_scene_caching(True)
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.scene = self.scene_widget.scene
        self.scene_widget.set_on_key(self.key_callbacks)
        self.scene_widget.frame = gui.Rect(self.initial_screen_width * 0.25, self.window.content_rect.y, self.initial_screen_width, self.window.content_rect.height)

        self.add_all_geometry(cloud_structs)

        self.scene_widget.scene.set_background([0.5, 0.5, 0.5, 0.4])

        self.window.add_child(self.scene_widget)

        return

    def on_layout(self, context):
        self.layout.frame = gui.Rect(0, self.window.content_rect.y, self.window.content_rect.get_right() * 0.25, self.window.content_rect.height)
        self.scene_widget.frame = gui.Rect(self.window.content_rect.get_right() * 0.25, self.window.content_rect.y, self.window.content_rect.get_right(), self.window.content_rect.height)

        return
    
    def add_all_geometry(self, cloud_structs):
        self.material = rendering.MaterialRecord()

        self.scene.add_geometry("Merge Result", self.total_cloud, self.material)

        self.scene.add_geometry("Total Cloud", self.total_cloud, self.material)
        self.scene.show_geometry("Total Cloud", False)

        for cloud_struct in cloud_structs:
            self.scene.add_geometry(cloud_struct.name, cloud_struct.cloud, self.material)
            self.scene.show_geometry(cloud_struct.name, False)

        self.scene.add_geometry("Mesh", self.mesh, self.material)
        self.scene.show_geometry("Mesh", False)

        return

    def remove_all_geometry(self, cloud_structs):
        self.scene.remove_geometry("Merge Result")

        self.scene.remove_geometry("Total Cloud")

        for cloud_struct in cloud_structs:
            self.scene.remove_geometry(cloud_struct.name)

        self.scene.remove_geometry("Mesh")

        return

    def key_callbacks(self, context):
        if context.type == gui.KeyEvent.UP:
            if context.key == gui.KeyName.RIGHT: # right arrow key
                self.view_next_cloud()

            elif context.key == gui.KeyName.LEFT: #left arrow key
                self.view_previous_cloud()

            elif context.key == gui.KeyName.S:
                self.send_cloud_to_back()

            elif context.key in (260, 334): #insert and keypad plus
                if self.current_mode == "multiway_registration":
                    self.multiway_registration_merge()
                elif self.current_mode == "point to plane":
                    self.point_to_plane_merge()
                elif self.current_mode == "point to point":
                    self.point_to_point_merge()
                elif self.current_mode == "color point to point":
                    self.color_point_to_plane_merge()

            elif context.key in (257, 335): #enter and keypad enter
                self.add_transformed_cloud()

            elif context.key in (48, 320):
                self.current_mode = "multiway_registration"

            elif context.key in (49, 321):
                self.current_mode = "point to plane"

            elif context.key in (50, 322):
                self.current_mode = "point to point"

            elif context.key in (51, 323):
                self.current_mode = "color point to point"

            elif context.key == gui.KeyName.R:
                self.revert_changes()

        return gui.Widget.EventCallbackResult.IGNORED

    # clear changes to the cloud
    def revert_changes(self):
        self.cloud_structs = copy.deepcopy(self.backup_structs)
        self.current_cloud_index = 1

        self.total_cloud.clear()
        self.down_sample_total.clear()
        self.mesh.clear()

        if not self.current_3d_label == None:
            self.scene_widget.remove_3d_label(self.current_3d_label)

        if not self.current_geometry_name == "":
            self.scene.show_geometry(self.current_geometry_name, False)

        self.transformation = np.identity(4)
        
        self.current_geometry_name = ""
        self.current_3d_label = None

        self.remove_all_geometry(self.cloud_structs)
        self.add_all_geometry(self.cloud_structs)

        self.update_visualizer(self.geometries[1].geometry, self.geometries[1].name)

        return

    # increase the index and view the next cloud angle
    def view_next_cloud(self):
        if self.current_geometry_name == "Merge Result":
            self.update_visualizer(self.geometries[self.circular_array_index].geometry, self.geometries[self.circular_array_index].name)
        else:
            self.circular_array_index += 1
            self.update_visualizer(self.geometries[self.circular_array_index].geometry, self.geometries[self.circular_array_index].name)

        return

    # decrease the index and view the previous cloud angle
    def view_previous_cloud(self):
        if self.current_geometry_name == "Merge Result":
            self.update_visualizer(self.geometries[self.circular_array_index].geometry, self.geometries[self.circular_array_index].name)
        else:
            self.circular_array_index -= 1
            self.update_visualizer(self.geometries[self.circular_array_index].geometry, self.geometries[self.circular_array_index].name)

        return

    # cloud to be merged into is gray, cloud being merged in red
    def display_registration_result(self, source_cloud, target_cloud, transformation):
        source_cloud_copy = copy.deepcopy(source_cloud)
        target_cloud_copy = copy.deepcopy(target_cloud)

        source_cloud_copy.transform(transformation)
        source_cloud_copy.paint_uniform_color([0.54, 0, 0])
        target_cloud_copy.paint_uniform_color([0.5, 0.5, 0.5])

        combined_cloud = source_cloud_copy + target_cloud_copy

        self.scene.remove_geometry("Merge Result")
        self.scene.add_geometry("Merge Result", combined_cloud, self.material)
        self.update_visualizer(combined_cloud, "Merge Result")

        return

    # display outliers given cloud and indices of inliers
    def display_inlier_outlier(self, cloud, indices):
        cloud_copy = copy.deepcopy(cloud)

        inlier_cloud = cloud_copy.select_by_index(indices)
        outlier_cloud = cloud_copy.select_by_index(indices, invert=True)

        outlier_cloud.paint_uniform_color([1, 0, 0])
        inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
        
        combined_cloud = inlier_cloud + outlier_cloud

        self.update_visualizer(combined_cloud, "Remove Outlier Result")
    
    def update_visualizer(self, geometry, name):
        label_position = [0, 0, 0]

        if geometry == None:
            self.scene.camera.look_at([0, 0, 0], [1, 1, 1], [0, 0, 1])
        else:
            bounding_box = geometry.get_axis_aligned_bounding_box()
            bounding_box_corners = np.asarray(bounding_box.get_box_points())
            label_position = bounding_box_corners[5]
            self.scene_widget.setup_camera(60, bounding_box, bounding_box.get_center())

        if not self.current_3d_label == None:
            self.scene_widget.remove_3d_label(self.current_3d_label)

        if not self.current_geometry_name == "":
            self.scene.show_geometry(self.current_geometry_name, False)

        self.current_3d_label = self.scene_widget.add_3d_label(label_position, name)
        self.scene.show_geometry(name, True)
        self.current_geometry_name = name
        
        return

    # start the visualization
    def run_visualizer(self):
        self.visualizer.run()

        return

    # clean up by destroying visualization window
    def destroy_visualizer(self):
        self.visualizer.destroy_window()

        return

    # add a cloud with the last calculated transformation
    def add_transformed_cloud(self):
        if not self.get_cloud_struct_len() > 0:
            return
        
        print("Adding: " + self.cloud_structs[0].name)

        self.total_cloud += self.cloud_structs[0].cloud.transform(self.transformation)
        self.down_sample_total += self.cloud_structs[0].downsampled_cloud.transform(self.transformation)

        self.cloud_structs.pop(0)

        self.scene.remove_geometry("Total Cloud")
        self.material.shader = "DEPTH"
        self.scene.add_geometry("Total Cloud", self.total_cloud, self.material)
        self.update_visualizer(self.total_cloud, "Total Cloud")

        return

    # move the cloud being merged to the end of the cloud structs
    def send_cloud_to_back(self):
        if not self.get_cloud_struct_len() > 0:
            return
        
        self.cloud_structs.append(self.cloud_structs[0])
        self.cloud_structs.pop(0)

        return

    def multiway_registration_merge(self):
        if not self.get_cloud_struct_len() > 0:
            return
        
        pose_graph = self.full_registration()

        option = o3d.pipelines.registration.GlobalOptimizationOption(max_correspondence_distance = self.cloud_structs[0].voxel_size,
                                                                     edge_prune_threshold = 0.25,
                                                                     reference_node = 0)
        
        o3d.pipelines.registration.global_optimization(pose_graph,
                                                       o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                                                       o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                                                       option)

        for i in range(self.get_cloud_struct_len()):
            self.transformation = pose_graph.nodes[i].pose
            self.add_transformed_cloud()

        self.scene.remove_geometry("Total Cloud")
        self.scene.add_geometry("Total Cloud", self.total_cloud, self.material)
        self.update_visualizer(self.total_cloud, "Total Cloud")

        return

    def full_registration(self):
        pose_graph = o3d.pipelines.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))

        print("There are " + str(self.get_cloud_struct_len()) + " clouds")
        
        cloud_struct_length = self.get_cloud_struct_len()
        for i in range(cloud_struct_length):
            for j in range(i + 1, cloud_struct_length):
                transformation, information_matrix = self.pairwise_registration(self.cloud_structs[i],
                                                            self.cloud_structs[j],
                                                            self.cloud_structs[i].voxel_size)
                
                if j == i + 1:
                    odometry = np.dot(transformation, odometry)
                    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                    pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(i, 
                                                                                    j, 
                                                                                    transformation, 
                                                                                    information_matrix, 
                                                                                    uncertain = False))
                
                else:
                    pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(i,
                                                                                    j,
                                                                                    transformation,
                                                                                    information_matrix,
                                                                                    uncertain = True))                                               


        print("Pose graph has " + str(len(pose_graph.nodes)) + " nodes")

        return pose_graph
    
    # apply a coarse point to plane merge and then a fine point to plane merge based on that coarse merge transformation
    def pairwise_registration(self, source, target, voxel_size):
        print("Applying pairwise registration with point to plane icp to: " + source.name + " and " + target.name)

        source.downsampled_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn = 30))
        target.downsampled_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn = 30))
   
        coarse_transformation = self.execute_distance_point_to_plane_refinement(source.downsampled_cloud,
                                                                       target.downsampled_cloud,
                                                                       voxel_size * 15,
                                                                       np.identity(4))
        
        fine_transformation = self.execute_distance_point_to_plane_refinement(source.downsampled_cloud,
                                                                     target.downsampled_cloud,
                                                                     voxel_size * 1.5,
                                                                     coarse_transformation)
   
        

        information_matrix = o3d.pipelines.registration.get_information_matrix_from_point_clouds(source.downsampled_cloud,
                                                                                                 target.downsampled_cloud,
                                                                                                 voxel_size * 1.5,
                                                                                                 fine_transformation)

        return fine_transformation, information_matrix

    # calculate the transformation for a point to plane merge with one point cloud onto the total cloud and display visualization
    def point_to_plane_merge(self):
            if len(self.total_cloud.points) == 0:
                self.add_transformed_cloud()

            if not self.get_cloud_struct_len() > 0:
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
                                                                                              max_nn = 30))
            
            self.total_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.cloud_structs[0].voxel_size * 2,
                                                                                   max_nn =30 ))

            transformation = self.execute_point_to_plane_refinement(self.cloud_structs[0].cloud, 
                                                                    self.total_cloud, 
                                                                    self.cloud_structs[0].voxel_size * 1.5, 
                                                                    ransac_result.transformation)

            self.display_registration_result(self.cloud_structs[0].cloud, self.total_cloud, transformation)

            self.transformation = transformation

            return
    
    # calculate the transformation for a point to point merge with one point cloud onto the total cloud and display visualization
    def point_to_point_merge(self):
            if len(self.total_cloud.points) == 0:
                self.add_transformed_cloud()

            if not self.get_cloud_struct_len() > 0:
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
    
    # calculate the transformation for a color point to plane merge with one point cloud onto the total cloud and display visualization
    def color_point_to_plane_merge(self):
            if len(self.total_cloud.points) == 0:
                self.add_transformed_cloud()

            if not self.get_cloud_struct_len() > 0:
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
            
            transformation = self.execute_color_point_to_plane_refinement(self.cloud_structs[0].cloud, 
                                                                          self.total_cloud,
                                                                          self.cloud_structs[0].voxel_size * 1.5, 
                                                                          ransac_result.transformation)

            self.display_registration_result(self.cloud_structs[0].cloud, self.total_cloud, transformation)

            self.transformation = transformation

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

    # use point to point refinement
    def execute_point_to_point_refinement(self, source, target, voxel_size, initial_transformation):
        print("Local Point to Point Refinement Started")
        registration_result = o3d.pipelines.registration.registration_icp(source, 
                                                                target, 
                                                                voxel_size * 1.5,
                                                                initial_transformation,
                                                                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
        

        return registration_result.transformation

    # use point to plane refinement
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

    def execute_distance_point_to_plane_refinement(self, source, target, distance, initial_transformation):
        registration_result = o3d.pipelines.registration.registration_icp(source,
                                                                          target,
                                                                          distance,
                                                                          initial_transformation,
                                                                          o3d.pipelines.registration.TransformationEstimationPointToPlane())

        return registration_result.transformation

    # use color point to plane refinement
    def execute_color_point_to_plane_refinement(self, source, target, voxel_size, initial_transformation):
        print("Local Color Point to Point Refinement Started")
        registration_result = o3d.pipelines.registration.registration_colored_icp(source,
                                                                                  target,
                                                                                  voxel_size * 0.05,
                                                                                  initial_transformation,
                                                                                  o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                                                                                  o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))

        return registration_result.transformation

    # might need to get rid of this
    # use to make different meshes with different options
    def make_mesh(self):
        if self.meshing_method == "poisson":
            self.make_poisson_mesh()
        else:
            raise TypeError("Attempt to use unsupported meshing type")
        return

    # make a mesh using poisson meshing
    def make_poisson_mesh(self):
        if len(self.total_cloud.points) == 0:
            return

        self.total_cloud.estimate_normals()
        self.total_cloud.orient_normals_consistent_tangent_plane(100)
        self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.total_cloud, depth=12, width=0, scale=1.1, linear_fit=False)[0]

        self.color_mesh([0.5, 0.5, 0.5])
        self.mesh.compute_vertex_normals()
        
        self.scene.remove_geometry("Mesh")
        self.scene.add_geometry("Mesh", self.mesh, self.material)
        self.update_visualizer(self.mesh, "Mesh")

        return

    def smooth_mesh(self):
        self.mesh = self.mesh.filter_smooth_simple(number_of_iterations=1)

        self.scene.remove_geometry("Mesh")
        self.scene.add_geometry("Mesh", self.mesh, self.material)
        self.update_visualizer(self.mesh, "Mesh")

        return

    # color the total point cloud a specific color
    def color_cloud(self, color):
        self.total_cloud.paint_uniform_color([color[0], color[1], color[2]])

        return

    # color the mesh a specific color
    def color_mesh(self, color):
        self.mesh.paint_uniform_color([color[0], color[1], color[2]])

        return

    # calculate a voxel size for a point cloud
    # this calculated voxel size can be used for icp
    def calculate_voxel_size(self, cloud):
        return round(max(cloud.get_max_bound() - cloud.get_min_bound()) * 0.01, 4)

    # get length of cloud struct
    # this is me being lazy because I don't want to tyle len(self.cloud_structs) - oh no I just did it
    def get_cloud_struct_len(self):
        return len(self.cloud_structs)

    # write a point cloud file with a specific file extension
    def write_point_cloud_file(self, file_path, file_type):
        o3d.io.write_point_cloud(file_path + file_type, self.total_cloud)

        return

    # write a mesh file 
    def write_mesh_file(self, file_path, file_type):
        o3d.io.write_triangle_mesh(file_path + file_type, self.mesh)

        return