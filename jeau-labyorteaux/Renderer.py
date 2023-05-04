import sys
import os
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import open3d.visualization as vis
import tkinter
from tkinter import filedialog
from win32api import GetSystemMetrics
import threading

window_X = int(GetSystemMetrics(0) * 0.75)
window_Y = int(GetSystemMetrics(1) * 0.75)

file_path = None
directory = None
file_paths = None

class Renderer:
    def __init__(self, filename):

        self.old_window = None
        self.material = None
        self.em = None
        self.scene_widget = None
        self.parent_widget_width = None
        self.parent_widget_height = None
        self.parent_widget = None
        self.window = None

        self.directory_walkthrough = []
        self.stored_geometries = []
        self.directory_index = 0
        self.render(filename)

        self.cached_filename = None

    def render(self, filename):
        gui.Application.instance.initialize()
        self.window = gui.Application.instance.create_window("Render", window_X, window_Y)

        # Create the parent widget
        self.parent_widget = gui.Widget()
        self.parent_widget_height = self.window.content_rect.height
        self.parent_widget_width = self.window.content_rect.width * .25

        # Create the scene widget
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.scene_widget.enable_scene_caching(True)

        # Material Settings
        self.material = rendering.MaterialRecord()
        self.material.shader = "defaultLit"

        # Setting for size of scene widget
        self.scene_widget.frame = gui.Rect(window_X*.25, 0, window_X*.75, window_Y)

        # Settings for size of parent widget
        self.parent_widget.frame = gui.Rect(0, 0, window_X*.25, window_Y)

        # Loading the file into the widget scene
        cloud = self.load_geometry(filename)
        self.scene_widget.scene.add_geometry(filename, cloud, self.material)

        # Setting background color
        self.scene_widget.scene.set_background([0.5, 0.5, 0.5, 0.4])
        bounding_box = cloud.get_axis_aligned_bounding_box()
        self.scene_widget.setup_camera(30, bounding_box, [0, 0, 0])

        # Create a button widget
        change_file = gui.Button("Change File")
        change_file.frame = gui.Rect(5, 100, self.parent_widget_width-10, 50)
        change_file.set_on_clicked(lambda: self.change_file(filename))

        # Getting specific file name
        split_up = filename.split("/")
        file = split_up[-1]

        # Create label to display current file
        current_file_label = gui.Label("Current File: " + file)
        current_file_label.frame = gui.Rect(25, 50, self.parent_widget_width - 50, 50)

        # Create button widget for directory walkthrough
        directory_button = gui.Button("Directory\nWalkthrough")
        directory_button.frame = gui.Rect(5, 200, self.parent_widget_width-10, 50)
        directory_button.set_on_clicked(lambda: self.walkthrough(filename))

        # Create button widget for comparison mode
        comparison_button = gui.Button("Comparison")
        comparison_button.frame = gui.Rect(5, 300, self.parent_widget_width-10, 50)
        comparison_button.set_on_clicked(lambda: self.initiate_comarison(filename))

        # Create a button widget
        exit_button = gui.Button("Exit")
        exit_button.frame = gui.Rect(5, 400, self.parent_widget_width-10, 50)
        exit_button.set_on_clicked(lambda: self.exit_button())

        # Add the button widgets to the parent widget
        self.parent_widget.add_child(comparison_button)
        self.parent_widget.add_child(change_file)
        self.parent_widget.add_child(exit_button)
        self.parent_widget.add_child(directory_button)

        # Adding parent widget, scene and label to main window
        self.window.add_child(self.parent_widget)
        self.window.add_child(self.scene_widget)
        self.window.add_child(current_file_label)

        gui.Application.instance.run()

        return

    def compare_mode(self, file_1, file_2):
        gui.Application.instance.initialize()
        # Create the window
        window = gui.Application.instance.create_window("Comparison", window_X, window_Y)

        half_window_width = window.content_rect.width/2
        window_height = window.content_rect.height

        # Set up widget for the top info
        parent_widget = gui.Widget()
        parent_widget_height = 100
        parent_widget_width = window.content_rect.width
        parent_widget.frame = gui.Rect(0, 0, parent_widget_width, parent_widget_height)

        # Setting up the scene widget
        scene_widget_left = gui.SceneWidget()
        scene_widget_left.scene = rendering.Open3DScene(window.renderer)
        scene_widget_left.enable_scene_caching(True)

        scene_widget_right = gui.SceneWidget()
        scene_widget_right.scene = rendering.Open3DScene(window.renderer)
        scene_widget_right.enable_scene_caching(True)

        # Settting specifics for the scene frame size parameters
        scene_widget_left.frame = gui.Rect(0, 100, half_window_width-5, window_height)
        scene_widget_right.frame = gui.Rect(half_window_width, 100, half_window_width + 5, window_height)

        # Getting specific file names
        split_up = file_1.split("/")
        file_1_name = split_up[-1]

        split_up = file_2.split("/")
        file_2_name = split_up[-1]

        # Create label to display file 1 name
        file_1_label = gui.Label("File: " + file_1_name)
        file_1_label.frame = gui.Rect(half_window_width/3, 25, 100, 50)

        # Create label to display file 2 name
        file_2_label = gui.Label("File: " + file_2_name)
        file_2_label.frame = gui.Rect((half_window_width+half_window_width/3)+10, 25, 100, 50)

        # Create button widget for directory walkthrough
        exit_button = gui.Button("Exit")
        exit_button.frame = gui.Rect(half_window_width-50, 25, 100, 50)
        exit_button.set_on_clicked(lambda: self.exit_comparison(window))

        # Material Settings
        material = rendering.MaterialRecord()
        material.shader = "defaultLit"

        cloud = self.load_geometry(file_1)
        scene_widget_left.scene.add_geometry(file_1, cloud, material)

        cloud = self.load_geometry(file_2)
        scene_widget_right.scene.add_geometry(file_2, cloud, material)

        # Setting background color
        scene_widget_left.scene.set_background([0.5, 0.5, 0.5, 0.4])
        scene_widget_right.scene.set_background([0.5, 0.5, 0.5, 0.4])

        bounding_box = cloud.get_axis_aligned_bounding_box()

        scene_widget_left.setup_camera(30, bounding_box, [0, 0, 0])
        scene_widget_right.setup_camera(30, bounding_box, [0, 0, 0])

        # Add labels and buttons to parent widget
        parent_widget.add_child(exit_button)
        parent_widget.add_child(file_1_label)
        parent_widget.add_child(file_2_label)

        # Adding children to main window
        window.add_child(parent_widget)
        window.add_child(scene_widget_left)
        window.add_child(scene_widget_right)

        # Run the appplication
        gui.Application.instance.run()

    def render_walkthrough(self):
        gui.Application.instance.initialize()
        # Create the window
        window = gui.Application.instance.create_window("Directory Walkthrough", window_X, window_Y)

        # Setting up parent widget
        parent_widget = gui.Widget()
        parent_widget_height = window.content_rect.height
        parent_widget_width = window.content_rect.width * .25

        # Setting parent widget specific size parameters
        parent_widget.frame = gui.Rect(0, 0, parent_widget_width, parent_widget_height)

        # Setting up the scene widget
        scene_widget = gui.SceneWidget()
        scene_widget.scene = rendering.Open3DScene(window.renderer)
        scene_widget.enable_scene_caching(True)

        # Settting specifics for the scene frame size parameters
        scene_widget.frame = gui.Rect(parent_widget_width, window.content_rect.y,
                                      window.content_rect.width - parent_widget_width,
                                      window.content_rect.height)

        # Material Settings
        material = rendering.MaterialRecord()
        material.shader = "defaultLit"

        # Get name of directory
        split_data = self.directory_walkthrough[0].split("/")
        directory = self.directory_walkthrough[0].replace(split_data[-1], "")
        # Create label to display current directory
        current_directory_label = gui.Label("Current Directory:\n" + directory)
        current_directory_label.frame = gui.Rect(10, 10, self.parent_widget_width - 50, 100)

        # Create label to display current file
        current_file_label = gui.Label("Current File:\n" + split_data[-1])
        current_file_label.frame = gui.Rect(10, 110, self.parent_widget_width - 50, 100)

        # Create a next button widget
        next_button = gui.Button(">")
        next_button.frame = gui.Rect(120, 200, 50, 100)
        next_button.set_on_clicked(lambda: self.next(scene_widget, current_file_label))

        # Create a previous button widget
        previous_button = gui.Button("<")
        previous_button.frame = gui.Rect(30, 200, 50, 100)
        previous_button.set_on_clicked(lambda: self.previous(scene_widget, current_file_label))

        # Create an exit button widget
        exit_button = gui.Button("Exit")
        exit_button.frame = gui.Rect(5, 400, self.parent_widget_width-10, 50)
        exit_button.set_on_clicked(lambda: self.exit_walkthrough(window, scene_widget))

        # Temp cloud variable for temporarily storing geometry data and bounding box use
        cloud = None
        # Loading the file into the widget scene
        for file in self.directory_walkthrough:
            cloud = self.load_geometry(file)
            scene_widget.scene.add_geometry(file, cloud, material)
            scene_widget.scene.show_geometry(file, False)
        scene_widget.scene.show_geometry(self.directory_walkthrough[0], True)

        # Setting background color
        scene_widget.scene.set_background([0.5, 0.5, 0.5, 0.4])
        bounding_box = cloud.get_axis_aligned_bounding_box()
        scene_widget.setup_camera(30, bounding_box, [0, 0, 0])

        # Adding labels to parent frame
        parent_widget.add_child(current_directory_label)
        parent_widget.add_child(current_file_label)

        # Adding buttons to parent frame
        parent_widget.add_child(exit_button)
        parent_widget.add_child(previous_button)
        parent_widget.add_child(next_button)

        # Adding children to main window
        window.add_child(parent_widget)
        window.add_child(scene_widget)

        # Run the appplication
        gui.Application.instance.run()

        return

    def next(self, scene_widget, current_file_label):
        scene_widget.scene.show_geometry(self.directory_walkthrough[self.directory_index], False)
        self.directory_index += 1
        if self.directory_index == len(self.directory_walkthrough):
            self.directory_index = 0
        scene_widget.scene.show_geometry(self.directory_walkthrough[self.directory_index], True)
        scene_widget.force_redraw()
        self.update_label(current_file_label, self.directory_walkthrough[self.directory_index])

    def previous(self, scene_widget, current_file_label):
        scene_widget.scene.show_geometry(self.directory_walkthrough[self.directory_index], False)
        self.directory_index -= 1
        if self.directory_index == -1:
            self.directory_index = len(self.directory_walkthrough) - 1
        scene_widget.scene.show_geometry(self.directory_walkthrough[self.directory_index], True)
        scene_widget.force_redraw()
        self.update_label(current_file_label, self.directory_walkthrough[self.directory_index])

    def update_label(self, current_file_label, filename):
        split_data = filename.split("/")
        current_file_label.text = "Current File:\n" + split_data[-1]

        # Define how the layout should change when the window is resized

    def exit_walkthrough(self, window, scene_widget):
        for each in self.directory_walkthrough:
            scene_widget.scene.remove_geometry(each)
        self.directory_index = 0
        self.directory_walkthrough = []
        window.show(False)
        self.window.show(True)
        #self.render(self.cached_filename)

    def exit_comparison(self, window):
        window.show(False)
        self.render(self.cached_filename)

    def initiate_comarison(self, filename):
        thread = threading.Thread(target=self.ask_comparison_files)
        thread.start()
        thread.join()
        del thread

        self.cached_filename = filename
        global file_paths
        if len(file_paths) == 2:
            self.window.show(False)
            self.compare_mode(file_paths[0], file_paths[1])
            return
        else:
            return


    def ask_comparison_files(self):
        tk = tkinter.Tk()
        tk.withdraw()
        global file_paths
        file_paths = filedialog.askopenfilenames(title="Select Two Files")
        tk.destroy()


    def load_geometry(self, filename):
        # Determine File Type
        split = filename.split("/")
       # if (".ply" or ".xyz" or ".pcl") in split[-1]:
        if filename.endswith((".xyz",".ply",".pcl")):
            geometry = o3d.io.read_point_cloud(filename)
        elif ".stl" in split[-1]:
            geometry = o3d.io.read_triangle_mesh(filename)
        else:
            print("Please Enter Proper File Type")
            sys.exit()
        return geometry

    def change_file(self, filename):
        thread = threading.Thread(target=self.ask_file)
        thread.start()
        thread.join()
        del thread
        global file_path
        if file_path == "":
            return
        else:
            self.window.show(False)
            self.render(file_path)

    def ask_file(self):
        tk = tkinter.Tk()
        tk.withdraw()
        global file_path
        file_path = filedialog.askopenfilename(title="Select File")
        tk.destroy()

    def ask_directory(self):
        tk = tkinter.Tk()
        tk.withdraw()
        global directory
        directory = filedialog.askdirectory(title="Select Directory")
        tk.destroy()

    def exit_button(self):
        gui.Application.instance.quit()
        self.window.close()
        sys.exit()

    def walkthrough(self, filename):
        thread = threading.Thread(target=self.ask_directory)
        thread.start()
        thread.join()
        del thread

        self.cached_filename = filename
        global directory
        if directory == "":
            return
        else:
            for file in sorted(os.listdir(directory), key=len):
                temp_filename = (directory + "/" + file)
                self.directory_walkthrough.append(temp_filename)
            self.window.show(False)
            self.render_walkthrough()
            return True

