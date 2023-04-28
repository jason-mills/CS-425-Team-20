import os
import sys
import tkinter as tk
from tkinter import filedialog
import open3d as o3d


class GUI:
    def __init__(self, root):
        self.root = root
        self.filename = sys.argv[1]

        self.directory = None
        self.directory_files = []
        self.directory_index = 0
        self.directory_vis = None

        self.color_pc = True
        # Set the minimum size of the window
        self.root.minsize(600, 300)

        self.left_sub_menu = tk.Frame(root)
        self.right_sub_menu = tk.Frame(root)

        # Create the 'Exit' button
        self.exit_button = tk.Button(self.root, text="Exit", command=self.exit_program, width=10, height=1)
        self.exit_button.pack(side='bottom', padx=10, pady=10)

        # Left Side Label
        self.file_display = tk.Label(self.left_sub_menu, text="Point Cloud Files")
        self.file_display.pack(side='top', padx=10, pady=10)

        # Right Side Label
        self.file_display = tk.Label(self.right_sub_menu, text="Non-Point Cloud Files")
        self.file_display.pack(side='top', padx=10, pady=10)

        # File label widget
        self.file_display = tk.Label(self.left_sub_menu, text="Current File:\n" + self.filename)
        self.file_display.pack(side='top', padx=10, pady=10)

        # Create the 'Open' button for point cloud files
        self.open_button = tk.Button(self.left_sub_menu, text="Open", command=self.open_file, width=30, height=5)
        self.open_button.pack(side='top', padx=10, pady=10)

        # Create the 'Directory Walkthrough' button for walkthrough mode
        self.open_button = tk.Button(self.left_sub_menu, text="Directory Walkthrough",
                                     command=self.directory_walkthrough, width=18, height=1)
        self.open_button.pack(side='bottom', padx=10, pady=10)

        # Create the 'Change File' button
        self.change_file_button = tk.Button(self.left_sub_menu, text="Change File", command=self.change_file, width=10,
                                            height=1)
        self.change_file_button.pack(side='left', padx=20, pady=10)

        # Dropdown bar for point cloud files
        options = ["Color", "No Color"]
        self.text = tk.StringVar()
        self.text.set("Color")

        self.drop = tk.OptionMenu(self.left_sub_menu, self.text, *options)
        self.drop.pack(side='right', padx=20, pady=10)

        self.left_sub_menu.pack(side='left')
        self.right_sub_menu.pack(side='right')

    def directory_walkthrough(self):
        self.directory = filedialog.askdirectory()
        self.update_color_pc()
        for root, dirs, files in os.walk(self.directory):
            for filename in sorted(files, key=len):
                print(filename)
                pcd = o3d.io.read_point_cloud(self.directory + "/" + str(filename))
                self.directory_files.append(pcd)
        self.directory_vis = o3d.visualization.VisualizerWithKeyCallback()
        self.directory_vis.create_window(window_name='Open3D Walkthrough', width=800, height=600)
        # 68 ASCII code for "d"
        self.directory_vis.register_key_callback(ord("D"), self.next_file)
        # 65 ASCII code for "a"
        self.directory_vis.register_key_callback(ord("A"), self.prev_file)
        self.directory_vis.add_geometry(self.directory_files[0])
        self.directory_vis.run()

    def next_file(self, event):
        self.directory_index += 1
        if self.directory_index == len(self.directory_files):
            # When we reach the last file close window, clear list and index
            self.directory_index = 0
            self.directory_files = []
            self.directory_vis.destroy_window()
            return True
        pcd = self.directory_files[self.directory_index]
        # if/else depends on if user wants to render with color or not
        if self.color_pc:
            self.directory_vis.clear_geometries()
            self.directory_vis.add_geometry(pcd)
        else:
            self.directory_vis.clear_geometries()
            self.directory_vis.add_geometry(pcd.paint_uniform_color([0.5, 0.5, 0.5]))
        return True

    def prev_file(self, event):
        self.directory_index -= 1
        if self.directory_index < 0:
            self.directory_index = 0
            return True
        pcd = self.directory_files[self.directory_index]
        # if/else depends on if user wants to render with color or not
        if self.color_pc:
            self.directory_vis.clear_geometries()
            self.directory_vis.add_geometry(pcd)
        else:
            self.directory_vis.clear_geometries()
            self.directory_vis.add_geometry(pcd.paint_uniform_color([0.5, 0.5, 0.5]))
        return True

    def open_file(self):
        # Load point cloud data
        pcd = o3d.io.read_point_cloud(self.filename)
        self.update_color_pc()

        # Create Open3D visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='Open3D Visualization', width=800, height=600)
        # if/else depends on if user wants to render with color or not
        if self.color_pc:
            vis.add_geometry(pcd)
        else:
            vis.add_geometry(pcd.paint_uniform_color([0.5, 0.5, 0.5]))
        vis.run()

    # Function for changing the file the program will open
    def change_file(self):
        self.filename = filedialog.askopenfilename()
        self.update_label()

    # Function to update label text
    def update_label(self):
        self.file_display.config(text="Current File: " + self.filename)

    def update_color_pc(self):
        if "No Color" in self.text.get():
            self.color_pc = False
        else:
            self.color_pc = True

    # Give the user an easy option to exit the program
    def exit_program(self):
        self.root.destroy()
