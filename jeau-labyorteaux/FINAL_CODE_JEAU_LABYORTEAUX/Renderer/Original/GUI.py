import os
import sys
import tkinter as tk
from tkinter import filedialog
import open3d as o3d


# Simple function to display a pop-up window if the user selects the wrong file type to display
def create_popup_window(message):
    popup_window = tk.Tk()
    popup_window.title("Warning")
    popup_window.geometry("250x100")
    label = tk.Label(popup_window, text=message)
    label.pack(pady=20)
    button = tk.Button(popup_window, text="Close", command=popup_window.destroy)
    button.pack()
    popup_window.mainloop()


class GUI:
    def __init__(self, root):
        self.root = root
        self.filename = sys.argv[1]

        self.directory = None
        self.directory_files = []
        self.directory_index = 0
        self.directory_vis = None

        self.color_pc = True
        self.file_pc = True
        # Set the minimum size of the window
        self.root.minsize(350, 250)

        self.mid_frame = tk.Frame(self.root)

        # Create the 'Exit' button
        self.exit_button = tk.Button(self.root, text="Exit", command=self.exit_program, width=10, height=1)
        self.exit_button.pack(side='bottom', padx=10, pady=10)

        # Label
        self.file_display = tk.Label(self.root, text="Current File:\n" + self.filename)
        self.file_display.pack(side='top', padx=10, pady=10)

        # Create the 'Open' button for point cloud files
        self.open_button = tk.Button(self.root, text="Open", command=self.open_file, width=30, height=5)
        self.open_button.pack(side='top', padx=10, pady=10)

        # Create the 'Directory Walkthrough' button for walkthrough mode
        self.open_button = tk.Button(self.root, text="Directory Walkthrough",
                                     command=self.directory_walkthrough, width=18, height=1)
        self.open_button.pack(side='bottom', padx=10, pady=10)

        # Create the 'Change File' button for point cloud files
        self.change_file_button = tk.Button(self.mid_frame, text="Change File", command=self.change_file, width=10,
                                            height=1)
        self.change_file_button.pack(side='left', padx=20, pady=10)

        # Dropdown bar for color or no color files
        options = ["Color", "No Color"]
        self.text = tk.StringVar()
        self.text.set("Color")

        self.drop = tk.OptionMenu(self.mid_frame, self.text, *options)
        self.drop.pack(side='right', padx=20, pady=10)

        # Dropdown bar for point cloud or non point cloud files files
        options2 = ["Point Cloud (.xyz, .pdc, .ply)", "Non-Point Cloud (.stl)"]
        self.text2 = tk.StringVar()
        self.text2.set("Point Cloud (.xyz, .pdc, .ply)")

        self.drop = tk.OptionMenu(self.mid_frame, self.text2, *options2)
        self.drop.pack(side='right', padx=20, pady=10)

        self.mid_frame.pack()

    def directory_walkthrough(self):
        self.directory = filedialog.askdirectory()
        # Check in case user exits out of directory select early
        if self.directory == "":
            return
        self.update_color_pc()

        self.update_file_pc()
        split = self.filename.split(".")
        extension = split[-1]

        if "stl" in extension and (self.file_pc is True):
            create_popup_window("Wrong File Type")
            return
        elif "stl" in extension and (self.file_pc is False):
            for root, dirs, files in os.walk(self.directory):
                for filename in sorted(files, key=len):
                    pcd = o3d.io.read_triangle_mesh(self.directory + "/" + str(filename))
                    self.directory_files.append(pcd)
        elif "stl" not in extension and (self.file_pc is True):
            for root, dirs, files in os.walk(self.directory):
                for filename in sorted(files, key=len):
                    pcd = o3d.io.read_point_cloud(self.directory + "/" + str(filename))
                    self.directory_files.append(pcd)
        else:
            create_popup_window("Wrong File Type")
            return

        self.directory_vis = o3d.visualization.VisualizerWithKeyCallback()
        self.directory_vis.create_window(window_name='Directory Walkthrough', width=800, height=600)
        # 68 ASCII code for "d"
        self.directory_vis.register_key_callback(ord("D"), self.next_file)
        # 65 ASCII code for "a"
        self.directory_vis.register_key_callback(ord("A"), self.prev_file)
        if self.color_pc:
            self.directory_vis.add_geometry(self.directory_files[0])
        else:
            self.directory_vis.add_geometry(self.directory_files[0].paint_uniform_color([0.5, 0.5, 0.5]))

        self.directory_vis.run()

    def next_file(self, event):
        self.directory_index += 1
        if self.directory_index == len(self.directory_files):
            # When we reach the last file close window, essentially reset all directory data
            self.directory_index = 0
            self.directory_files = []
            self.directory = None
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
        self.update_file_pc()
        split = self.filename.split(".")
        extension = split[-1]
        if "stl" in extension and (self.file_pc is True):
            create_popup_window("Wrong File Type")
            return
        elif "stl" in extension and (self.file_pc is False):
            pcd = o3d.io.read_triangle_mesh(self.filename)
        elif "stl" not in extension and (self.file_pc is True):
            pcd = o3d.io.read_point_cloud(self.filename)
        else:
            create_popup_window("Wrong File Type")
            return

        self.update_color_pc()

        # Create Open3D visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=self.filename, width=800, height=600)
        # if/else depends on if user wants to render with color or not
        if self.color_pc:
            vis.add_geometry(pcd)
        else:
            vis.add_geometry(pcd.paint_uniform_color([0.5, 0.5, 0.5]))
        vis.run()

    # Function for changing the file the program will open
    def change_file(self):
        temp = filedialog.askopenfilename()
        if temp != "":
            self.filename = temp
            self.update_label()

    # Function to update label text
    def update_label(self):
        self.file_display.config(text="Current File:\n" + self.filename)

    def update_color_pc(self):
        if "No Color" in self.text.get():
            self.color_pc = False
        else:
            self.color_pc = True

    def update_file_pc(self):
        if "stl" in self.text2.get():
            self.file_pc = False
        else:
            self.file_pc = True

    # Give the user an easy option to exit the program
    def exit_program(self):
        self.root.destroy()
