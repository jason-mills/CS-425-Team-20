import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np
import os
import sys
from time import sleep

class WindowApp:

    def __init__(self):
        self.window = gui.Application.instance.create_window("Spinnables", 1400, 900)
        w = self.window

        # member variables
        self.model_dir = ""
        self.model_name = ""

        em = w.theme.font_size
        # 3D Widget
        _widget3d = gui.SceneWidget()
        _widget3d.scene = rendering.Open3DScene(w.renderer)
        _widget3d.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)
        # create a frame that encapsulates the Scenewidget
        _widget3d.frame = gui.Rect(500, w.content_rect.y,
                                        900, w.content_rect.height)
        # mesh = o3d.geometry.TriangleMesh.create_sphere()
        # mesh.compute_vertex_normals()
        # material = rendering.MaterialRecord()
        # material.shader = "defaultLit"
        # _widget3d.scene.add_geometry('mesh', mesh, material)
        _widget3d.scene.set_background([200, 0, 0, 200]) # not working?!
        # _widget3d.scene.camera.look_at([0, 0, 0], [1, 1, 1], [0, 0, 1])
        _widget3d.set_on_mouse(self._on_mouse_widget3d)

        # gui layout
        gui_layout = gui.Vert(0, gui.Margins(0.5 * em, 0.5 * em, 0.5 * em, 0.5 * em))
        # create frame that encapsulates the gui
        gui_layout.frame = gui.Rect(w.content_rect.x, w.content_rect.y,
                                    500, w.content_rect.height)
        # File-chooser widget
        self._fileedit = gui.TextEdit()
        filedlgbutton = gui.Button("...")
        filedlgbutton.horizontal_padding_em = 0.5
        filedlgbutton.vertical_padding_em = 0
        filedlgbutton.set_on_clicked(self._on_filedlg_button)

        fileedit_layout = gui.Horiz()
        fileedit_layout.add_child(gui.Label("Model file"))
        fileedit_layout.add_child(self._fileedit)
        fileedit_layout.add_fixed(0.25 * em)
        fileedit_layout.add_child(filedlgbutton)
        # add to the top-level (vertical) layout
        gui_layout.add_child(fileedit_layout)

        w.add_child(gui_layout)
        w.add_child(_widget3d)

    def setup_scene(self):
        return

    def update_scene(self):
        return

    def _on_mouse_widget3d(self, event):
        print(event.type)
        return gui.Widget.EventCallbackResult.IGNORED

    def _on_filedlg_button(self):
        filedlg = gui.FileDialog(gui.FileDialog.OPEN, "Select file",
                                 self.window.theme)
        filedlg.add_filter(".obj .ply .stl", "Triangle mesh (.obj, .ply, .stl)")
        filedlg.add_filter("", "All files")
        filedlg.set_on_cancel(self._on_filedlg_cancel)
        filedlg.set_on_done(self._on_filedlg_done)
        self.window.show_dialog(filedlg)

    def _on_filedlg_cancel(self):
        self.window.close_dialog()

    def _on_filedlg_done(self, path):
        self._fileedit.text_value = path
        self.model_dir = os.path.normpath(path)
        # self.update_scene()
        self._widget3d
        self.window.close_dialog()

# def Viewer(object):
#     def __init__(self, title):
#         app = gui.Application.instance
#         app.initialize()

#         self.main_vis = o3d.visualization.O3DVisualizer(title)

#         self.setupPointClouds()
#         self.setupScene()

#     def update_o3d_scene(self):
#         self.main_vis.remove_geometry(self.point_cloud_o3d_name)
#         self.main_vis.add_geometry(self.point_cloud_o3d_name, self.ponit_cloud_o3d)

#     def run_one_tick(self):
#         app = gui.application.instance
#         tick_return = app.run_one_tick()
#         if tick_return:
#             self.main_vis.post_redraw()
#         return tick_return

def main():
    # viewer = Viewer("Title")
    # while True:
    #     viewer.update_point_clouds()
    #     viewer.update_o3d_scene()
    #     sleep(5)
    gui.Application.instance.initialize()
    w = WindowApp()
    gui.Application.instance.run()

if __name__ == '__main__':
    main()