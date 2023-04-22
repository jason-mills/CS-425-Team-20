import argparse
import open3d as o3d
import numpy as np

def getArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str, required=True)
    parser.add_argument("--out", type=str, required=True)
    args, unknown = parser.parse_known_args()
    return args

def screenshotImage(fIn, fOut):
    cloud = o3d.io.read_point_cloud(fIn)
    cloud.paint_uniform_color([0.5, 0.5, 0.5])
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(cloud)
    vis.update_geometry(cloud)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(fOut)
    vis.destroy_window()

def main():
    args = getArgs()
    fPath = args.file.replace("\\", "/")
    oPath = args.out.replace("\\", "/")
    screenshotImage(fPath, oPath)
    
    # Load a point cloud
    #cloud = o3d.io.read_point_cloud(fPath)
    
    # Creath depth image from point cloud
    #depthCloud = np.asarray(cloud.compute_depth_image())
    
    # Create color image from depth image
    #colorImage = o3d.geometry.Image.create_from_depth(depthCloud)
    
    # Write color image
    #o3d.io.write_image(oPath, colorImage)
    
    

if __name__ == "__main__":
    main()