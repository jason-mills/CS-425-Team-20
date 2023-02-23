#include <librealsense2/rs.hpp> 
#include <librealsense2/rs_advanced_mode.hpp> 
#include "example.hpp"    
#include <string.h>


void register_glfw_callbacks(window& app, glfw_state& app_state);

int main(int argc, char* argv[]) try
{
    
    window app(1280, 720, "RealSense Pointcloud Example");
    glfw_state app_state;
    register_glfw_callbacks(app, app_state);
    rs2::context ctx;
    rs2::config cfg;
    auto device = ctx.query_devices();
    auto dev = device[0];
    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
    // Select the custom configuration file
    std::string json_file_name = "small-objects.json";
    advanced_mode_dev.load_json(json_file_name);
    cfg.enable_device(serial);
    rs2::pointcloud pc;
    rs2::points points;

    rs2::pipeline pipe;
    pipe.start();
    while (app) // Application still alive?
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();
        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        // Upload the color frame to OpenGL
        app_state.tex.upload(color);

        // Draw the pointcloud
        draw_pointcloud(app.width(), app.height(), app_state, points);
    }

    auto frames = pipe.wait_for_frames();
    auto color = frames.get_color_frame();
    // Tell pointcloud object to map to this color frame
    pc.map_to(color);

    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    // Upload the color frame to OpenGL
    app_state.tex.upload(color);

    // Draw the pointcloud
    draw_pointcloud(app.width(), app.height(), app_state, points);
    points.export_to_ply("new.ply", color);

    return EXIT_SUCCESS;
}



catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
