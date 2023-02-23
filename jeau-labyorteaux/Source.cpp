#include <librealsense2/rs.hpp> 
#include <librealsense2/rs_advanced_mode.hpp> 
#include <librealsense2/hpp/rs_export2.hpp>
#include "example.hpp"    
#include <string.h>
#include <fstream>
#include <windows.h>


void register_glfw_callbacks(window& app, glfw_state& app_state);

int main(int argc, char* argv[]) try
{
    
   //window app(1280, 720, "RealSense Pointcloud Example");
    glfw_state app_state;
  // register_glfw_callbacks(app, app_state);
    rs2::context ctx;
   auto device = ctx.query_devices();
   auto dev = device[0];
 

   auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
   // Select the custom configuration file
   std::string json_file_name = "neeew.json";
   std::ifstream t(json_file_name);
   std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
   advanced_mode_dev.load_json(preset_json);

    rs2::hole_filling_filter holeFilter;
    rs2::decimation_filter decFilter;
    rs2::sequence_id_filter seqFilter;
    rs2::threshold_filter threshFilter;
    rs2::spatial_filter spacFilter;
    
    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;

    
    pipe.start();
    /*
    while (app) // Application still alive?
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        auto data = pipe.wait_for_frames().apply_filter(holeFilter);//
        auto color = frames.get_color_frame();
        // Tell pointcloud object to map to this color frame
        //pc.map_to(color);
        
        auto depth = frames.get_depth_frame();


        // Generate the pointcloud and texture mappings
        //points = pc.calculate(depth);
        rs2::frame filtered = depth;
        filtered = decFilter.process(filtered);
        //filtered = seqFilter.process(filtered);
       //filtered = threshFilter.process(filtered);
        filtered = spacFilter.process(filtered);
        points = pc.calculate(filtered);

        // Upload the color frame to OpenGL
       // app_state.tex.upload(color);

        // Draw the pointcloud
        draw_pointcloud(app.width(), app.height(), app_state, points);
    }
    */
    /*
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    auto data = pipe.wait_for_frames().apply_filter(holeFilter);//
    auto color = frames.get_color_frame();
    // Tell pointcloud object to map to this color frame
    //pc.map_to(color);

    auto depth = frames.get_depth_frame();


    // Generate the pointcloud and texture mappings
    //points = pc.calculate(depth);
    rs2::frame filtered = depth;
    filtered = decFilter.process(filtered);
   //filtered = seqFilter.process(filtered);
   //filtered = threshFilter.process(filtered);
    filtered = spacFilter.process(filtered);
    points = pc.calculate(filtered);

    // Upload the color frame to OpenGL
    //app_state.tex.upload(color);
    
    points.export_to_ply("new.ply", color);

    */
    
    //Scan 1
    auto frames = pipe.wait_for_frames();
    //auto color = frames.get_color_frame();
    // Tell pointcloud object to map to this color frame
    //pc.map_to(color);
    auto depth = frames.get_depth_frame();

    rs2::frame filtered = depth;
    filtered = decFilter.process(filtered);
    //filtered = seqFilter.process(filtered);
   //filtered = threshFilter.process(filtered);
    filtered = spacFilter.process(filtered);
    
   

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);
    
    points.export_to_ply("new.ply", points);
    // Upload the color frame to OpenGL
                    //app_state.tex.upload(color);
    // Draw the pointcloud
   // draw_pointcloud(app.width(), app.height(), app_state, points);
    //points.export_to_ply("1.ply", color);
    
    std::cout << "Scanned!" << std::endl;
 

    
    
    
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
