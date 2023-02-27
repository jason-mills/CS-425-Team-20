#include <librealsense2/rs.hpp> 
#include <librealsense2/rs_advanced_mode.hpp> 
#include <librealsense2/hpp/rs_export2.hpp>
#include "example.hpp"    
#include <string.h>
#include <fstream>
#include <windows.h>
#include <iostream>


void register_glfw_callbacks(window& app, glfw_state& app_state);

bool save_frame_raw_data(const std::string& filename, rs2::pipeline pipe);

int main(int argc, char* argv[]) try
{
    //Opening Serial Communication Port For Arduino Communication
    HANDLE hComm;
    char data[] = {'A'};
    DWORD dNoOFBytestoWrite;         // No of bytes to write into the port
    DWORD dNoOfBytesWritten = 0;     // No of bytes written to the port
    DWORD bytes_read = 0;

    hComm = CreateFileA("\\\\.\\COM3", GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);        // Null for Comm Devices

    if (hComm == INVALID_HANDLE_VALUE)
    {
        std::cerr << "Error in opening serial port";
        return -1;
    }
    else
        std::cerr << "opening serial port successful";


  //window app(1280, 720, "RealSense Pointcloud Example");
    glfw_state app_state;
   //register_glfw_callbacks(app, app_state);
    rs2::context ctx;
   auto device = ctx.query_devices();
   auto dev = device[0];
 

   auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
   // Select the custom configuration file
   std::string json_file_name = "test.json";
   std::ifstream t(json_file_name);
   std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
   advanced_mode_dev.load_json(preset_json);

    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;

    pipe.start();
    
   
    //Scan 1
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);
    points.export_to_ply("scans/1.ply", points);
    save_frame_raw_data("scans/frame2.txt", pipe);
    if (save_frame_raw_data("scans/frame.txt", pipe))
    {
        std::cout << "Frame Data Exported!" << std::endl;
    }
    std::cout << "Scanned!" << std::endl;

    WriteFile(hComm, data, sizeof(data), &dNoOfBytesWritten, NULL);
    Sleep(5000);

    //Scan 2
     frames = pipe.wait_for_frames();
     depth = frames.get_depth_frame();
    points = pc.calculate(depth);
    points.export_to_ply("scans/2.ply", points);
    std::cout << "Scanned!" << std::endl;

    WriteFile(hComm, data, sizeof(data), &dNoOfBytesWritten, NULL);
    Sleep(5000);

    //Scan 3
     frames = pipe.wait_for_frames();
     depth = frames.get_depth_frame();
    points = pc.calculate(depth);
    points.export_to_ply("scans/3.ply", points);
    std::cout << "Scanned!" << std::endl;

    WriteFile(hComm, data, sizeof(data), &dNoOfBytesWritten, NULL);
    Sleep(5000);

    //Scan 4
     frames = pipe.wait_for_frames();
     depth = frames.get_depth_frame();
    points = pc.calculate(depth);
    points.export_to_ply("scans/4.ply", points);
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


bool save_frame_raw_data(const std::string& filename, rs2::pipeline pipe)
{
    bool ret = false;
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    if (depth)
    {
        std::ofstream outfile(filename.data(), std::ofstream::binary);
        outfile.write(static_cast<const char*>(depth.get_data()), depth.get_height() * depth.get_stride_in_bytes());

        outfile.close();
        ret = true;
    }

    return ret;
}
