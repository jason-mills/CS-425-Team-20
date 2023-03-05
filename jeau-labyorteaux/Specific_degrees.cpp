#include <librealsense2/rs.hpp> 
#include <librealsense2/rs_advanced_mode.hpp> 
#include <librealsense2/hpp/rs_export2.hpp>
#include "example.hpp"    
#include <string.h>
#include <fstream>
#include <windows.h>
#include <iostream>


void register_glfw_callbacks(window& app, glfw_state& app_state);

void save_vertices(const std::string& filename, rs2::points points);


int main(int argc, char* argv[]) try
{
    //Opening Serial Communication Port For Arduino Communication
    HANDLE hComm;
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
        std::cerr << "opening serial port successful\n";


  //window app(1280, 720, "RealSense Pointcloud Example");
    glfw_state app_state;
   //register_glfw_callbacks(app, app_state);
    rs2::context ctx;
   auto device = ctx.query_devices();
   auto dev = device[0];
 

   auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
   // Select the custom configuration file
   std::string json_file_name = "JSON/test.json";
   std::ifstream t(json_file_name);
   std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
   advanced_mode_dev.load_json(preset_json);

    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;
    

    bool loop = true;
    char user_choice;
    int user_degrees = 0;
    char data[] = { 'X' };

    while (loop) 
    {
        std::cout << "Enter Degree Option: " << std::endl;
        std::cout << "A: 30" << std::endl;
        std::cout << "B: 45" << std::endl;
        std::cout << "C: 60" << std::endl;
        std::cout << "D: 90" << std::endl;
        std::cout << "E: 180" << std::endl;
        std::cin >> user_choice;
        if (user_choice == 'A')
        {
            data[0] = 'A';
            user_degrees = 30;
            loop = false;
        }
        else if (user_choice == 'B')
        {
            data[0] = 'B';
            user_degrees = 45;
            loop = false;
        }
        else if (user_choice == 'C')
        {
            data[0] = 'C';
            user_degrees = 60;
            loop = false;
        }
        else if (user_choice == 'D')
        {
            data[0] = 'D';
            user_degrees = 90;
            loop = false;
        }
        else if (user_choice == 'E')
        {
            data[0] = 'E';
            user_degrees = 180;
            loop = false;
        }
        else
        {
            std::cout << "Please enter one of the letter options" << std::endl;
        }
    }


    pipe.start();

    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);

    for (int index = 0; index * user_degrees != 360; index++)
    {
        std::stringstream filename;
        filename << "data_" << std::setw(2) << std::setfill('0') << index << ".txt";


        frames = pipe.wait_for_frames();
        depth = frames.get_depth_frame();
        points = pc.calculate(depth);

        save_vertices(filename.str(), points);
        std::cout << "Scanned!" << std::endl;
        
        WriteFile(hComm, data, sizeof(data), &dNoOfBytesWritten, NULL);
        Sleep(5000);

    }
 

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


void save_vertices(const std::string& filename, rs2::points points)
{
    const rs2::vertex* vertices = points.get_vertices();
    const rs2_intrinsics depth_intrinsics;

    size_t sz = points.size();

    std::ofstream out(filename, std::ofstream::binary);

    out << "X Y Z" << std::endl;

    for (int l = 0; l < sz; ++l) {
        float point[3];
        point[0] = vertices[l].x;
        point[1] = vertices[l].y;
        point[2] = vertices[l].z;

        out << point[0] << ", " << point[1] << ", " << point[2] << std::endl;
    }

    out.close();
    return;
}

