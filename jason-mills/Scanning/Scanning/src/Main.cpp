#include <librealsense2/rs.hpp> 
#include <librealsense2/rs_advanced_mode.hpp> 
#include <librealsense2/hpp/rs_export2.hpp>
#include "example.hpp"    
#include <string.h>
#include <fstream>
#include <windows.h>
#include <iostream>
#include <Eigen/Dense>
#include "PointCloud.h"


char translation_array[] = { 'a','b','c','d','e','f','g','h','i','j','k' };

char data[] = { 'X' };

int scans_per_rotation;

void register_glfw_callbacks(window& app, glfw_state& app_state);

void save_vertices(std::string filename, rs2::points points, int degrees, float translationFactor);


int main(int argc, char* argv[]) try
{
    scans_per_rotation = std::stoi(argv[1]);
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
    std::string json_file_name = "C:\\dev\\CS-425-Team-20\\app\\C3PO\\bin\\test.json";
    std::ifstream t(json_file_name);
    std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    advanced_mode_dev.load_json(preset_json);

    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;


    bool loop = true;
    int userInput = std::stoi(argv[2]);
    int degreesToRotate = userInput;
    int user_input_divided[4];
    int rotations = 360 / userInput;
    user_input_divided[3] = 10;


    for (int i = 0; i < 3; i++)
    {
        user_input_divided[2 - i] = userInput % 10;
        userInput = userInput / 10;
    }


    pipe.start();

    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);

    for (int index = 0; index < rotations; index++)
    {
        frames = pipe.wait_for_frames();
        depth = frames.get_depth_frame();
        points = pc.calculate(depth);
        std::stringstream filename;

        for (int voting = 0; voting < scans_per_rotation; voting++)
        {
            filename << "data_" << std::setw(2) << std::setfill('0') << index << "." << voting << ".txt";
            save_vertices(("C:\\dev\\CS-425-Team-20\\app\\C3PO\\output\\temp" + std::to_string(index)), points, index * degreesToRotate, 2);
            std::cout << "Scanned!" << std::endl;
        }


        // Send degree data to the arduino
        for (int i = 0; i < 4; i++)
        {
            data[0] = translation_array[user_input_divided[i]];
            WriteFile(hComm, data, sizeof(data), &dNoOfBytesWritten, NULL);
            std::cout << translation_array[user_input_divided[i]];
        }

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


void save_vertices(std::string filename, rs2::points points, int degrees, float translationFactor)
{
    PointCloud aPointCloud;
    std::cout << "Degrees to rotate: " << degrees << std::endl;

    const rs2::vertex* vertices = points.get_vertices();
    const rs2_intrinsics depth_intrinsics;

    size_t sz = points.size();

    for (int l = 0; l < sz; ++l) {
        aPointCloud.addPoint({ vertices[l].x, vertices[l].y, vertices[l].z, 0 });
    }

    aPointCloud.moveOrigin('z', translationFactor);
    aPointCloud.rotatePoints('y', degrees);


    std::cout << filename + ".xyz" << std::endl;

    aPointCloud.writeXYZFile(filename + ".xyz");
}
