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
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"



char translation_array[] = { 'a','b','c','d','e','f','g','h','i','j','k' };

char data[] = { 'X' };

int scans_per_rotation;

int global_points_captured = 0;

void write_metadata(int partitions, int scans, std::string path);

void register_glfw_callbacks(window& app, glfw_state& app_state);

void save_vertices(std::string filename, rs2::points points, int degrees, float translationFactor);

bool check_object(rs2::pipeline pipe);

void count_lines(std::string filename);


int main(int argc, char* argv[]) try
{
    scans_per_rotation = std::stoi(argv[1]);
    //Opening Serial Communication Port For Arduino Communication
    HANDLE hComm;
    DWORD dNoOFBytestoWrite;         // No of bytes to write into the port
    DWORD dNoOfBytesWritten = 0;     // No of bytes written to the port
    DWORD bytes_read = 0;

    hComm = CreateFileA("\\\\.\\COM3", GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);        // Null for Comm Devices

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hComm, &dcb);
    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    SetCommState(hComm, &dcb);


    if (hComm == INVALID_HANDLE_VALUE)
    {
        std::cerr << "Error in opening serial port\n";
        return -1;
    }
    else
        std::cerr << "Opening serial port successful\n";




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

    size_t device_count = device.size();
    if (!device_count)
    {
        std::cout << "No device detected.\n";
        //return -1;
    }


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
    /*
    // Basic check to see if the user placed the object on the platform
    if (!check_object(pipe))
    {
        std::cerr << "Object Not Found on Platform";
        return -1;
    }
    */
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
            //filename << "data_" << std::setw(2) << std::setfill('0') << index << "." << voting << ".txt";
            save_vertices((argv[3] + std::to_string(index)), points, index * degreesToRotate, 2);
            std::cout << "Scanned! " << std::endl;
            count_lines(argv[3] + std::to_string(index) + ".xyz");
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
    int deg = std::stoi(argv[2]);
    write_metadata((360/deg), (scans_per_rotation*(360/deg)), argv[4]);
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

bool check_object(rs2::pipeline pipe)
{
    float average = 0;
    float total = 0;
    // Number of iterations will determine how forgiving depth scan is
    for (int i = 0; i < 10; i++)
    {
        // Waiting for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Get the depth frame
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth value at the center of the frame
        float distance = depth.get_distance(depth.get_width() / 2, depth.get_height() / 2);

        // Calculating average to ward against false positives
        total = total + distance;
        average = total / (i + 1);
    }
    // Generate an average so threshold of item in view can be forgiving 
    if (average == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void write_metadata(int partitions, int scans, std::string path)
{
    // Create the JSON file object
    rapidjson::Document metadata;
    metadata.SetObject();

    // Add number of partitions
    rapidjson::Value partitions_count;
    partitions_count.SetInt(partitions);
    metadata.AddMember("Partitions", partitions_count, metadata.GetAllocator());

    // Add number of scans
    rapidjson::Value scans_count;
    scans_count.SetInt(scans);
    metadata.AddMember("Scans Taken", scans_count, metadata.GetAllocator());

    // Add points captured
    rapidjson::Value points_captured;
    points_captured.SetInt(global_points_captured);
    metadata.AddMember("Points Captured", points_captured, metadata.GetAllocator());

    // Create output file at specified path
    std::ofstream output(path + "data.json");
  
    // Load data into JSON data into buffer
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    metadata.Accept(writer);

    // Write buffer to file
    output << buffer.GetString() << std::endl;

    // Make sure to close file
    output.close();
}

void count_lines(std::string filename) 
{
    std::ifstream file(filename);
    std::string line;
    
    while (std::getline(file, line)) 
    {
        global_points_captured++;
    }

    file.close();

    return;
}

