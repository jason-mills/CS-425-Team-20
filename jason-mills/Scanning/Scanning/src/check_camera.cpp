#include <librealsense2/rs.hpp> 
#include <iostream>

int main(int argc, char* argv[])
{
    rs2::context ctx;
    auto device = ctx.query_devices();

    size_t device_count = device.size();
    if (!device_count)
    {
        std::cout << "No device detected.\n";
        return -1;
    }
    else
        std::cerr << "Opening D405 Successful";

    return 0;
}