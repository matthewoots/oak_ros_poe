#include <oak_ros/OakRosFactory.hpp>

#include <spdlog/spdlog.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "oak_ros");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    auto handler = OakRosFactory::getOakRosHandler();

    handler->getAllAvailableDeviceIds();

    OakRosParams params;
    handler->init(nh, params);

    spdlog::info("main exits cleanly");
}