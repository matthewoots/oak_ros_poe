#include <oak_ros/OakRosFactory.hpp>

#include <spdlog/spdlog.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "oak_ros");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    auto handler = OakRosFactory::getOakRosHandler();

    handler->getAllAvailableDeviceIds();

    handler->init(nh, OakRosParams());

    spdlog::info("main exits cleanly");
}