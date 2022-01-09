#include <oak_ros/OakRosFactory.hpp>

#include <spdlog/spdlog.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "oak_ros");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    auto device_ids = OakRosFactory::getAllAvailableDeviceIds();

    std::vector<OakRosInterface::Ptr> oak_handlers;

    for (auto& id : device_ids)
    {
        spdlog::info("main: start device with id {}", id);
        
        OakRosInterface::Ptr handler = oak_handlers.emplace_back(OakRosFactory::getOakRosHandler());
        OakRosParams params;

        params.device_id = id;
        params.enable_stereo = true;
        params.enable_depth = true;
        handler->init(nh, params);
    }

    ros::spin();

    spdlog::info("main exits cleanly");
}