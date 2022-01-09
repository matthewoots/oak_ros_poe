#include <oak_ros/OakRosFactory.hpp>

#include <spdlog/spdlog.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "oak_ros");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    auto device_ids = OakRosFactory::getAllAvailableDeviceIds();

    std::vector<OakRosInterface::Ptr> oak_handlers;

    size_t topic_name_seq = 1;
    for (auto& id : device_ids)
    {
        spdlog::info("main: start device with id {}", id);
        
        OakRosInterface::Ptr handler = oak_handlers.emplace_back(OakRosFactory::getOakRosHandler());
        OakRosParams params;

        params.device_id = id;
        params.topic_name = "oak" + std::to_string(topic_name_seq);
        params.enable_stereo = true;
        params.enable_depth = true;
        handler->init(nh_local, params);

        topic_name_seq++;
    }

    ros::spin();

    spdlog::info("main exits cleanly");
}