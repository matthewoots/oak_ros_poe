#include <oak_ros/OakRosFactory.hpp>

#include <spdlog/spdlog.h>

// Preset good for indoor calibration

OakRosParams getIndoorLightingParams()
{
    OakRosParams params;

    // enable raw stereo output, without depth generation
    params.enable_stereo = true;
    params.enable_depth = false;

    params.enable_imu = true;

    params.manual_exposure = 2500; // in usec
    params.manual_iso = 800; // 100 to 1600

    return params;
}

OakRosParams getCameraCalibrationParams()
{
    auto params = getIndoorLightingParams();

    // set to around 4 Hz data aquisition rate
    params.stereo_fps_throttle = 8;

    return params;
}

// preset that good for indoor low light
OakRosParams getLowLightParams()
{
    OakRosParams params;

    // enable raw stereo output, without depth generation
    params.enable_stereo = true;
    params.enable_depth = false;

    params.enable_imu = true;

    params.manual_exposure = 8000; // in usec
    params.manual_iso = 1600; // 100 to 1600

    return params;
}

int main(int argc, char **argv)
{
    // spdlog::set_level(spdlog::level::debug);

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

        auto params = getCameraCalibrationParams();
        params.device_id = id;
        params.topic_name = "oak" + std::to_string(topic_name_seq);

        handler->init(nh_local, params);

        topic_name_seq++;
    }

    ros::spin();

    spdlog::info("main exits cleanly");
}