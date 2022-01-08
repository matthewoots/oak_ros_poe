#include "OakRos.hpp"

#include <spdlog/spdlog.h>

void OakRos::init(ros::NodeHandle &nh, const OakRosParams &params)
{
}

std::vector<std::string> OakRos::getAllAvailableDeviceIds()
{
    auto DeviceInfo_vec = dai::Device::getAllAvailableDevices();

    spdlog::info("Found {} devices:", DeviceInfo_vec.size());

    std::vector<std::string> ret;

    for (auto &info : DeviceInfo_vec)
    {
        ret.push_back(info.getMxId());
        spdlog::info(info.getMxId());
    }

    return ret;
}