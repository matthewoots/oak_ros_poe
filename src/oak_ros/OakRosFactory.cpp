#include <oak_ros/OakRosFactory.hpp>

#include "OakRos.hpp"

OakRosInterface::Ptr OakRosFactory::getOakRosHandler()
{
    return std::make_shared<OakRos>();
}

std::vector<std::string> OakRosFactory::getAllAvailableDeviceIds()
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