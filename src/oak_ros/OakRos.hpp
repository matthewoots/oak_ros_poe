#pragma once

#include <depthai/depthai.hpp>

#include <oak_ros/OakRosInterface.hpp>

class OakRos : public OakRosInterface
{
public:
    void init(ros::NodeHandle &nh, const OakRosParams &params);

    std::vector<std::string> getAllAvailableDeviceIds();

private:
};