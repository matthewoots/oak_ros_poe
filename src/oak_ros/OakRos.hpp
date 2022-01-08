#pragma once

#include <depthai/depthai.hpp>

#include <oak_ros/OakRosInterface.hpp>

class OakRos : public OakRosInterface
{
public:
    void init(ros::NodeHandle &nh, const OakRosParams &params);

    std::vector<std::string> getAllAvailableDeviceIds();

    dai::DeviceInfo getDeviceInfo(const std::string& device_id);

private:

    dai::Pipeline m_pipeline;

    std::shared_ptr<dai::Device> m_device;

    std::shared_ptr<dai::node::MonoCamera> m_monoLeft, m_monoRight;
    std::shared_ptr<dai::node::StereoDepth> m_stereoDepth;

    std::shared_ptr<dai::node::ColorCamera> m_colorMain;

    std::shared_ptr<dai::node::IMU> m_imu;
};