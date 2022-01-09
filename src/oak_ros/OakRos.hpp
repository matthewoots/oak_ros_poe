#pragma once

#include <thread>

#include <spdlog/spdlog.h>

#include <depthai/depthai.hpp>

#include <oak_ros/OakRosInterface.hpp>

class OakRos : public OakRosInterface
{
public:
    void init(ros::NodeHandle &nh, const OakRosParams &params);

    static std::vector<std::string> getAllAvailableDeviceIds();

    dai::DeviceInfo getDeviceInfo(const std::string& device_id);

    ~OakRos(){
        m_running = false;

        if (m_run.joinable())
            m_run.join();

        spdlog::info("OakRos class destructor done.");
    }

private:

    bool m_running;
    std::string m_device_id;

    dai::Pipeline m_pipeline;

    std::shared_ptr<dai::Device> m_device;
    std::shared_ptr<dai::DataOutputQueue> leftQueue, rightQueue, depthQueue, rgbQueue;

    std::shared_ptr<dai::node::MonoCamera> m_monoLeft, m_monoRight;
    std::shared_ptr<dai::node::StereoDepth> m_stereoDepth;

    std::shared_ptr<dai::node::ColorCamera> m_colorMain;

    std::shared_ptr<dai::node::IMU> m_imu;

    std::thread m_run;
    void run();
};