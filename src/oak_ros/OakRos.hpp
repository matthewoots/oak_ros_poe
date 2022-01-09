#pragma once

#include <thread>

#include <spdlog/spdlog.h>

#include <depthai/depthai.hpp>

#include <oak_ros/OakRosInterface.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

class OakRos : public OakRosInterface
{
public:
    void init(const ros::NodeHandle &nh, const OakRosParams &params);

    static std::vector<std::string> getAllAvailableDeviceIds();

    dai::DeviceInfo getDeviceInfo(const std::string& device_id);

    ~OakRos(){
        m_running = false;

        if (m_run.joinable())
            m_run.join();

        spdlog::info("{} OakRos class destructor done.", m_device_id);
    }

private:

    bool m_running;
    std::string m_device_id;
    std::string m_topic_name;

    dai::Pipeline m_pipeline;

    std::shared_ptr<dai::Device> m_device;
    std::shared_ptr<dai::DataOutputQueue> leftQueue, rightQueue, depthQueue, rgbQueue;

    std::shared_ptr<dai::node::MonoCamera> m_monoLeft, m_monoRight;
    std::shared_ptr<dai::node::StereoDepth> m_stereoDepth;

    std::shared_ptr<dai::node::ColorCamera> m_colorMain;

    std::shared_ptr<dai::node::IMU> m_imu;

    std::thread m_run;
    void run();

    void depthCallback(std::shared_ptr<dai::ADatatype> data);

    // ROS related functionalities
    // ros::NodeHandle m_nh;
    std::shared_ptr<image_transport::ImageTransport> m_imageTransport;
    std::shared_ptr<image_transport::CameraPublisher> m_leftPub, m_rightPub;

};