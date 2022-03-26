#pragma once

#include <thread>

#include <spdlog/spdlog.h>

#include <depthai/depthai.hpp>

#include <oak_ros/OakRosInterface.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>

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
    OakRosParams m_params;
    bool m_stereo_is_rectified;

    // 1 means no throttling, only publish every N frames
    unsigned int m_stereo_seq_throttle = 1;
    unsigned int lastSeq = 0;
    unsigned int lastPublishedSeq = 0;
    double lastGyroTs = -1;
    
    bool m_ts_align_to_right;

    std::string m_device_id;
    std::string m_topic_name;

    dai::Pipeline m_pipeline;

    std::shared_ptr<dai::Device> m_device;
    std::shared_ptr<dai::DataOutputQueue> m_leftQueue, m_rightQueue, m_depthQueue, m_rgbQueue, m_imuQueue;
    std::shared_ptr<dai::DataOutputQueue> m_opticalFlowLeftQueue, m_opticalFlowRightQueue;
    std::shared_ptr<dai::DataOutputQueue> m_opticalFlowLeftPassQueue, m_opticalFlowRightPassQueue;

    std::shared_ptr<dai::DataInputQueue> m_controlQueue, m_configOpticalFlowQueue;

    // WORKAROUND FOR OV7251, inability to reduce framerate
    std::shared_ptr<dai::node::Script> m_scriptLeft;
    std::shared_ptr<dai::node::Script> m_scriptRight;

    std::shared_ptr<dai::node::MonoCamera> m_monoLeft, m_monoRight;
    std::shared_ptr<dai::node::StereoDepth> m_stereoDepth;

    std::shared_ptr<dai::node::ColorCamera> m_colorMain;

    std::shared_ptr<dai::node::IMU> m_imu;

    std::shared_ptr<dai::node::FeatureTracker> m_opticalFlowLeft, m_opticalFlowRight;

    std::thread m_run;
    void run();

    // the fuctions below is to setup pipeline before m_device
    void configureRatesWorkaround();
    std::shared_ptr<dai::node::XLinkIn> configureControl();
    void configureImu();
    void configureStereo();
    void configureOpticalFlow();

    // the functions below assumes m_device is properly setup
    void setupControlQueue(std::shared_ptr<dai::node::XLinkIn>);
    void setupImuQueue();
    void setupStereoQueue();
    void setupOpticalFlowQueue();

    void depthCallback(std::shared_ptr<dai::ADatatype> data);
    void imuCallback(std::shared_ptr<dai::ADatatype> data);
    void opticalFlowPassCallback(std::shared_ptr<dai::ADatatype> data, int camId);

    sensor_msgs::CameraInfo getCameraInfo(std::shared_ptr<dai::ImgFrame> img, dai::CameraBoardSocket socket); // In Oak convention, right camera is the main camera

    // ROS related functionalities
    ros::NodeHandle m_nh;
    std::shared_ptr<image_transport::ImageTransport> m_imageTransport;
    std::shared_ptr<image_transport::CameraPublisher> m_leftPub, m_rightPub;
    std::shared_ptr<ros::Publisher> m_imuPub;

};