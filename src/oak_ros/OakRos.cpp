#include "OakRos.hpp"

#include <chrono>

#include <cv_bridge/cv_bridge.h>

void OakRos::init(const ros::NodeHandle &nh, const OakRosParams &params)
{
    spdlog::info("initialising device {}", params.device_id);

    m_device_id = params.device_id;
    m_topic_name = params.topic_name;
    m_stereo_is_rectified = false;

    auto xoutLeft = m_pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = m_pipeline.create<dai::node::XLinkOut>();
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    // auto xoutDisp = m_pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = m_pipeline.create<dai::node::XLinkOut>();
    xoutDepth->setStreamName("depth");
    // auto xoutRectifL = m_pipeline.create<dai::node::XLinkOut>();
    // auto xoutRectifR = m_pipeline.create<dai::node::XLinkOut>();

    // auto xoutColor = m_pipeline.create<dai::node::XLinkOut>();
    // auto colorMain = m_pipeline.create<dai::node::ColorCamera>();

    // configure the stereo sensors' format
    auto stereoDepth = m_pipeline.create<dai::node::StereoDepth>();
    auto monoLeft = m_pipeline.create<dai::node::MonoCamera>();
    auto monoRight = m_pipeline.create<dai::node::MonoCamera>();


    // ROS-related
    m_imageTransport = std::make_shared<image_transport::ImageTransport>(nh);

    if (params.enable_stereo || params.enable_depth)
    {
        
        monoLeft->setResolution(params.stereo_resolution);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight->setResolution(params.stereo_resolution);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

        // direct link from sensor to output
        if (params.enable_stereo && !params.enable_depth)
        {
            spdlog::info("enabling both only raw stereo...");
            monoLeft->out.link(xoutLeft->input);
            monoRight->out.link(xoutRight->input);

        } // sensor to stereo unit before going to output
        else if (params.enable_depth)
        {

            stereoDepth->depth.link(xoutDepth->input);

            if (params.enable_stereo)
            {
                spdlog::info("enabling both depth and stereo streams...");
                stereoDepth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
                stereoDepth->setRectifyEdgeFillColor(0); // black, to better see the cutout
                // stereoDepth->setInputResolution(1280, 720);
                stereoDepth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
                stereoDepth->setLeftRightCheck(true);
                stereoDepth->setExtendedDisparity(false);
                stereoDepth->setSubpixel(false);

                // Linking
                monoLeft->out.link(stereoDepth->left);
                monoRight->out.link(stereoDepth->right);

                if (!params.enable_stereo_rectified)
                {
                    // output raw images
                    stereoDepth->syncedLeft.link(xoutLeft->input);
                    stereoDepth->syncedRight.link(xoutRight->input);
                }
                else
                {
                    // output rectified images
                    stereoDepth->rectifiedLeft.link(xoutLeft->input);
                    stereoDepth->rectifiedRight.link(xoutRight->input);
                    m_stereo_is_rectified = true;
                }
            }
            else
            {
                throw std::runtime_error("not implemented for enabled depth, but disabled stereo");
            }
        }
    }

    // if (params.enable_rgb)
    // {
    //     xoutColor->setStreamName("rgb");
    // }

    // alway try to see if IMU stream is there
    {  
        auto imu = m_pipeline.create<dai::node::IMU>();
        auto xoutIMU = m_pipeline.create<dai::node::XLinkOut>();
        xoutIMU->setStreamName("imu");

        // enable ACCELEROMETER_RAW and GYROSCOPE_RAW at 500 hz rate
        imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 500);
        // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
        imu->setBatchReportThreshold(1);
        // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
        // if lower or equal to batchReportThreshold then the sending is always blocking on device
        // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
        imu->setMaxBatchReports(10);

        // Link plugins IMU -> XLINK
        imu->out.link(xoutIMU->input);
        
    }

    

    if (m_device_id.empty())
    {
        spdlog::info("Creating device without specific id");
        m_device = std::make_shared<dai::Device>(m_pipeline);
    }else
    {
        m_device = std::make_shared<dai::Device>(m_pipeline, getDeviceInfo(m_device_id));
    }
    

    spdlog::info("device created with speed {}", m_device->getUsbSpeed());

    
    if (params.enable_stereo)
    {
        m_leftQueue = m_device->getOutputQueue("left", 8, false);
        m_rightQueue = m_device->getOutputQueue("right", 8, false);

        spdlog::info("advertising stereo cameras in ros topics...");
        m_leftPub.reset(new auto(m_imageTransport->advertiseCamera(m_topic_name + "/left/image_rect_raw", 3)));
        m_rightPub.reset(new auto(m_imageTransport->advertiseCamera(m_topic_name + "/right/image_rect_raw", 3)));
    }

    if (params.enable_depth)
    {
        m_depthQueue = m_device->getOutputQueue("depth", 8, false);
    }

    {
        m_imuQueue = m_device->getOutputQueue("imu", 50, false);
    }

    m_run = std::thread(&OakRos::run, this);
}

void OakRos::run()
{
    m_running = true;

    spdlog::info("{} OakRos running now", m_device_id);

    dai::DataOutputQueue::CallbackId depthCallbackId, imuCallbackId;
    
    if (m_depthQueue.get())
    {
        spdlog::info("{} adds depth queue callback", m_device_id);
        depthCallbackId = m_depthQueue->addCallback(std::bind(&OakRos::depthCallback, this, std::placeholders::_1));
    }
        

    if (m_imuQueue.get())
    {
        spdlog::info("{} adds imu queue callback", m_device_id);
        imuCallbackId = m_imuQueue->addCallback(std::bind(&OakRos::imuCallback, this, std::placeholders::_1));
    }
        


    cv::Mat leftCvFrame, rightCvFrame;
    sensor_msgs::CameraInfo leftCameraInfo, rightCameraInfo;

    while (m_running)
    {
        // process stereo data
        if (m_leftQueue.get() && m_rightQueue.get())
        {
            unsigned int seqLeft, seqRight;

            std::shared_ptr<dai::ImgFrame> left, right;

            left = m_leftQueue->get<dai::ImgFrame>();
            seqLeft = left->getSequenceNum();

            right = m_rightQueue->get<dai::ImgFrame>();
            seqRight = right->getSequenceNum();
            
            while (seqRight != seqLeft)
            {
                spdlog::warn("sequence number mismatch, skip frame. seqLeft = {}, seqRight = {}", seqLeft, seqRight);

                if (seqRight < seqLeft)
                {
                    right = m_rightQueue->get<dai::ImgFrame>();
                    seqRight = right->getSequenceNum();
                }else
                {
                    left = m_leftQueue->get<dai::ImgFrame>();
                    seqLeft = left->getSequenceNum();
                }
            }

            // Here we have make sure the stereo have the same sequence number. According to OAK, this ensures synchronisation

            // initialise camera_info msgs if needed
            if (leftCameraInfo.width != left->getWidth() || rightCameraInfo.width != right->getWidth())
            {
                spdlog::info("{} Initialise camera_info messages, rectification = {}", m_device_id, m_stereo_is_rectified ? "Enabled" : "Disabled");
                
                if (!m_stereo_is_rectified)
                {
                    leftCameraInfo = getCameraInfo(left, dai::CameraBoardSocket::LEFT);
                    rightCameraInfo = getCameraInfo(right, dai::CameraBoardSocket::RIGHT);
                }else
                {
                    leftCameraInfo = getCameraInfo(right, dai::CameraBoardSocket::RIGHT); // after rectification in OAK, the intrinsics of left will be the same as the right
                    rightCameraInfo = getCameraInfo(right, dai::CameraBoardSocket::RIGHT);
                }
                
            }

            double tsLeft = left->getTimestamp().time_since_epoch().count() / 1.0e9;
            double tsRight = right->getTimestamp().time_since_epoch().count() / 1.0e9;

            spdlog::debug("{} left seq = {}, ts = {}", m_device_id, seqLeft, tsLeft);
            spdlog::debug("{} right seq = {}, ts = {}", m_device_id, seqRight, tsRight);

            // publish left frame and camera info
            {
                leftCvFrame = left->getFrame();

                leftCameraInfo.header.stamp = ros::Time().fromSec(tsLeft);
                cv_bridge::CvImage leftBridge = cv_bridge::CvImage(leftCameraInfo.header, sensor_msgs::image_encodings::MONO8, leftCvFrame);

                m_leftPub->publish(*leftBridge.toImageMsg(), leftCameraInfo);
            }

            // publish right frame and camera info
            {
                rightCvFrame = right->getFrame();

                rightCameraInfo.header.stamp = ros::Time().fromSec(tsRight);
                cv_bridge::CvImage rightBridge = cv_bridge::CvImage(rightCameraInfo.header, sensor_msgs::image_encodings::MONO8, rightCvFrame);

                m_rightPub->publish(*rightBridge.toImageMsg(), rightCameraInfo);
            }
            

        }else
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

    if (m_depthQueue.get())
        m_depthQueue->removeCallback(depthCallbackId);

    if (m_imuQueue.get())
        m_imuQueue->removeCallback(imuCallbackId);

    spdlog::info("{} OakRos quitting", m_device_id);
}

void OakRos::depthCallback(std::shared_ptr<dai::ADatatype> data)
{
    std::shared_ptr<dai::ImgFrame> depthFrame = std::static_pointer_cast<dai::ImgFrame>(data);

    unsigned int seq = depthFrame->getSequenceNum();
    double ts = depthFrame->getTimestamp().time_since_epoch().count() / 1.0e9;

    spdlog::debug("{} depth seq = {}, ts = {}", m_device_id, seq, ts);
}

void OakRos::imuCallback(std::shared_ptr<dai::ADatatype> data)
{
    std::shared_ptr<dai::IMUData> imuData = std::static_pointer_cast<dai::IMUData>(data);

    auto imuPackets = imuData->packets;
    for(auto& imuPacket : imuPackets) {
        auto& acceleroValues = imuPacket.acceleroMeter;
        auto& gyroValues = imuPacket.gyroscope;

        double acceleroTs = acceleroValues.timestamp.get().time_since_epoch().count() / 1.0e9;
        double gyroTs = gyroValues.timestamp.get().time_since_epoch().count() / 1.0e9;

        spdlog::debug("{} imu accel ts = {}", m_device_id, acceleroTs);
        spdlog::debug("{} imu gyro ts = {}", m_device_id, gyroTs);

        // printf("Accelerometer timestamp: %ld ms\n", static_cast<long>(acceleroTs.time_since_epoch().count()));
        // printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", acceleroValues.x, acceleroValues.y, acceleroValues.z);
        // printf("Gyroscope timestamp: %ld ms\n", static_cast<long>(gyroTs.time_since_epoch().count()));
        // printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);
    }

}

sensor_msgs::CameraInfo OakRos::getCameraInfo(std::shared_ptr<dai::ImgFrame> img, dai::CameraBoardSocket socket)
{
    sensor_msgs::CameraInfo info;

    std::vector<double> flatIntrinsics, distCoeffsDouble;

    dai::CalibrationHandler calibData = m_device->readCalibration();

    std::vector<std::vector<float>> intrinsics, extrinsics;
    intrinsics = calibData.getCameraIntrinsics(socket);
    // extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, socket);

    // fill in K
    flatIntrinsics.resize(9);
    for(int i = 0; i < 3; i++) {
        std::copy(intrinsics[i].begin(), intrinsics[i].end(), flatIntrinsics.begin() + 3 * i);
    }

    std::copy(flatIntrinsics.begin(), flatIntrinsics.end(), info.K.begin());

    // TODO: fill in P

    info.width = static_cast<uint32_t>(img->getWidth());
    info.height = static_cast<uint32_t>(img->getHeight());

    // undistrotion is always done in OAK camera, so distortion coefficient should be zero always
    info.distortion_model = "opencv";

    return info;
}

dai::DeviceInfo OakRos::getDeviceInfo(const std::string& device_id)
{
    auto DeviceInfo_vec = dai::Device::getAllAvailableDevices();

    for (auto &info : DeviceInfo_vec)
    {
        if (info.getMxId() == device_id)
        {
            spdlog::info("found device with specified id {}", device_id);
            return info;
        }
            
    }
    spdlog::error("failed to find device with id {}", device_id);
    throw std::runtime_error("");
}