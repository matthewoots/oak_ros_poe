#include "OakRos.hpp"

#include <chrono>

#include <cv_bridge/cv_bridge.h>

void OakRos::init(const ros::NodeHandle &nh, const OakRosParams &params)
{
    spdlog::info("initialising device {}", params.device_id);

    m_device_id = params.device_id;
    m_topic_name = params.topic_name;

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
        leftQueue = m_device->getOutputQueue("left", 8, false);
        rightQueue = m_device->getOutputQueue("right", 8, false);

        spdlog::info("advertising stereo cameras in ros topics...");
        m_leftPub.reset(new auto(m_imageTransport->advertiseCamera(m_topic_name + "/left/image_rect_raw", 3)));
        m_rightPub.reset(new auto(m_imageTransport->advertiseCamera(m_topic_name + "/right/image_rect_raw", 3)));
    }

    if (params.enable_depth)
    {
        depthQueue = m_device->getOutputQueue("depth", 8, false);
    }

    m_run = std::thread(&OakRos::run, this);
}

void OakRos::run()
{
    m_running = true;

    spdlog::info("{} OakRos running now", m_device_id);

    dai::DataOutputQueue::CallbackId depthCallbackId;
    
    if (depthQueue.get())
        depthCallbackId = depthQueue->addCallback(std::bind(&OakRos::depthCallback, this, std::placeholders::_1));

    while (m_running)
    {
        cv::Mat leftCvFrame, rightCvFrame;

        // process stereo data
        if (leftQueue.get() && rightQueue.get())
        {
            unsigned int seqLeft, seqRight;

            std::shared_ptr<dai::ImgFrame> left, right;

            left = leftQueue->get<dai::ImgFrame>();
            seqLeft = left->getSequenceNum();

            right = rightQueue->get<dai::ImgFrame>();
            seqRight = right->getSequenceNum();
            
            while (seqRight != seqLeft)
            {
                spdlog::warn("sequence number mismatch, skip frame. seqLeft = {}, seqRight = {}", seqLeft, seqRight);

                if (seqRight < seqLeft)
                {
                    right = rightQueue->get<dai::ImgFrame>();
                    seqRight = right->getSequenceNum();
                }else
                {
                    left = leftQueue->get<dai::ImgFrame>();
                    seqLeft = left->getSequenceNum();
                }
            }

            double tsLeft = left->getTimestamp().time_since_epoch().count() / 1.0e9;
            double tsRight = right->getTimestamp().time_since_epoch().count() / 1.0e9;

            spdlog::debug("{} left seq = {}, ts = {}", m_device_id, seqLeft, tsLeft);
            spdlog::debug("{} right seq = {}, ts = {}", m_device_id, seqRight, tsRight);

            // publish left frame and camera info
            {
                leftCvFrame = left->getFrame();

                std_msgs::Header header;
                header.stamp = ros::Time().fromSec(tsLeft);

                sensor_msgs::CameraInfo cameraInfo;
                cameraInfo.header = header;

                cameraInfo.height = left->getWidth();
                cameraInfo.width = left->getHeight();

                cameraInfo.distortion_model = "opencv";


                cv_bridge::CvImage leftBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, leftCvFrame);

                m_leftPub->publish(*leftBridge.toImageMsg(), cameraInfo);
            }

            // publish right frame and camera info
            {
                rightCvFrame = right->getFrame();

                std_msgs::Header header;
                header.stamp = ros::Time().fromSec(tsRight);

                sensor_msgs::CameraInfo cameraInfo;
                cameraInfo.header = header;

                cameraInfo.height = right->getWidth();
                cameraInfo.width = right->getHeight();

                cameraInfo.distortion_model = "opencv";


                cv_bridge::CvImage rightBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, rightCvFrame);

                m_rightPub->publish(*rightBridge.toImageMsg(), cameraInfo);
            }
            

        }else
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

    if (depthQueue.get())
        depthQueue->removeCallback(depthCallbackId);

    spdlog::info("{} OakRos quitting", m_device_id);
}

void OakRos::depthCallback(std::shared_ptr<dai::ADatatype> data)
{
    std::shared_ptr<dai::ImgFrame> depthFrame = std::static_pointer_cast<dai::ImgFrame>(data);

    unsigned int seq = depthFrame->getSequenceNum();
    double ts = depthFrame->getTimestamp().time_since_epoch().count() / 1.0e9;

    spdlog::debug("{} depth seq = {}, ts = {}", m_device_id, seq, ts);
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