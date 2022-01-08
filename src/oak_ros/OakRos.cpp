#include "OakRos.hpp"

#include <spdlog/spdlog.h>

void OakRos::init(ros::NodeHandle &nh, const OakRosParams &params)
{
    spdlog::info("initialising device {}", params.device_id);

    auto xoutLeft = m_pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = m_pipeline.create<dai::node::XLinkOut>();
    // auto xoutDisp = m_pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = m_pipeline.create<dai::node::XLinkOut>();
    // auto xoutRectifL = m_pipeline.create<dai::node::XLinkOut>();
    // auto xoutRectifR = m_pipeline.create<dai::node::XLinkOut>();

    auto xoutColor = m_pipeline.create<dai::node::XLinkOut>();
    auto colorMain = m_pipeline.create<dai::node::ColorCamera>();

    // configure the stereo sensors' format
    if (params.enable_stereo || params.enable_depth)
    {
        auto monoLeft = m_pipeline.create<dai::node::MonoCamera>();
        auto monoRight = m_pipeline.create<dai::node::MonoCamera>();
        
        monoLeft->setResolution(params.stereo_resolution);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight->setResolution(params.stereo_resolution);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

        // direct link from sensor to output
        if (params.enable_stereo && !params.enable_depth)
        {
            xoutLeft->setStreamName("left");
            xoutRight->setStreamName("right");

            monoLeft->out.link(xoutLeft->input);
            monoRight->out.link(xoutRight->input);

        } // sensor to stereo unit before going to output
        else if (params.enable_depth)
        {
            auto stereoDepth = params.enable_depth ? m_pipeline.create<dai::node::StereoDepth>() : nullptr;
            if (params.enable_stereo)
            {
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

    if (params.enable_rgb)
    {
        xoutColor->setStreamName("rgb");
    }

    spdlog::info("creating device");

    dai::Device device = params.device_id.empty() ? dai::Device(m_pipeline) : dai::Device(m_pipeline, getDeviceInfo(params.device_id));

    spdlog::info("device created");

    std::shared_ptr<dai::DataOutputQueue> leftQueue, rightQueue, depthQueue, rgbQueue;
    if (params.enable_stereo)
    {
        leftQueue = device.getOutputQueue("left", 8, false);
        rightQueue = device.getOutputQueue("right", 8, false);
    }

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