#pragma once

#include <memory>
#include <string>
#include <optional>

#include <ros/ros.h>

#include <depthai-shared/properties/MonoCameraProperties.hpp>
#include <depthai-shared/properties/ColorCameraProperties.hpp>
#include <depthai-shared/properties/StereoDepthProperties.hpp>

/**
 * @brief Oak Device parameters to configure output
 * 
 * @note
 */

struct OakRosParams
{
    std::string device_id;
    std::string topic_name = "oak";
    bool enable_stereo = true;
    bool enable_stereo_rectified = true; // if enable_depth is false, then rectification will never happen
    std::optional<dai::MonoCameraProperties::SensorResolution> stereo_resolution = {}; // dai::MonoCameraProperties::SensorResolution::THE_480_P

    std::optional<float> stereo_fps = {};
    std::optional<int> stereo_fps_throttle = {};

    bool enable_depth = true;
    bool enable_depth_pointcloud = false;
    // dai::StereoDepthProperties

    bool enable_rgb = false;
    bool enable_imu = false;

    std::optional<dai::ColorCameraProperties::SensorResolution> rgb_resolution = {}; // dai::ColorCameraProperties::SensorResolution::THE_1080_P;
};

class OakRosInterface
{
public:
    typedef std::shared_ptr<OakRosInterface> Ptr;

    virtual void init(const ros::NodeHandle &nh, const OakRosParams &params) = 0;

protected:
};