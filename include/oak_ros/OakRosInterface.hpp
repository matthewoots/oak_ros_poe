#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>

#include <depthai-shared/properties/MonoCameraProperties.hpp>
#include <depthai-shared/properties/ColorCameraProperties.hpp>
#include <depthai-shared/properties/StereoDepthProperties.hpp>

struct OakRosParams
{
    std::string device_id;
    std::string topic_name = "oak";
    bool enable_stereo = true;
    bool enable_stereo_rectified = true; // if enable_depth is false, then rectification will never happen
    dai::MonoCameraProperties::SensorResolution stereo_resolution = dai::MonoCameraProperties::SensorResolution::THE_480_P;

    bool enable_depth = true;
    bool enable_depth_pointcloud = false;
    // dai::StereoDepthProperties

    bool enable_rgb = false;
    dai::ColorCameraProperties::SensorResolution rgb_resolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
};

class OakRosInterface
{
public:
    typedef std::shared_ptr<OakRosInterface> Ptr;

    virtual void init(const ros::NodeHandle &nh, const OakRosParams &params) = 0;

protected:
};