#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>

// #include <depthai-shared/properties/MonoCameraProperties.hpp>
// #include <depthai-shared/properties/ColorCameraProperties.hpp>
// #include <depthai-shared/properties/StereoDepthProperties.hpp>

enum class MonoSensorResolution : int32_t { THE_720_P, THE_800_P, THE_400_P, THE_480_P };
enum class ColorSensorResolution : int32_t { THE_1080_P, THE_4_K, THE_12_MP, THE_13_MP };

struct OakRosParams
{
    std::string device_id;
    bool enable_stereo = true;
    bool enable_stereo_rectified = true;
    MonoSensorResolution stereo_resolution = MonoSensorResolution::THE_480_P;

    bool enable_depth = true;
    bool enable_depth_pointcloud = false;
    // dai::StereoDepthProperties

    bool enable_rgb = false;
    ColorSensorResolution rgb_resolution = ColorSensorResolution::THE_1080_P;
};

class OakRosInterface
{
public:
    typedef std::shared_ptr<OakRosInterface> Ptr;

    virtual void init(ros::NodeHandle &nh, const OakRosParams &params) = 0;

    virtual std::vector<std::string> getAllAvailableDeviceIds() = 0;

protected:
};