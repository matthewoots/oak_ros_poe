#pragma once

#include <memory>

#include <ros/ros.h>

struct OakRosParams
{
    bool enable_stereo = true;
    bool enable_depth = false;
    bool enable_depth_pointcloud = false;
    bool enable_rgb = false;
};

class OakRosInterface
{
public:
    typedef std::shared_ptr<OakRosInterface> Ptr;

    virtual void init(ros::NodeHandle& nh, const OakRosParams& params) = 0;

protected:

};