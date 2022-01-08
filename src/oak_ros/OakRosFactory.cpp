#include <oak_ros/OakRosFactory.hpp>

#include "OakRos.hpp"

OakRosInterface::Ptr OakRosFactory::getOakRosHandler()
{
    return std::make_shared<OakRos>();
}