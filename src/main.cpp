#include <oak_ros/OakRosFactory.hpp>

int main(int argc, char **argv)
{
    auto handler = OakRosFactory::getOakRosHandler();

    handler->getAllAvailableDeviceIds();
}