#pragma once

#include "OakRosInterface.hpp"

class OakRosFactory
{
public:
    static OakRosInterface::Ptr getOakRosHandler();
};