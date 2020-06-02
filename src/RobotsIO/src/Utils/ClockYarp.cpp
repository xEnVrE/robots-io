/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/ClockYarp.h>

#include <stdexcept>

#include <yarp/os/Time.h>

using namespace RobotsIO::Utils;


ClockYarp::ClockYarp()
{
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP network is not available."));
    }
}


ClockYarp::~ClockYarp()
{}


double ClockYarp::now() const
{
    return yarp::os::Time::now();
}


void ClockYarp::delay(const int& milliseconds) const
{
    return yarp::os::Time::delay(double(milliseconds) / 1000.0);
}
