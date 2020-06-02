/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_CLOCKYARP_H
#define ROBOTSIO_CLOCKYARP_H

#include <RobotsIO/Utils/Clock.h>

#include <yarp/os/Network.h>

namespace RobotsIO {
    namespace Utils {
        class ClockYarp;
    }
}

class RobotsIO::Utils::ClockYarp : public RobotsIO::Utils::Clock
{
public:
    ClockYarp();

    virtual ~ClockYarp();

    double now() const override;

    void delay(const int& milliseconds) const override;

private:
    yarp::os::Network yarp_;

    const std::string log_name_ = "ClockYarp";
};

#endif /* ROBOTSIO_CLOCKYARP_H */
