/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_REALSENSECAMERAYARP_H
#define ROBOTSIO_REALSENSECAMERAYARP_H

#include <RobotsIO/Camera/YarpCamera.h>

#include <string>

namespace RobotsIO {
    namespace Camera {
        class RealsenseCameraYarp;
    }
}

class RobotsIO::Camera::RealsenseCameraYarp : public RobotsIO::Camera::YarpCamera
{
public:
    RealsenseCameraYarp(const std::string& port_prefix);

    ~RealsenseCameraYarp();

private:
    /**
     * Log name to be used in messages printed by the class.
     */

    const std::string log_name_ = "RealsenseCameraYarp";
};

#endif /* ROBOTSIO_REALSENSECAMERAYARP_H */
