/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/CameraParameters.h>

using namespace RobotsIO::Camera;


bool CameraParameters::is_initialized() const
{
    return initialized_;
}


void CameraParameters::set_initialized()
{
    initialized_ = true;
}
