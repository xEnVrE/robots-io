/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_CAMERAPARAMETERS_H
#define ROBOTSIO_CAMERAPARAMETERS_H

#include <RobotsIO/Utils/Parameters.h>

namespace RobotsIO {
    namespace Camera {
        struct CameraParameters;
    }
}


class RobotsIO::Camera::CameraParameters : public RobotsIO::Utils::Parameters
{
public:
    CameraParameters();

    virtual ~CameraParameters();

    robots_io_accessor(CameraParameters);

    robots_io_declare_field(CameraParameters, int, width);

    robots_io_declare_field(CameraParameters, int, height);

    robots_io_declare_field(CameraParameters, double, cx);

    robots_io_declare_field(CameraParameters, double, cy);

    robots_io_declare_field(CameraParameters, double, fx);

    robots_io_declare_field(CameraParameters, double, fy);

    robots_io_declare_field(CameraParameters, bool, initialized);
};

#endif /* ROBOTSIO_CAMERAPARAMETERS_H */
