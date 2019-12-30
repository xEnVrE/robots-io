/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/CameraParameters.h>

using namespace RobotsIO::Camera;

robots_io_accessor_impl(CameraParameters);


robots_io_declare_field_impl(CameraParameters, int, width);


robots_io_declare_field_impl(CameraParameters, int, height);


robots_io_declare_field_impl(CameraParameters, double, cx);


robots_io_declare_field_impl(CameraParameters, double, cy);


robots_io_declare_field_impl(CameraParameters, double, fx);


robots_io_declare_field_impl(CameraParameters, double, fy);


robots_io_declare_field_impl(CameraParameters, bool, initialized);


CameraParameters::CameraParameters()
{
    /* Set default values. */
    width(0);

    height(0);

    cx(0);

    cy(0);

    fx(0);

    fy(0);

    initialized(false);
}


CameraParameters::~CameraParameters()
{}
