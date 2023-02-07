/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/RealsenseCameraYarp.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Property.h>

using namespace RobotsIO::Camera;
using namespace yarp::dev;
using namespace yarp::os;


RealsenseCameraYarp::RealsenseCameraYarp(const std::string& port_prefix, const std::size_t& width, const std::size_t& height) :
    RealsenseCameraYarp(port_prefix, true, width, height)
{}


RealsenseCameraYarp::RealsenseCameraYarp(const std::string& port_prefix) :
    RealsenseCameraYarp(port_prefix, false)
{}


RealsenseCameraYarp::RealsenseCameraYarp(const std::string& port_prefix, const bool& enforce_resolution, const std::size_t& width, const std::size_t& height) :
    YarpCamera(port_prefix)
{
    /* Extract camera parameters. */
    yarp::dev::PolyDriver driver;
    yarp::dev::IRGBDSensor* interface;

    Property driver_properties;
    driver_properties.put("device", "RGBDSensorClient");
    driver_properties.put("localImagePort",  "/" + port_prefix + "/RGBDSensorClient/image:i");
    driver_properties.put("localDepthPort",  "/" + port_prefix + "/RGBDSensorClient/depth:i");
    driver_properties.put("localRpcPort",    "/" + port_prefix + "/RGBDSensorClient/rpc:i");
    driver_properties.put("remoteImagePort", "/depthCamera/rgbImage:o");
    driver_properties.put("remoteDepthPort", "/depthCamera/depthImage:o");
    driver_properties.put("remoteRpcPort",   "/depthCamera/rpc:i");

    if (driver.open(driver_properties) && driver.view(interface) && (interface != nullptr))
    {
        Property camera_intrinsics;
        interface->getDepthIntrinsicParam(camera_intrinsics);

        std::size_t camera_width = interface->getRgbWidth();
        std::size_t camera_height = interface->getRgbHeight();

        double scaler_x = 1.0;
        double scaler_y = 1.0;
        if (enforce_resolution)
        {
            if ((width > camera_width) || (height > camera_height))
                throw(std::runtime_error(log_name_ + "::ctor. Cannot enforce a resolution higher than the source resolution"));

            scaler_x = width / camera_width;
            scaler_y = height / camera_height;
        }

        parameters_.width(camera_width * scaler_x);
        parameters_.height(camera_height * scaler_y);
        parameters_.fx(camera_intrinsics.find("focalLengthX").asFloat64() * scaler_x);
        parameters_.fy(camera_intrinsics.find("focalLengthY").asFloat64() * scaler_y);
        parameters_.cx(camera_intrinsics.find("principalPointX").asFloat64() * scaler_x);
        parameters_.cy(camera_intrinsics.find("principalPointY").asFloat64() * scaler_y);
        parameters_.initialized(true);

        driver.close();
    }
    else
        throw(std::runtime_error(log_name_ + "::ctor. Cannot get camera parameters."));

    Camera::initialize();

    /* Log parameters. */
    std::cout << log_name_ + "::ctor. Camera parameters:" << std::endl;
    std::cout << log_name_ + "    - width: " << parameters_.width() << std::endl;
    std::cout << log_name_ + "    - height: " << parameters_.height() << std::endl;
    std::cout << log_name_ + "    - fx: " << parameters_.fx() << std::endl;
    std::cout << log_name_ + "    - fy: " << parameters_.fy() << std::endl;
    std::cout << log_name_ + "    - cx: " << parameters_.cx() << std::endl;
    std::cout << log_name_ + "    - cy: " << parameters_.cy() << std::endl;
}


RealsenseCameraYarp::~RealsenseCameraYarp()
{}
