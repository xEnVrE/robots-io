/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_ICUBHAND_H
#define ROBOTSIO_ICUBHAND_H

#include <Eigen/Dense>

#include <iCub/iKin/iKinFwd.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <string>
#include <unordered_map>

namespace RobotsIO {
    namespace Hand {
        class iCubHand;
    }
}


class RobotsIO::Hand::iCubHand
{
public:
    iCubHand(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const std::string& context, const bool& use_analogs, const std::string& thumb_version = "");

    virtual ~iCubHand();

    std::pair<bool, std::unordered_map<std::string, Eigen::VectorXd>> encoders(const bool& blocking);

protected:
    std::pair<bool, yarp::sig::Vector> load_vector_double(const yarp::os::ResourceFinder& rf, const std::string key, const std::size_t size);

    yarp::os::Network yarp_;

    const bool use_analogs_ = false;

    /**
     * Indicates whether the PolyDriver interface is available.
     */

    bool use_interface_analogs_ = false;

    bool use_interface_arm_ = false;

    /**
     * To be used if the interface is available.
     */

    yarp::dev::PolyDriver drv_analog_;

    yarp::dev::IAnalogSensor *ianalog_;

    yarp::dev::PolyDriver drv_arm_;

    yarp::dev::IEncoders *iarm_;

    /**
     * To be used if the interface is not available.
     */

    yarp::os::BufferedPort<yarp::os::Bottle> port_analogs_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_arm_;

    /**
     * Instances of iCub::iKin::iCubFinger required to combine arm and analog encoders.
     */

    std::unordered_map<std::string, iCub::iKin::iCubFinger> fingers_;

    /**
     * Optional analog bounds.
     */

    yarp::sig::Matrix analog_bounds_;

    bool use_bounds_ = false;

    /**
     * Log name to be used in messages printed by the class.
     */

    const std::string log_name_ = "iCubHand";
};

#endif /* ROBOTSIO_ICUBHAND_H */
