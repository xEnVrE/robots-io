/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_TRANSFORMWITHVELOCITYYARPPORT_H
#define ROBOTSIO_TRANSFORMWITHVELOCITYYARPPORT_H

#include <RobotsIO/Utils/TransformWithVelocity.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>

#include <yarp/sig/Vector.h>

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class TransformWithVelocityYarpPort;
    }
}

class RobotsIO::Utils::TransformWithVelocityYarpPort : public RobotsIO::Utils::TransformWithVelocity,
                                                       public RobotsIO::Utils::YarpBufferedPort<yarp::sig::Vector>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TransformWithVelocityYarpPort(const std::string& port_name);

    virtual ~TransformWithVelocityYarpPort();

    Eigen::Transform<double, 3, Eigen::Affine> transform() override;

    Eigen::Vector3d linear_velocity() override;

    Eigen::Vector3d angular_velocity() override;

    bool freeze(const bool blocking = false) override;

private:
    Eigen::Transform<double, 3, Eigen::Affine> transform_;

    Eigen::Vector3d linear_velocity_;

    Eigen::Vector3d angular_velocity_;
};

#endif /* ROBOTSIO_TRANSFORMWITHVELOCITYYARPPORT_H */
