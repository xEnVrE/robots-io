/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_TRANSFORMWITHVELOCITY_H
#define ROBOTSIO_TRANSFORMWITHVELOCITY_H

#include <RobotsIO/Utils/Transform.h>

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class TransformWithVelocity;

        struct TransformWithVelocityStorage;
    }
}

struct RobotsIO::Utils::TransformWithVelocityStorage
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Transform<double, 3, Eigen::Affine> transform;

    Eigen::Vector3d linear_velocity;

    Eigen::Vector3d angular_velocity;
};


class RobotsIO::Utils::TransformWithVelocity : public RobotsIO::Utils::Transform
{
public:
    virtual ~TransformWithVelocity();

    virtual Eigen::Vector3d linear_velocity() = 0;

    virtual Eigen::Vector3d angular_velocity() = 0;
};

#endif /* ROBOTSIO_TRANSFORMWVELOCITY_H */
