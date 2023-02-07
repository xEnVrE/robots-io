/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_SPATIALVELOCITYBUFFER_H
#define ROBOTSIO_SPATIALVELOCITYBUFFER_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/SpatialVelocity.h>

namespace RobotsIO {
    namespace Utils {
        class SpatialVelocityBuffer;
    }
}


class RobotsIO::Utils::SpatialVelocityBuffer : public RobotsIO::Utils::SpatialVelocity
{
public:
    SpatialVelocityBuffer();

    virtual ~SpatialVelocityBuffer();

    bool freeze(const bool blocking) override;

    void set_twist(const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const double& elapsed_time = 0.0);

    double elapsed_time() override;

protected:
    Eigen::VectorXd twist() override;

    Eigen::VectorXd twist_;

    double elapsed_time_;
};

#endif /* ROBOTSIO_SPATIALVELOCITYBUFFER_H */
