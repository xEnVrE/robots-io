/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_SPATIALVELOCITY_H
#define ROBOTSIO_SPATIALVELOCITY_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/DataStream.h>

namespace RobotsIO {
    namespace Utils {
        class SpatialVelocity;
    }
}


class RobotsIO::Utils::SpatialVelocity : public RobotsIO::Utils::DataStream
{
public:
    virtual Eigen::Vector3d angular_velocity();

    virtual Eigen::Vector3d linear_velocity_origin();

    virtual Eigen::Vector3d linear_velocity_screw();

    virtual Eigen::Vector3d screw_position();

    virtual double elapsed_time() = 0;

    bool is_screw_degenerate();

protected:
    virtual Eigen::VectorXd twist() = 0;
};

#endif /* ROBOTSIO_SPATIALVELOCITY_H */
