/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_SPATIALVELOCITY_H
#define ROBOTSIO_SPATIALVELOCITY_H

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class SpatialVelocity;
    }
}

class RobotsIO::Utils::SpatialVelocity
{
public:
    virtual std::tuple<bool, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, double> velocity(const bool& blocking = false) = 0;
};

#endif /* ROBOTSIO_SPATIALVELOCITY_H */
