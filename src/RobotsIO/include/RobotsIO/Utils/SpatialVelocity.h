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
    /**
     * @param blocking, whether the reading has to be blocking or not
     * @return true if the reading was successful and false otherwise
     *         the position of a point on the screw axis in the camera frame
     *         the velocity of a point on the screw axis in the camera frame
     *         the angular velocity expressed in the camera frame
     */
    virtual std::tuple<bool, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, double> velocity(const bool& blocking = false) = 0;
};

#endif /* ROBOTSIO_SPATIALVELOCITY_H */
