/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_SPATIALVELOCITYYARPVECTOR_H
#define ROBOTSIO_SPATIALVELOCITYYARPVECTOR_H

#include <RobotsIO/Utils/SpatialVelocity.h>
#include <RobotsIO/Utils/YarpVectorOfProbe.hpp>

#include <Eigen/Dense>

#include <chrono>

namespace RobotsIO {
    namespace Utils {
        class SpatialVelocityYarpPort;
    }
}

class RobotsIO::Utils::SpatialVelocityYarpPort : public RobotsIO::Utils::SpatialVelocity,
                                                 public RobotsIO::Utils::YarpVectorOfProbe<double>
{
public:
    SpatialVelocityYarpPort(const std::string& port_name);

    virtual ~SpatialVelocityYarpPort();

    bool freeze(const bool blocking = false) override;

    double elapsed_time() override;

protected:
    Eigen::VectorXd twist() override;

private:
    std::chrono::steady_clock::time_point last_time_;

    double elapsed_time_;

    bool last_time_initialized_ = false;

    Eigen::VectorXd twist_;
};

#endif /* ROBOTSIO_SPATIALVELOCITYYARPVECTOR_H */
