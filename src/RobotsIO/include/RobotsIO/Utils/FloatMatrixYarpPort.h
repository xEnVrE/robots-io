/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_FLOATMATRIXYARPPORT_H
#define ROBOTSIO_FLOATMATRIXYARPPORT_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/FloatMatrix.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>

#include <yarp/sig/Image.h>

namespace RobotsIO {
    namespace Utils {
        class FloatMatrixYarpPort;
    }
}


class RobotsIO::Utils::FloatMatrixYarpPort : public RobotsIO::Utils::FloatMatrix,
                                             public RobotsIO::Utils::YarpBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>>
{
public:
    FloatMatrixYarpPort(const std::string& port_name);

    virtual ~FloatMatrixYarpPort();

    bool freeze(const bool blocking = false) override;

    Eigen::MatrixXf matrix() override;

private:
    Eigen::MatrixXf matrix_;
};

#endif /* ROBOTSIO_FLOATMATRIXYARPPORT_H */
