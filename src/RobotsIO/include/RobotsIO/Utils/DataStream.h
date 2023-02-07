/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DATASTREAM_H
#define ROBOTSIO_DATASTREAM_H

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class DataStream;
    }
}

class RobotsIO::Utils::DataStream
{
public:
    virtual ~DataStream();

    virtual bool freeze(const bool blocking = false) = 0;
};

#endif /* ROBOTSIO_DATASTREAM_H */
