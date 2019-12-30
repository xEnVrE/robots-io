/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_PARAMETERSYARPPORT_H
#define ROBOTSIO_PARAMETERSYARPPORT_H

#include <RobotsIO/Utils/ParametersFiller.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>

#include <yarp/os/Bottle.h>

#include <string>

namespace RobotsIO {
    namespace Utils {
        class ParametersYarpPort;
    }
}


class RobotsIO::Utils::ParametersYarpPort : public RobotsIO::Utils::YarpBufferedPort<yarp::os::Bottle>,
                                            public RobotsIO::Utils::ParametersFiller
{
public:
    ParametersYarpPort(const std::string& port_name);

    virtual ~ParametersYarpPort();

    bool receive_parameters();

    const std::pair<bool, std::string> fill_string(const std::string& key) const override;

    const std::pair<bool, std::size_t> fill_size_t(const std::string& key) const override;

    const std::pair<bool, int> fill_int(const std::string& key) const override;

    const std::pair<bool, double> fill_double(const std::string& key) const override;

    const std::pair<bool, bool> fill_bool(const std::string& key) const override;

private:
    yarp::os::Bottle last_parameters_;

    bool data_available_ = false;
};

#endif /* ROBOTSIO_YARPBOTTLE2PARAMETERS_H */
