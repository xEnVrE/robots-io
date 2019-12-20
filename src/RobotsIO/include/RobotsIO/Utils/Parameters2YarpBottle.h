/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_PARAMETERS2YARPBOTTLE_H
#define ROBOTSIO_PARAMETERS2YARPBOTTLE_H

#include <RobotsIO/Utils/ParametersExtractor.h>

#include <yarp/os/Bottle.h>


class Parameters2YarpBottle : public RobotsIO::Utils::ParametersExtractor
{
public:
    Parameters2YarpBottle(const RobotsIO::Utils::Parameters& parameters);

    virtual ~Parameters2YarpBottle();

    yarp::os::Bottle extract_to_bottle();

    void extract_field(const std::string& key, const std::string& value) override;

    void extract_field(const std::string& key, const std::size_t& value) override;

    void extract_field(const std::string& key, const int& value) override;

    void extract_field(const std::string& key, const double& value) override;

    void extract_field(const std::string& key, const bool& value) override;

private:
    yarp::os::Bottle bottle_;
};

#endif
