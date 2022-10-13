/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Parameters2YarpBottle.h>

using namespace RobotsIO::Utils;
using namespace yarp::os;


Parameters2YarpBottle::Parameters2YarpBottle(const Parameters& parameters) :
    ParametersExtractor(parameters)
{}


Parameters2YarpBottle::~Parameters2YarpBottle()
{}


Bottle Parameters2YarpBottle::extract_to_bottle()
{
    extract_fields();

    return bottle_;
}


void Parameters2YarpBottle::extract_field(const std::string& key, const std::string& value)
{
    bottle_.addString(key);
    bottle_.addString(value);
}


void Parameters2YarpBottle::extract_field(const std::string& key, const std::size_t& value)
{
    bottle_.addString(key);
    bottle_.addInt32(value);
}


void Parameters2YarpBottle::extract_field(const std::string& key, const int& value)
{
    bottle_.addString(key);
    bottle_.addInt32(value);
}


void Parameters2YarpBottle::extract_field(const std::string& key, const double& value)
{
    bottle_.addString(key);
    bottle_.addFloat64(value);
}


void Parameters2YarpBottle::extract_field(const std::string& key, const bool& value)
{
    bottle_.addString(key);
    bottle_.addInt32(value ? 1 : 0);
}
