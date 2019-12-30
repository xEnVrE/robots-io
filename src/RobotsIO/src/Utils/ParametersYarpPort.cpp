/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/ParametersYarpPort.h>

#include <yarp/os/Value.h>

using namespace RobotsIO::Utils;
using namespace yarp::os;


ParametersYarpPort::ParametersYarpPort(const std::string& port_name) :
    YarpBufferedPort<Bottle>(port_name)
{}


ParametersYarpPort::~ParametersYarpPort()
{}


bool ParametersYarpPort::receive_parameters()
{
    Bottle* bottle = receive_data(false);

    if (bottle == nullptr)
        return false;
    else
    {
        last_parameters_ = *bottle;

        data_available_ = true;

        return true;
    }
}


const std::pair<bool, std::string> ParametersYarpPort::fill_string(const std::string& key) const
{
    if (!data_available_)
        return std::make_pair(false, std::string());

    Value value = last_parameters_.find(key);
    if (value.isNull())
        return std::make_pair(false, std::string());

    return std::make_pair(true, value.asString());
}


const std::pair<bool, std::size_t> ParametersYarpPort::fill_size_t(const std::string& key) const
{
    if (!data_available_)
        return std::make_pair(false, std::size_t());

    Value value = last_parameters_.find(key);
    if (value.isNull())
        return std::make_pair(false, std::size_t());

    return std::make_pair(true, value.asInt());
}


const std::pair<bool, int> ParametersYarpPort::fill_int(const std::string& key) const
{
    if (!data_available_)
        return std::make_pair(false, int());

    Value value = last_parameters_.find(key);
    if (value.isNull())
        return std::make_pair(false, int());

    return std::make_pair(true, value.asInt());
}


const std::pair<bool, double> ParametersYarpPort::fill_double(const std::string& key) const
{
    if (!data_available_)
        return std::make_pair(false, double());

    Value value = last_parameters_.find(key);
    if (value.isNull())
        return std::make_pair(false, double());

    return std::make_pair(true, value.asDouble());
}


const std::pair<bool, bool> ParametersYarpPort::fill_bool(const std::string& key) const
{
    if (!data_available_)
        return std::make_pair(false, bool());

    Value value = last_parameters_.find(key);
    if (value.isNull())
        return std::make_pair(false, bool());

    return std::make_pair(true, value.asBool());
}
