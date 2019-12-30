/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_YARPBOTTLEPROBE_H
#define ROBOTSIO_YARPBOTTLEPROBE_H

#include <RobotsIO/Utils/Parameters.h>
#include <RobotsIO/Utils/Probe.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>

#include <yarp/os/Bottle.h>

#include <string>

namespace RobotsIO {
    namespace Utils {
        template <class T = yarp::os::Bottle>
        class YarpBottleProbe;
    }
}


template <class T>
class RobotsIO::Utils::YarpBottleProbe : public RobotsIO::Utils::YarpBufferedPort<yarp::os::Bottle>,
                                         public RobotsIO::Utils::Probe
{
public:
    YarpBottleProbe(const std::string& port_name);

    virtual ~YarpBottleProbe();

protected:
    void on_new_data() override;

private:
    yarp::os::Bottle convert_from(const T& data);

    yarp::os::Bottle data_;

    const std::string log_name_ = "YarpBottleProbe";
};


template <class T>
RobotsIO::Utils::YarpBottleProbe<T>::YarpBottleProbe(const std::string& port_name) :
    YarpBufferedPort<yarp::os::Bottle>(port_name)
{}


template <class T>
RobotsIO::Utils::YarpBottleProbe<T>::~YarpBottleProbe()
{}


template <class T>
void RobotsIO::Utils::YarpBottleProbe<T>::on_new_data()
{
    data_ = convert_from(RobotsIO::Utils::any_cast<T>(get_data()));

    this->send_data(data_);
}


template <class T>
yarp::os::Bottle RobotsIO::Utils::YarpBottleProbe<T>::convert_from(const T& data)
{
    return data;
}


template <>
void RobotsIO::Utils::YarpBottleProbe<RobotsIO::Utils::Parameters>::on_new_data();


template <>
yarp::os::Bottle RobotsIO::Utils::YarpBottleProbe<RobotsIO::Utils::Parameters>::convert_from(const RobotsIO::Utils::Parameters& data);


#endif /* ROBOTSIO_YARPBOTTLEPROBE_H */
