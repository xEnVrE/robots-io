/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_YARPBUFFEREDPORT_H
#define ROBOTSIO_YARPBUFFEREDPORT_H

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>

#include <string>
#include <unordered_map>

namespace RobotsIO {
    namespace Utils {
        template<class T>
        class YarpBufferedPort;
    }
}


template<class T>
class RobotsIO::Utils::YarpBufferedPort
{
public:
    YarpBufferedPort(const std::string& port_name);

    virtual ~YarpBufferedPort();

    void send_data(const T& data);

    T* receive_data(const bool& blocking);

    double time_stamp();

    std::size_t flush();

protected:
    yarp::os::Network yarp_;

    yarp::os::BufferedPort<T> port_;

    const std::string log_name_ = "YarpBufferedPort";
};


template<class T>
RobotsIO::Utils::YarpBufferedPort<T>::YarpBufferedPort(const std::string& port_name)
{
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP network is not available."));
    }

    if(!port_.open(port_name))
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot open port " + port_name + "."));
    }
}


template <class T>
RobotsIO::Utils::YarpBufferedPort<T>::~YarpBufferedPort()
{
    if(!(port_.isClosed()))
        port_.close();
}


template <class T>
void RobotsIO::Utils::YarpBufferedPort<T>::send_data(const T& data)
{
    T& data_to_be_sent = port_.prepare();

    data_to_be_sent = data;

    port_.write();
}


template <class T>
T* RobotsIO::Utils::YarpBufferedPort<T>::receive_data(const bool& blocking)
{
    return port_.read(blocking);
}


template <class T>
double RobotsIO::Utils::YarpBufferedPort<T>::time_stamp()
{
    yarp::os::Stamp stamp;
    port_.getEnvelope(stamp);

    return stamp.getTime();
}


template <class T>
std::size_t RobotsIO::Utils::YarpBufferedPort<T>::flush()
{
    std::size_t pending_reads = port_.getPendingReads();
    for (std::size_t i = 0; i < pending_reads; i++)
        port_.read(false);

    return pending_reads;
}

#endif /* ROBOTSIO_YARPBUFFEREDPORT_H */
