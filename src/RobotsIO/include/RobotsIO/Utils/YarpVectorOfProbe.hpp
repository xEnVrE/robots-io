/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_YARPVECTOROFPROBE_H
#define ROBOTSIO_YARPVECTOROFPROBE_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/Data.h>
#include <RobotsIO/Utils/Probe.h>
#include <RobotsIO/Utils/TransformWithVelocity.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>
#include <RobotsIO/Utils/any.h>

#include <string>

#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>

namespace RobotsIO {
    namespace Utils {
        template <class T, class U = yarp::sig::VectorOf<T>>
        class YarpVectorOfProbe;
    }
}


template <class T, class U>
class RobotsIO::Utils::YarpVectorOfProbe : public RobotsIO::Utils::YarpBufferedPort<yarp::sig::VectorOf<T>>,
                                           public RobotsIO::Utils::Probe
{
public:
    YarpVectorOfProbe(const std::string& port_name);

    virtual ~YarpVectorOfProbe();

protected:
    void on_new_data() override;

private:
    yarp::sig::VectorOf<T> convert_from(const U& data);

    yarp::sig::VectorOf<T> data_;

    const std::string log_name_ = "YarpVectorOfProbe";
};


template <class T, class U>
RobotsIO::Utils::YarpVectorOfProbe<T, U>::YarpVectorOfProbe(const std::string& port_name) :
    YarpBufferedPort<yarp::sig::VectorOf<T>>(port_name)
{}


template <class T, class U>
RobotsIO::Utils::YarpVectorOfProbe<T, U>::~YarpVectorOfProbe()
{}


template <class T, class U>
void RobotsIO::Utils::YarpVectorOfProbe<T, U>::on_new_data()
{
    data_ = convert_from(RobotsIO::Utils::any_cast<U>(get_data()));

    this->send_data(data_);
}


template <class T, class U>
yarp::sig::VectorOf<T> RobotsIO::Utils::YarpVectorOfProbe<T, U>::convert_from(const U& data)
{
    return data;
}


template <>
yarp::sig::VectorOf<double> RobotsIO::Utils::YarpVectorOfProbe<double, Eigen::VectorXd>::convert_from(const Eigen::VectorXd& data);


template <>
yarp::sig::VectorOf<double> RobotsIO::Utils::YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>::convert_from(const Eigen::Transform<double, 3, Eigen::Affine>& data);


template <>
yarp::sig::VectorOf<double> RobotsIO::Utils::YarpVectorOfProbe<double, RobotsIO::Utils::TransformWithVelocityStorage>::convert_from(const RobotsIO::Utils::TransformWithVelocityStorage& data);

#endif /* ROBOTSIO_YARPVECTOROFPROBE_H */
