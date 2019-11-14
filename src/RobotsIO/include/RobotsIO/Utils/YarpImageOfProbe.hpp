/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_YARPIMAGEOFPROBE_H
#define ROBOTSIO_YARPIMAGEOFPROBE_H

#include <RobotsIO/Utils/Data.h>
#include <RobotsIO/Utils/Probe.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>
#include <RobotsIO/Utils/any.h>

#include <string>

#include <yarp/cv/Cv.h>
#include <yarp/sig/Image.h>

namespace RobotsIO {
    namespace Utils {
        template <class T>
        class YarpImageOfProbe;
    }
}


template <class T>
class RobotsIO::Utils::YarpImageOfProbe : public RobotsIO::Utils::YarpBufferedPort<yarp::sig::ImageOf<T>>,
                                          public RobotsIO::Utils::Probe
{
public:
    YarpImageOfProbe(const std::string& port_name);

    virtual ~YarpImageOfProbe();

protected:
    void on_new_data() override;

private:
    cv::Mat data_cv_;

    yarp::sig::ImageOf<T> data_;

    const std::string log_name_ = "YarpImageOfProbe";
};


template <class T>
RobotsIO::Utils::YarpImageOfProbe<T>::YarpImageOfProbe(const std::string& port_name) :
    YarpBufferedPort<yarp::sig::ImageOf<T>>(port_name)
{}


template <class T>
RobotsIO::Utils::YarpImageOfProbe<T>::~YarpImageOfProbe()
{}


template <class T>
void RobotsIO::Utils::YarpImageOfProbe<T>::on_new_data()
{
    data_cv_ = RobotsIO::Utils::any_cast<cv::Mat>(get_data());

    data_ = yarp::cv::fromCvMat<T>(data_cv_);

    this->send_data(data_);
}


#endif /* ROBOTSIO_YARPIMAGEOFPROBE_H */
