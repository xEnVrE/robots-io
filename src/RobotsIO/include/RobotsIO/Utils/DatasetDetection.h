/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DATASETDETECTION_H
#define ROBOTSIO_DATASETDETECTION_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/DatasetDataStream.h>
#include <RobotsIO/Utils/Detection.h>

#include <opencv2/opencv.hpp>

namespace RobotsIO {
    namespace Utils {
        class DatasetDetection;
    }
}


class RobotsIO::Utils::DatasetDetection : public RobotsIO::Utils::DatasetDataStream,
                                          public RobotsIO::Utils::Detection
{
public:
    DatasetDetection(const std::string& file_path);

    virtual ~DatasetDetection();

    bool freeze(const bool blocking = false) override;

    cv::Rect detection() const override;

private:
    cv::Rect detection_;

    const std::string log_name_ = "DatasetDetection";
};

#endif /* ROBOTSIO_DATASETDETECTION_H */
