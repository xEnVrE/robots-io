/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_BOPDATASETDETECTION_H
#define ROBOTSIO_BOPDATASETDETECTION_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/DatasetDataStream.h>
#include <RobotsIO/Utils/Detection.h>

#include <opencv2/opencv.hpp>

#include <vector>

namespace RobotsIO {
    namespace Utils {
        class BOPDatasetDetection;
    }
}


class RobotsIO::Utils::BOPDatasetDetection : public RobotsIO::Utils::Detection
{
public:
    BOPDatasetDetection(const std::string& sequence_root_path, const std::size_t& object_id);

    virtual ~BOPDatasetDetection();

    bool freeze(const bool blocking = false);

    cv::Rect detection() const override;

private:
    const std::size_t object_id_;

    std::vector<cv::Rect> detection_;

    int head_ = -1;

    int number_frames_;

    const std::string log_name_ = "BOPDatasetDetection";
};

#endif /* ROBOTSIO_BOPDATASETDETECTION_H */
