/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DEPTHTOFILE_H
#define ROBOTSIO_DEPTHTOFILE_H

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

namespace RobotsIO {
    namespace Utils {
        bool depth_to_file(const std::string& output_path, const cv::Mat& depth);

        bool depth_to_file(const std::string& output_path, const Eigen::MatrixXf& depth);
    }
}

#endif /* ROBOTSIO_DEPTHTOFILE_H */
