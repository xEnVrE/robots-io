/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/DepthToFile.h>

#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace Eigen;


bool RobotsIO::Utils::depth_to_file(const std::string& output_path, const cv::Mat& depth)
{
    const std::string log_name = "RobotsIO::Utils::depth_to_file";

    std::FILE* out;

    if ((out = std::fopen(output_path.c_str(), "wb")) == nullptr)
    {
        std::cout << log_name << "Error: cannot open output file " + output_path << std::endl;
        return false;
    }

    /* Write image size. */
    std::size_t dims[2];
    dims[0] = depth.cols;
    dims[1] = depth.rows;
    if (std::fwrite(dims, sizeof(dims), 1, out) != 1)
    {
        std::cout << log_name << "Error: cannot write image size to " + output_path << std::endl;

        fclose(out);

        return false;
    }

    if (std::fwrite(depth.data, sizeof(float), dims[0] * dims[1], out) != dims[0] * dims[1])
    {
        std::cout << log_name << "Error: cannot write image data to " + output_path << std::endl;

        fclose(out);

        return false;
    }

    std::fclose(out);

    return true;
}


bool RobotsIO::Utils::depth_to_file(const std::string& output_path, const MatrixXf& depth)
{
    cv::Mat depth_cv;
    cv::eigen2cv(depth, depth_cv);
    return RobotsIO::Utils::depth_to_file(output_path, depth_cv);
}
