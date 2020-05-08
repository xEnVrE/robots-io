/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/FileToDepth.h>

#include <Eigen/Dense>

#include <iostream>

using namespace Eigen;


std::pair<bool, Eigen::MatrixXf> RobotsIO::Utils::file_to_depth(const std::string& file_name)
{
    const std::string log_name = "RobotsIO::Utils::file_to_depth";

    std::FILE* in;

    if ((in = std::fopen(file_name.c_str(), "rb")) == nullptr)
    {
        std::cout << log_name << "Error: cannot load depth frame " + file_name << std::endl;
        return std::make_pair(false, MatrixXf());
    }

    /* Load image size .*/
    std::size_t dims[2];
    if (std::fread(dims, sizeof(dims), 1, in) != 1)
    {
        std::cout << log_name << "Error: cannot load depth size for frame " + file_name << std::endl;

        fclose(in);

        return std::make_pair(false, MatrixXf());
    }

    /* Load image. */
    float float_image_raw[dims[0] * dims[1]];
    if (std::fread(float_image_raw, sizeof(float), dims[0] * dims[1], in) != dims[0] * dims[1])
    {
        std::cout << log_name << "Error: cannot load depth data for frame " + file_name << std::endl;

        fclose(in);

        return std::make_pair(false, MatrixXf());
    }

    std::fclose(in);

    /* Store image. */
    MatrixXf float_image(dims[1], dims[0]);
    float_image = Map<Matrix<float, -1, -1, RowMajor>>(float_image_raw, dims[1], dims[0]);

    return std::make_pair(true, float_image);
}
