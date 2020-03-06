/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <cstdlib>
#include <iostream>

#include <RobotsIO/Camera/DatasetCamera.h>

using namespace RobotsIO::Camera;


bool parse_size_t (char** argv, const std::size_t& index, const std::string& name, std::size_t& retrieved);


int main(int argc, char** argv)
{
    const std::string log_name = "test_DatasetCamera";

    if (argc != 4)
    {
        std::cerr << "Synopsis: " + log_name + " <dataset_path> <heading_zeros> <index_offset>" << std::endl << std::endl;

        return EXIT_FAILURE;
    }

    const std::string dataset_path{argv[1]};

    std::size_t heading_zeros;
    if (!parse_size_t(argv, 2, "heading_zeros", heading_zeros))
        return EXIT_FAILURE;

    std::size_t index_offset;
    if (!parse_size_t(argv, 3, "index_offset", index_offset))
        return EXIT_FAILURE;

    DatasetCamera dataset(dataset_path, "", "rgb/", "depth/", "txt", "ppm", "float", heading_zeros, index_offset, 0, 0, 0, 0, 0, 0);

    std::vector<double> rgb_time_stamps;
    std::vector<double> depth_time_stamps;
    while (dataset.status())
    {
        dataset.step_frame();
        dataset.rgb(false);
        dataset.depth(false);

        bool valid_stamp;

        double rgb_stamp;
        std::tie(valid_stamp, rgb_stamp) = dataset.time_stamp_rgb();
        if (valid_stamp)
            rgb_time_stamps.push_back(rgb_stamp);

        double depth_stamp;
        std::tie(valid_stamp, depth_stamp) = dataset.time_stamp_depth();
        if (valid_stamp)
            depth_time_stamps.push_back(depth_stamp);
    }

    std::cout << "Collected " << rgb_time_stamps.size() << " rgb stamps." << std::endl;
    std::cout << "Collected " << depth_time_stamps.size() << " depth stamps." << std::endl << std::endl;;
    if (rgb_time_stamps.size() != depth_time_stamps.size())
        return EXIT_FAILURE;

    std::cout << "Stamps are the following:" << std::endl;
    for (std::size_t i = 0; i < rgb_time_stamps.size(); i++)
        std::cout << "(rgb, depth): " << std::fixed << rgb_time_stamps.at(i) << ", " << depth_time_stamps.at(i) << std::endl;
    std::cout << std::endl;

    double rgb_stamps_mean_difference = 0;
    double depth_stamps_mean_difference = 0;
    double mutual_mean_difference = 0;
    for (std::size_t i = 0; i < rgb_time_stamps.size(); i++)
    {
        if (i > 0)
        {
            rgb_stamps_mean_difference += (rgb_time_stamps.at(i) - rgb_time_stamps.at(i - 1));
            depth_stamps_mean_difference += (depth_time_stamps.at(i) - depth_time_stamps.at(i - 1));
        }
        mutual_mean_difference += (std::abs(rgb_time_stamps.at(i) - depth_time_stamps.at(i)));
    }
    rgb_stamps_mean_difference /= (rgb_time_stamps.size() - 1);
    depth_stamps_mean_difference /= (rgb_time_stamps.size() - 1);
    mutual_mean_difference /= rgb_time_stamps.size();

    std::cout << "Mean RGB stamp difference (ms): " << rgb_stamps_mean_difference * 1000.0 << std::endl;
    std::cout << "Mean Depth stamp difference (ms): " << depth_stamps_mean_difference * 1000.0 << std::endl;
    std::cout << "Mean mutual RGB-Depth stamp difference (ms): " << mutual_mean_difference * 1000.0 << std::endl;


    return EXIT_SUCCESS;
}


bool parse_size_t (char** argv, const std::size_t& index, const std::string& name, std::size_t& retrieved)
{
    try
    {
        if (std::stoi(argv[index]) < 0)
            throw(std::invalid_argument(""));
        retrieved = std::stoul(argv[index]);
    }
    catch (std::invalid_argument)
    {
        std::cerr << "Invalid value " << argv[index] << " for parameter <" << name << ">." << std::endl;
        return false;
    }

    return true;
}
