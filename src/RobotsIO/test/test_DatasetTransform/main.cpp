/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <cstdlib>
#include <iostream>

#include <RobotsIO/Utils/DatasetTransform.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


bool parse_size_t (char** argv, const std::size_t& index, const std::string& name, std::size_t& retrieved);


int main(int argc, char** argv)
{
    const std::string log_name = "test_DatasetTransform";

    if (argc != 4)
    {
        std::cerr << "Synopsis: " + log_name + " <dataset_path> <skip_rows> <skip_cols>" << std::endl << std::endl;

        return EXIT_FAILURE;
    }

    const std::string dataset_path{argv[1]};

    std::size_t skip_rows;
    if (!parse_size_t(argv, 2, "skip_rows", skip_rows))
        return EXIT_FAILURE;

    std::size_t skip_cols;
    if (!parse_size_t(argv, 3, "skip_cols", skip_cols))
        return EXIT_FAILURE;

    DatasetTransform dataset(dataset_path, skip_rows, skip_cols, 7);

    std::size_t i = 0;
    while (dataset.freeze())
    {
        auto transform = dataset.transform();
        auto rotation = AngleAxisd(transform.rotation());
        std::cout << i++ << ": "
                  << transform.translation().transpose() << " "
                  << rotation.axis().transpose() << " " << rotation.angle()
                  << std::endl;
    }

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
