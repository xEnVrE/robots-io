/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/FileToEigen.h>

#include <iostream>
#include <fstream>
#include <vector>

using namespace Eigen;


std::pair<bool, Eigen::MatrixXd> RobotsIO::Utils::file_to_eigen(const std::string& file_path, const std::size_t& skip_rows, const std::size_t skip_cols, const std::size_t& expected_cols)
{
    MatrixXd data;

    std::ifstream istrm(file_path);
    if (!istrm.is_open())
    {
        std::cerr << "RobotsIO::Utils::file_to_eigen. Error: failed to open " << file_path << std::endl;

        return std::make_pair(false, MatrixXd());
    }

    std::vector<std::string> istrm_strings;
    std::string line;
    {
        std::size_t rows = -1;
        while (std::getline(istrm, line))
        {
            rows++;

            if (rows < skip_rows)
                continue;

            istrm_strings.push_back(line);
        }
    }

    istrm.close();

    data.resize(expected_cols, istrm_strings.size());
    std::size_t found_lines = 0;
    for (auto line : istrm_strings)
    {
        std::size_t cols = -1;
        std::size_t found_fields = 0;
        std::string number_str;
        std::istringstream iss(line);

        while (iss >> number_str)
        {
            cols++;

            if (cols < skip_cols)
                continue;

            if ((found_fields + 1) > expected_cols)
            {
                std::cerr << "RobotsIO::Utils::file_to_eigen. Error: malformed input file " << file_path << "." << std::endl
                          << "Detailed error: found more columns (" << found_fields + 1 << ") than expected (" << expected_cols << ")." << std::endl;

                return std::make_pair(false, MatrixXd());
            }

            try
            {
                std::size_t index = (expected_cols * found_lines) + found_fields;
                *(data.data() + index) = std::stod(number_str);
            }
            catch (std::invalid_argument)
            {
                std::cerr << "RobotsIO::Utils::file_to_eigen. Error: malformed input file " << file_path << "." << std::endl
                          << "Detailed error: data cannot be interpreted as double "
                          << "at line " + std::to_string(skip_cols + found_lines) << ", "
                          << "token " + std::to_string(found_fields) << std::endl;

                return std::make_pair(false, MatrixXd());
            }

            found_fields++;
        }

        if (found_fields != expected_cols)
        {
            std::cerr << "RobotsIO::Utils::read_data_from_file. Error: malformed input file " << file_path << std::endl
                      << "Detailed error: found less columns (" << found_fields << ") than expected (" << expected_cols << ")." << std::endl;

            return std::make_pair(false, MatrixXd());
        }
        found_lines++;
    }

    return std::make_pair(true, data);
}
