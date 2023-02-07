/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_FILETOEIGEN_H
#define ROBOTSIO_FILETOEIGEN_H

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        std::pair<bool, Eigen::MatrixXd> file_to_eigen(const std::string& file_path, const std::size_t& skip_rows, const std::size_t skip_cols, const std::size_t& expected_cols);
    }
}

#endif /* ROBOTSIO_FILETOEIGEN_H */
