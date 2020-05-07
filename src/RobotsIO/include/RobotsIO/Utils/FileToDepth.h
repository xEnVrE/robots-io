/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_FILETODEPTH_H
#define ROBOTSIO_FILETODEPTH_H

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        std::pair<bool, Eigen::MatrixXf> file_to_depth(const std::string& file_path);
    }
}

#endif /* ROBOTSIO_FILETODEPTH_H */
