/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
NN *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DATASETTRANSFORM_H
#define ROBOTSIO_DATASETTRANSFORM_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/DatasetDataStream.h>
#include <RobotsIO/Utils/Transform.h>

namespace RobotsIO {
    namespace Utils {
        class DatasetTransform;
    }
}


class RobotsIO::Utils::DatasetTransform : public RobotsIO::Utils::DatasetDataStream,
                                          public RobotsIO::Utils::Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DatasetTransform(const std::string& file_path, const std::size_t& skip_rows, const std::size_t& skip_cols, const std::size_t& expected_cols, const int rx_time_index = RobotsIO::Utils::DatasetDataStream::NoTimeIndex, const int tx_time_index = RobotsIO::Utils::DatasetDataStream::NoTimeIndex);

    virtual ~DatasetTransform();

    Eigen::Transform<double, 3, Eigen::Affine> transform() override;

    bool freeze(const bool blocking = false) override;

private:
    Eigen::Transform<double, 3, Eigen::Affine> transform_;

    const std::string log_name_ = "DatasetTransform";
};

#endif /* ROBOTSIO_DATASETTRANSFORM_H */
