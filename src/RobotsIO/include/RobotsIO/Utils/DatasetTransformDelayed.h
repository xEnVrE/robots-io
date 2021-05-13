/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
NN *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DATASETTRANSFORMDELAYED_H
#define ROBOTSIO_DATASETTRANSFORMDELAYED_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/DatasetDataStreamDelayed.h>
#include <RobotsIO/Utils/Transform.h>

namespace RobotsIO {
    namespace Utils {
        class DatasetTransformDelayed;
    }
}


class RobotsIO::Utils::DatasetTransformDelayed : public RobotsIO::Utils::DatasetDataStreamDelayed,
                                                 public RobotsIO::Utils::Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DatasetTransformDelayed(const double& fps, const double& simulated_fps, const bool simulate_inference_time, const std::string& file_path, const std::size_t& skip_rows, const std::size_t& skip_cols, const std::size_t& expected_cols, const int rx_time_index = RobotsIO::Utils::DatasetDataStream::NoTimeIndex, const int tx_time_index = RobotsIO::Utils::DatasetDataStream::NoTimeIndex);

    virtual ~DatasetTransformDelayed();

    Eigen::Transform<double, 3, Eigen::Affine> transform() override;

    bool freeze(const bool blocking = false) override;

    int get_frames_between_iterations() const override;

private:
    Eigen::Transform<double, 3, Eigen::Affine> transform_;

    double fps_;

    double simulated_fps_;

    const std::string log_name_ = "DatasetTransformDelayed";
};

#endif /* ROBOTSIO_DATASETTRANSFORMDELAYED_H */
