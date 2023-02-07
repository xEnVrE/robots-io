/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DATASETSPATIALVELOCITY_H
#define ROBOTSIO_DATASETSPATIALVELOCITY_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/DatasetDataStream.h>
#include <RobotsIO/Utils/SpatialVelocity.h>

namespace RobotsIO {
    namespace Utils {
        class DatasetSpatialVelocity;
    }
}


class RobotsIO::Utils::DatasetSpatialVelocity : public RobotsIO::Utils::DatasetDataStream,
                                                public RobotsIO::Utils::SpatialVelocity
{
public:
    DatasetSpatialVelocity(const std::string& file_path, const std::size_t& skip_rows, const std::size_t& skip_cols, const std::size_t& expected_cols, const int rx_time_index = RobotsIO::Utils::DatasetDataStream::NoTimeIndex, const int tx_time_index = RobotsIO::Utils::DatasetDataStream::NoTimeIndex);

    virtual ~DatasetSpatialVelocity();

    bool freeze(const bool blocking = false) override;

    double elapsed_time() override;

protected:
    Eigen::VectorXd twist() override;

private:
    double last_time_;

    double elapsed_time_;

    bool last_time_initialized_ = false;

    Eigen::VectorXd twist_;

    const std::string log_name_ = "DatasetSpatialVelocity";
};

#endif /* ROBOTSIO_DATASETSPATIALVELOCITY_H */
