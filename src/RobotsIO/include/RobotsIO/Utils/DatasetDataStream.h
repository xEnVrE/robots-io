/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DATASETDATASTREAM_H
#define ROBOTSIO_DATASETDATASTREAM_H

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class DatasetDataStream;
    }
}


class RobotsIO::Utils::DatasetDataStream
{
public:
    static const int NoTimeIndex = -1;

    DatasetDataStream(const std::string& file_path, const std::size_t& skip_rows, const std::size_t& skip_cols, const std::size_t& expected_cols, const int rx_time_index = NoTimeIndex, const int tx_time_index = NoTimeIndex);

    virtual ~DatasetDataStream();

    double rx_time();

    double tx_time();

    virtual Eigen::VectorXd data();

    virtual bool freeze();

protected:
    int get_head();

    bool set_head(const int& value);

private:
    Eigen::MatrixXd data_;

    Eigen::VectorXd data_rx_time_;

    Eigen::VectorXd data_tx_time_;

    int rx_time_index_;

    int tx_time_index_;

    int head_ = -1;

    const std::string log_name_ = "DatasetDataStream";
};

#endif /* ROBOTSIO_DATASETDATASTREAM_H */
