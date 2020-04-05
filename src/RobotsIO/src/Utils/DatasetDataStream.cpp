/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/DatasetDataStream.h>
#include <RobotsIO/Utils/FileToEigen.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


DatasetDataStream::DatasetDataStream(const std::string& file_path, const std::size_t& skip_rows, const std::size_t& skip_cols, const std::size_t& expected_cols, const int rx_time_index, const int tx_time_index)
{
    bool valid_file;
    MatrixXd data_all;
    std::tie(valid_file, data_all) = file_to_eigen(file_path, skip_rows, skip_cols, expected_cols);

    if (!valid_file)
        throw(std::runtime_error(log_name_ + "::ctor. Error cannot read data from file " + file_path + "."));

    if (rx_time_index >= data_all.cols())
        throw(std::runtime_error(log_name_ + "::ctor. Specified rx time index " + std::to_string(rx_time_index) + " is out of range."));

    if (tx_time_index >= data_all.cols())
        throw(std::runtime_error(log_name_ + "::ctor. Specified tx time index " + std::to_string(tx_time_index) + " is out of range."));

    std::size_t data_time_rows = 0;
    if (rx_time_index != -1)
    {
        data_rx_time_ = data_all.row(rx_time_index);
        data_time_rows++;
    }

    if (tx_time_index != -1)
    {
        data_tx_time_ = data_all.row(tx_time_index);
        data_time_rows++;
    }

    data_.resize(data_all.rows() - data_time_rows, data_all.cols());
    std::size_t j = 0;
    for (std::size_t i = 0; i < data_all.rows(); i++)
    {
        if ((i == rx_time_index) || (i == tx_time_index))
            continue;
        data_.row(j) = data_all.row(i);

        j++;
    }
}


DatasetDataStream::~DatasetDataStream()
{}


double DatasetDataStream::rx_time()
{
    if (data_rx_time_.size() != 0)
        return data_rx_time_(get_head());

    return 0.0;
}


double DatasetDataStream::tx_time()
{
    if (data_tx_time_.size() != 0)
        return data_tx_time_(get_head());

    return 0.0;
}


VectorXd DatasetDataStream::data()
{
    return data_.col(get_head());
}


bool DatasetDataStream::freeze()
{
    return set_head(get_head() + 1);
}


int DatasetDataStream::get_head()
{
    return head_;
}


bool DatasetDataStream::set_head(const int& value)
{
    if (value >= data_.cols())
        return false;

    head_ = value;

    return true;
}


VectorXd DatasetDataStream::data(const int& index)
{
    if (index < 0 || index >= data_.cols())
        throw(std::runtime_error(log_name_ + "::data(const int& index). Error: invalid index provided (index = " + std::to_string(index) + ")."));

    return data_.col(index);
}
