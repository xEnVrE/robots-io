/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Parameters.h>

using namespace RobotsIO::Utils;

#define robots_io_field_getter_impl(type) \
    type Parameters::get_##type(const std::string& name) const \
    { \
        return type##_data_.at(name); \
    } \

#define robots_io_std_field_getter_impl(type) \
    std::type Parameters::get_##type(const std::string& name) const \
    { \
        return type##_data_.at(name); \
    } \

#define robots_io_field_setter_impl(type) \
    void Parameters::set_##type(const std::string& name, const type& value) \
    { \
        type##_data_[name] = value; \
    } \

#define robots_io_std_field_setter_impl(type) \
    void Parameters::set_##type(const std::string& name, const std::type& value) \
    { \
        type##_data_[name] = value; \
    }


robots_io_std_field_getter_impl(string);


robots_io_std_field_getter_impl(size_t);


robots_io_field_getter_impl(double);


robots_io_field_getter_impl(int);


robots_io_field_getter_impl(bool);


robots_io_std_field_setter_impl(string);


robots_io_std_field_setter_impl(size_t);


robots_io_field_setter_impl(double);


robots_io_field_setter_impl(int);


robots_io_field_setter_impl(bool);
