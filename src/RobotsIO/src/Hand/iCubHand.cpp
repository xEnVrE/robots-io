/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Hand/iCubHand.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <deque>
#include <iostream>

using namespace Eigen;
using namespace RobotsIO::Hand;
using namespace iCub::iKin;
using namespace yarp::dev;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;


iCubHand::iCubHand
(
    const std::string& robot_name,
    const std::string& laterality,
    const std::string& port_prefix,
    const std::string& context,
    const bool& use_analogs,
    const std::string& thumb_version
) :
    use_analogs_(use_analogs)
{
    /* Check YARP network. */
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP network is not available."));
    }

    /* Check for laterality. */
    if ((laterality != "right") && (laterality != "left"))
    {
        throw std::runtime_error(log_name_ + "::ctor. Error: invalid laterality" + laterality + ".");
    }

    if (use_analogs_)
    {
        /* Load configuration from config file. */
        ResourceFinder rf;
        rf.setVerbose(true);
        rf.setDefaultConfigFile("icub_hand_configuration.ini");
        rf.setDefaultContext(context);
        rf.configure(0, NULL);

        /* Get inner resource finder according to requested laterality. */
        ResourceFinder inner_rf = rf.findNestedResourceFinder(laterality.c_str());
        bool use_bounds_ = inner_rf.check("use_bounds", Value(false)).asBool();
        if (use_bounds_)
        {
            bool valid_vector;
            yarp::sig::Vector bounds_col_0;
            std::tie(valid_vector, bounds_col_0) = load_vector_double(inner_rf, "bounds_col_0", 16);
            if (!valid_vector)
            {
                throw std::runtime_error(log_name_ + "::ctor. Error: bounds requested but not available in the configuration file.");
            }

            yarp::sig::Vector bounds_col_1;
            std::tie(valid_vector, bounds_col_1) = load_vector_double(inner_rf, "bounds_col_1", 16);
            if (!valid_vector)
            {
                throw std::runtime_error(log_name_ + "::ctor. Error: bounds requested but not available in the configuration file.");
            }

            analog_bounds_.resize(16, 2);
            analog_bounds_.setCol(0, bounds_col_0);
            analog_bounds_.setCol(1, bounds_col_1);
        }

        /* Try to use PolyDriver for analogs. */
        Property prop_analog;
        prop_analog.put("device", "analogsensorclient");
        prop_analog.put("local", "/" + port_prefix + "/" + laterality + "_hand/analog:i");
        prop_analog.put("remote", "/" + robot_name + "/" + laterality + "_hand/analog:o");
        if (drv_analog_.open(prop_analog))
        {
            /* Try to retrieve the view. */
            if (drv_analog_.view(ianalog_) && ianalog_ != nullptr)
                use_interface_analogs_ = true;
        }

        if (!use_interface_analogs_)
        {
            /* If the PolyDriver is not available, use a standard port. */
            std::cout << log_name_ + "::ctor. Info: PolyDriver interface IAnalogSensors not available. Using raw encoders from a port." << std::endl;

            if (!port_analogs_.open("/" + port_prefix + "/" + laterality + "_hand/analog:i"))
            {
                throw std::runtime_error(log_name_ + "::ctor. Error: unable to open port for analog encoders.");
            }
        }
    }

    /* Try to use PolyDriver for encoders. */
    Property prop_encoders;
    prop_encoders.put("device", "remote_controlboard");
    prop_encoders.put("local", "/" + port_prefix + "/" + laterality + "_arm");
    prop_encoders.put("remote", "/" + robot_name + "/" + laterality + "_arm");
    if (drv_arm_.open(prop_encoders))
    {
        /* Try to retrieve the view. */
        if (drv_arm_.view(iarm_) && iarm_ != nullptr)
            use_interface_arm_ = true;

        /* Try to retrieve the control limits view. */
        if (!(drv_arm_.view(ilimits_)) || (ilimits_ == nullptr))
            throw std::runtime_error(log_name_ + "::ctor. Error: unable get view for finger control limits.");
    }

    if (!use_interface_arm_)
    {
        /* If the PolyDriver is not available, use a standard port. */
        std::cout << log_name_ + "::ctor. Info: PolyDriver interface IEncoders not available. Using raw encoders from a port." << std::endl;

        if (!port_arm_.open("/" + port_prefix + "/" + laterality + "_arm"))
        {
            throw std::runtime_error(log_name_ + "::ctor. Error: unable to open port for arm encoders.");
        }
    }

    /* Instantiate iCubFinger-s .*/
    std::string thumb_key = "thumb";
    if (!thumb_version.empty())
        thumb_key += "_" + thumb_version;
    fingers_["thumb"] = iCubFinger(laterality + "_" + thumb_key);
    fingers_["index"] = iCubFinger(laterality + "_index");
    fingers_["middle"] = iCubFinger(laterality + "_middle");
    fingers_["ring"] = iCubFinger(laterality + "_ring");
    fingers_["little"] = iCubFinger(laterality + "_little");

    /* Align joint bounds using those of the real robot. */
    if (ilimits_)
    {
        std::deque<IControlLimits*> limits;
        limits.push_back(ilimits_);
        for (auto& finger : fingers_)
            finger.second.alignJointsBounds(limits);
    }
}


iCubHand::~iCubHand()
{
    if (use_analogs_)
    {
        if (use_interface_analogs_)
            drv_analog_.close();
        else
            port_analogs_.close();
    }

    if (use_interface_arm_)
        drv_arm_.close();
    else
        port_arm_.close();
}


std::pair<bool, std::unordered_map<std::string, Eigen::VectorXd>> iCubHand::encoders(const bool& blocking)
{
    /* Analog encoders. */
    yarp::sig::Vector analogs(15);
    bool outcome_analog = false;

    if (use_analogs_)
    {
        if (use_interface_analogs_)
            outcome_analog = (ianalog_->read(analogs)) == (IAnalogSensor::AS_OK);
        else
        {
            Bottle* bottle_analogs =  port_analogs_.read(blocking);

            if (bottle_analogs != nullptr)
            {
                for (size_t i = 0; i < analogs.size(); i++)
                    analogs(i) = bottle_analogs->get(i).asFloat64();

                outcome_analog = true;
            }
        }
    }

    /* Arm encoders. */
    yarp::sig::Vector encoders(9);

    bool outcome_arm = false;

    {
        yarp::sig::Vector arm(16);

        if (use_interface_arm_)
            outcome_arm = iarm_->getEncoders(arm.data());
        else
        {
            Bottle* bottle_arm =  port_arm_.read(blocking);

            if (bottle_arm != nullptr)
            {
                for (size_t i = 0; i < arm.size(); i++)
                    arm(i) = bottle_arm->get(i).asFloat64();

                outcome_arm = true;
            }
        }

        /* Get only part of arm encoders related to the fingers. */
        toEigen(encoders) = toEigen(arm).segment<9>(7);
    }

    if (use_analogs_)
    {
        if (!(outcome_analog && outcome_arm))
            return std::make_pair(false, std::unordered_map<std::string, VectorXd>());
    }
    else
    {
        if (!outcome_arm)
            return std::make_pair(false, std::unordered_map<std::string, VectorXd>());
    }

    /* Combine arm and analog encoders. */
    std::unordered_map<std::string, VectorXd> output_encoders;
    for (auto& finger : fingers_)
    {
        yarp::sig::Vector chain_joints;
        if (use_analogs_)
        {
            if (use_bounds_)
                finger.second.getChainJoints(encoders, analogs, chain_joints, analog_bounds_);
            else
                finger.second.getChainJoints(encoders, analogs, chain_joints);
        }
        else
            finger.second.getChainJoints(encoders, chain_joints);

        VectorXd chain_joints_eigen = toEigen(chain_joints) * M_PI / 180.0;
        output_encoders[finger.first] = chain_joints_eigen;
    }

    return std::make_pair(true, output_encoders);
}


std::pair<bool, yarp::sig::Vector> iCubHand::load_vector_double(const ResourceFinder& rf, const std::string key, const std::size_t size)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (b->size() != size)
        ok = false;

    if (!ok)
        return std::make_pair(false, yarp::sig::Vector());

    yarp::sig::Vector vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return std::make_pair(false, yarp::sig::Vector());

        if (!item_v.isFloat64())
            return std::make_pair(false, yarp::sig::Vector());

        vector(i) = item_v.asFloat64();
    }

    return std::make_pair(true, vector);
}
