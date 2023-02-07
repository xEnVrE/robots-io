/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Parameters.h>
#include <RobotsIO/Utils/ParametersExtractor.h>

#include <TestParameters.h>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <string>

class ExtractionTest : public RobotsIO::Utils::ParametersExtractor
{
public:
    ExtractionTest(const RobotsIO::Utils::Parameters& parameters) :
        RobotsIO::Utils::ParametersExtractor(parameters)
    { }

    bool test()
    {
        /* Perform extraction */
        extract_fields();

        /* Check extraction correctness */
        bool ok = true;
        if (field0_ != -1)
        {
            std::cerr << "Expected field_0 to be -1, instead it is " << field0_ << std::endl;
            ok = false;
        }
        if (field1_ != M_PI)
        {
            std::cerr << "Expected field_1 to be M_PI, instead it is " << field1_ << std::endl;
            ok = false;
        }
        if (field2_ != true)
        {
            std::cerr << "Expected field_2 to be true, instead it is " << (field2_ ? "true" : "false") << std::endl;
            ok = false;
        }
        if (field3_ != "This is a string.")
        {
            std::cerr << "Expected field_3 to be 'This is a string.', instead it is " << field3_ << std::endl;
            ok = false;
        }
        if (field4_ != 1)
        {
            std::cerr << "Expected field_4 to be 1, instead it is " << field4_ << std::endl;
            ok = false;
        }

        return ok;
    }

    void extract_field(const std::string& key, const int& value) override
    {
        field0_ = value;
    }

    void extract_field(const std::string& key, const double& value) override
    {
        field1_ = value;
    }

    void extract_field(const std::string& key, const bool& value) override
    {
        field2_ = value;
    }

    void extract_field(const std::string& key, const std::string& value) override
    {
        field3_ = value;
    }

    void extract_field(const std::string& key, const std::size_t& value) override
    {
        field4_ = value;
    }

private:
    int field0_;

    double field1_;

    bool field2_;

    std::string field3_;

    std::size_t field4_;
};


int main(int argc, char** argv)
{
    TestParameters parameters;

    std::cout << "Setting parameters." << std::endl;
    parameters.field0(-1);
    parameters.field1(M_PI);
    parameters.field2(true);
    parameters.field3("This is a string.");
    parameters.field4(1);

    std::cout << "Testing parameters using named accessors." << std::endl;
    if (parameters.field0() != -1)
    {
        std::cerr << "Expected field_0 to be -1, instead it is " << parameters.field0() << std::endl;
        return EXIT_FAILURE;
    }
    if (parameters.field1() != M_PI)
    {
        std::cerr << "Expected field_1 to be M_PI, instead it is " << parameters.field1() << std::endl;
        return EXIT_FAILURE;
    }
    if (parameters.field2() != true)
    {
        std::cerr << "Expected field_2 to be true, instead it is " << (parameters.field2() ? "true" : "false") << std::endl;
        return EXIT_FAILURE;
    }
    if (parameters.field3() != "This is a string.")
    {
        std::cerr << "Expected field_3 to be 'This is a string.', instead it is " << parameters.field3() << std::endl;
        return EXIT_FAILURE;
    }
    if (parameters.field4() != 1)
    {
        std::cerr << "Expected field_4 to be 1, instead it is " << parameters.field4() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Testing parameters using extractor." << std::endl;
    ExtractionTest extraction_test(parameters);
    if (!(extraction_test.test()))
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
