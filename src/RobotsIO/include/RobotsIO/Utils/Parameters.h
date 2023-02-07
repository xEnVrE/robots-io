/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_PARAMETERS_H
#define ROBOTSIO_PARAMETERS_H

#include <RobotsIO/Utils/ParametersExtractor.h>
#include <RobotsIO/Utils/ParametersFiller.h>

#include <string>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace RobotsIO {
    namespace Utils {
        class Parameters;
    }
}

#define robots_io_field_storage(type) \
    std::unordered_map<std::string, type > type##_data_; \

#define robots_io_std_field_storage(type) \
    std::unordered_map<std::string, std::type > type##_data_; \

#define robots_io_field_getter(type) \
    type get_##type(const std::string& name) const; \

#define robots_io_std_field_getter(type) \
    std::type get_##type(const std::string& name) const; \

#define robots_io_field_setter(type) \
    void set_##type(const std::string& name, const type& value); \

#define robots_io_std_field_setter(type) \
    void set_##type(const std::string& name, const std::type& value); \

#define robots_io_declare_field(class_name, type, name) \
    type name() const; \
    \
    void name(const type& value); \
    \
    class Field_##name : public Field \
    { \
        public: \
        void extract_field(const class_name &parameters, RobotsIO::Utils::ParametersExtractor& extractor) const override; \
        \
        bool fill_field(class_name &parameters, const RobotsIO::Utils::ParametersFiller& extractor) override; \
        \
    }; \
    \
    friend class Field_##name;

#define robots_io_declare_std_field(class_name, type, name) \
    std::type name() const; \
    \
    void name(const std::type& value); \
    \
    class Field_##name : public Field \
    { \
        public: \
        void extract_field(const class_name &parameters, RobotsIO::Utils::ParametersExtractor& extractor) const override; \
        \
        bool fill_field(class_name &parameters, const RobotsIO::Utils::ParametersFiller& extractor) override; \
        \
    }; \
    \
    friend class Field_##name;

#define robots_io_accessor(class_name) \
    class_name(const RobotsIO::Utils::ParametersFiller& filler); \
    \
    class Field \
    { \
    public: \
        virtual void extract_field(const class_name& parameters, RobotsIO::Utils::ParametersExtractor& extractor) const = 0; \
        \
        virtual bool fill_field(class_name& parameters, const RobotsIO::Utils::ParametersFiller& filler) = 0; \
    }; \
    \
    std::unordered_map<std::string, Field*> fields_; \
    \
    void extract_field(const std::string& key, RobotsIO::Utils::ParametersExtractor& extractor) const override; \
    \
    bool fill_field(const std::string& key, const RobotsIO::Utils::ParametersFiller& filler) override; \
    \
    std::vector<std::string> keys() const override;

#define robots_io_declare_field_impl(class_name, type, name) \
    type class_name::name() const \
    { \
        return get_##type(#name); \
    } \
    \
    void class_name::name(const type& value) \
    { \
        if (fields_.find(#name) == fields_.end()) \
            fields_[#name] = new Field_##name(); \
        set_##type(#name, value); \
    } \
    \
    void class_name::Field_##name::extract_field(const class_name& parameters, RobotsIO::Utils::ParametersExtractor& extractor) const \
    { \
        extractor.extract_field(#name, parameters.name()); \
    } \
    \
    bool class_name::Field_##name::fill_field(class_name& parameters, const RobotsIO::Utils::ParametersFiller& filler) \
    { \
        bool is_value = false; \
        type value; \
        std::tie(is_value, value) = filler.fill_##type(#name); \
        if (!is_value) \
            return false; \
        \
        parameters.name(value); \
        \
        return true; \
    } \

#define robots_io_declare_std_field_impl(class_name, type, name) \
    std::type class_name::name() const \
    { \
        return get_##type(#name); \
    } \
    \
    void class_name::name(const std::type& value) \
    { \
        if (fields_.find(#name) == fields_.end()) \
            fields_[#name] = new Field_##name(); \
        set_##type(#name, value); \
    } \
    \
    void class_name::Field_##name::extract_field(const class_name& parameters, RobotsIO::Utils::ParametersExtractor& extractor) const \
    { \
        extractor.extract_field(#name, parameters.name()); \
    } \
    \
    bool class_name::Field_##name::fill_field(class_name& parameters, const RobotsIO::Utils::ParametersFiller& filler) \
    { \
        bool is_value = false; \
        std::type value;                                       \
        std::tie(is_value, value) = filler.fill_##type(#name); \
        if (!is_value) \
            return false; \
        \
        parameters.name(value); \
        \
        return true; \
    }

#define robots_io_accessor_impl(class_name) \
    class_name::class_name(const RobotsIO::Utils::ParametersFiller& filler) \
        : class_name() \
    { \
        for (const std::string& key : keys()) \
            if (!fill_field(key, filler)) \
            { \
                throw(std::runtime_error(std::string(#class_name) + "::ctor. Field " + key + " not available in provided filler.")); \
            } \
    } \
    \
    void class_name::extract_field(const std::string& key, RobotsIO::Utils::ParametersExtractor& extractor) const \
    { \
        fields_.at(key)->extract_field(*this, extractor); \
    } \
    \
    bool class_name::fill_field(const std::string& key, const RobotsIO::Utils::ParametersFiller& filler) \
    { \
        return fields_.at(key)->fill_field(*this, filler); \
    } \
    \
    std::vector<std::string> class_name::keys() const\
    { \
        std::vector<std::string> keys; \
        for (const auto& field : fields_) \
            keys.push_back(field.first); \
        return keys; \
    } \

class RobotsIO::Utils::Parameters
{
public:
    /**
     * Field filler from name
     */
    virtual bool fill_field(const std::string& key, const RobotsIO::Utils::ParametersFiller& filler) = 0;

    /**
     * Field extractor from name
     */
    virtual void extract_field(const std::string& key, RobotsIO::Utils::ParametersExtractor& extractor) const = 0;

    /**
     * Field keys accessor
     */
    virtual std::vector<std::string> keys() const = 0;

    /**
     * Pointer to the base class object.
     */
    const Parameters* parameters() const;

protected:
    /**
     * Getters
     */
    robots_io_std_field_getter(string);

    robots_io_std_field_getter(size_t);

    robots_io_field_getter(double);

    robots_io_field_getter(int);

    robots_io_field_getter(bool);

    /**
     * Setters
     */
    robots_io_std_field_setter(string);

    robots_io_std_field_setter(size_t);

    robots_io_field_setter(double);

    robots_io_field_setter(int);

    robots_io_field_setter(bool);

private:
    /**
     * Key-value pairs storage
     */
    robots_io_std_field_storage(string);

    robots_io_std_field_storage(size_t);

    robots_io_field_storage(double);

    robots_io_field_storage(int);

    robots_io_field_storage(bool);
};

#endif /* ROBOTSIO_PARAMETERS_H */
