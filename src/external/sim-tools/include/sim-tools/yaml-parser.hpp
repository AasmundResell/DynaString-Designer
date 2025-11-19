#pragma once
#include "../vendor/yaml-cpp-0.8.0/include/yaml-cpp/yaml.h"
#include "includes.hpp"
#include <cassert>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>

using std::cout;
using std::endl;
using std::exception;
using std::map;
using std::nullopt;
using std::optional;
using std::pair;
using std::set;
using std::string;
using std::stringstream;
using std::to_string;
using std::vector;
using std::filesystem::path;
namespace sito {
class YamlParser {

    YAML::Node root_node;
    string input_filename;

    const set<string> all_predefined_options; // List of predefined valid options
    vector<string> all_parsed_options;        /*All options parsed from the yaml file*/

    /*Remember that not depending on the control flow when parsing, not all options will be requested when parsing.
     * These are only the options that are actually requested by the input parser.*/
    set<string> requested_options;

    string required_option_not_specified_msg(const string &option_path, const string &extra_msg = "") const;

  public:
    path work_dir;
    YamlParser(const string &input_filename, const set<string> &all_predefine_options = set<string>{});
    // Can be used to create a new YamlParser from a node when parsing lists
    YamlParser(const YAML::Node &node) { root_node = Clone(node); }

    template <typename T>
    T read_required_option(const string &option_path, optional<T> min = nullopt, optional<T> max = nullopt,
                           const string &extra_msg = "");

    template <typename T>
    T read_optional_option(const string &option_path, T val_default, optional<T> min = nullopt,
                           optional<T> max = nullopt, const string &extra_msg = "");
    template <typename T>
    vector<T> read_required_option_vector(const string &option_path, optional<T> min = nullopt,
                                          optional<T> max = nullopt, const string &extra_msg = "");

    template <typename T>
    vector<T> read_optional_option_vector(const string &option_path, const vector<T> &val_default = vector<T>{},
                                          optional<T> min = nullopt, optional<T> max = nullopt,
                                          const string &extra_msg = "");

    /*A more thoughtful design for reading lists should be implemented*/
    vector<YAML::Node> read_required_yaml_node_vector(const string &option_path, const string &extra_msg = "");
    Vec3 read_required_option_Vec3(const string &option_path, optional<scalar> min = nullopt,
                                   optional<scalar> max = nullopt, const string &extra_msg = "");

    Vec3 read_optional_option_Vec3(const string &option_path, const Vec3 &val_default, optional<scalar> min = nullopt,
                                   optional<scalar> max = nullopt, const string &extra_msg = "");

    template <typename EnumType>
    EnumType read_required_enum_option(const string &option_path, const std::map<string, EnumType> &enum_map);
    template <typename EnumType>
    EnumType read_optional_enum_option(const string &option_path, const std::map<string, EnumType> &enum_map,
                                       EnumType val_default);

    bool option_exists(const string &option_path) const;

    void report_unused_options() const;

  private:
    template <typename T> bool lookup_option_val(const string &option_path, T *val) const;

    //  void find_unused_options(const YAML::Node &node, const string &node_path_current, set<string> &invalid_options);

    template <typename T>
    static void report_value_outside_interval(T val, optional<T> min, optional<T> max, const string &option_path);

    void add_requested_option(const string &option_path);

    template <typename EnumType>
    static EnumType lookup_enum_option_map(const map<string, EnumType> &map, const string &key,
                                           const string &option_path);

    pair<bool, YAML::Node> lookup_option_node(const string &option_path) const;

    /*If all_options are passed into the constructor, this function is called. It checks the correctness
    of the yaml file based on the predefined list of options all_options*/
    void check_that_parsed_options_matches_predefined_options() const;

    /*Reads all options and stores them as "path/to/option" instead of the more abstract format used by yaml-cpp*/
    vector<string> read_yaml_file_options_as_strings();
};

template <typename T> constexpr const char *get_type_name() {
    using std::is_same;
    if (std::is_same<T, bool>::value) {
        return (const char *)"bool";
    } else if (std::is_same<T, int>::value) {
        return (const char *)"int";
    } else if (std::is_same<T, uint>::value) {
        return (const char *)"uint";
    } else if (is_same<T, double>::value) {
        return (const char *)"double";
    } else if (is_same<T, float>::value) {
        return (const char *)"float";
    } else if (is_same<T, string>::value) {
        return (const char *)"string";
    } else if (is_same<T, Vec3>::value)
        return (const char *)"Vec3";
    else {
        return (const char *)"unknown type";
    }
}

template <typename T> bool YamlParser::lookup_option_val(const string &option_path, T *val) const {
    auto [exists, node] = lookup_option_node(option_path);
    if (exists) {

        try {
            *val = node.as<T>();
        } catch (exception &e) {
            const string type_as_string = node.as<string>();
            THROW_RUNTIME_ERROR(string("Couldn't convert option\'" + option_path + ": " + type_as_string + "\' to " +
                                       string(get_type_name<T>())));
        }
        return true;
    } else {
        return false;
    }
}

template <typename T>
T YamlParser::read_required_option(const string &option_path, optional<T> min, optional<T> max,
                                   const string &extra_msg) {
    T val;
    if (!lookup_option_val<T>(option_path, &val)) {
        THROW_RUNTIME_ERROR(required_option_not_specified_msg(option_path, extra_msg));
    }
    add_requested_option(option_path);
    report_value_outside_interval(val, min, max, option_path);
    return val;
}

template <typename T>
T YamlParser::read_optional_option(const string &option_path, T val_default, optional<T> min, optional<T> max,
                                   const string &extra_msg) {
    if (min.has_value()) {
        assert(val_default >= min.value());
    }
    if (max.has_value()) {
        assert(val_default <= max.value());
    }
    T val;
    if (lookup_option_val<T>(option_path, &val)) {
        report_value_outside_interval(val, min, max, option_path);
        add_requested_option(option_path);
        return val;
    } else {
        cout << "Warning: Optional option \'" << option_path + "\' is unspecified, setting default value "
             << val_default << endl;
        return val_default;
    }
}

template <typename T>
vector<T> YamlParser::read_required_option_vector(const string &option_path, optional<T> min, optional<T> max,
                                                  const string &extra_msg) {
    vector<T> val;
    if (!lookup_option_val<vector<T>>(option_path, &val)) {
        THROW_RUNTIME_ERROR(required_option_not_specified_msg(option_path, extra_msg));
    }
    add_requested_option(option_path);
    for (const auto &v_i : val)
        report_value_outside_interval(v_i, min, max, option_path);
    return val;
}

template <typename T>
vector<T> YamlParser::read_optional_option_vector(const string &option_path, const vector<T> &val_default,
                                                  optional<T> min, optional<T> max, const string &extra_msg) {
    vector<T> val;
    if (lookup_option_val<vector<T>>(option_path, &val)) {
        for (const auto &v_i : val)
            report_value_outside_interval(v_i, min, max, option_path);
        add_requested_option(option_path);
        return val;
    } else {
        cout << "Warning: Optional vector option \'" << option_path + "\' is unspecified, setting default value:\n";
        print_std_vector(val);
        return val_default;
    }
}

template <typename EnumType>
EnumType YamlParser::lookup_enum_option_map(const map<string, EnumType> &map, const string &key,
                                            const string &option_path) {
    if (map.count(key) == 1) {
        return map.at(key);
    } else {
        string keys;
        for (const auto &pair : map)
            keys += "'" + pair.first + "'\n";
        THROW_RUNTIME_ERROR("Illegal value '" + key + "' specified for setting '" + option_path +
                            "'. Legal values are:\n" + keys);
    }
}

template <typename EnumType>
EnumType YamlParser::read_required_enum_option(const string &option_path, const map<string, EnumType> &enum_map) {
    string val;
    if (lookup_option_val(option_path, &val)) {
        add_requested_option(option_path);
        EnumType enum_type = lookup_enum_option_map(enum_map, val, option_path);
        return enum_type;
    } else {
        THROW_RUNTIME_ERROR(required_option_not_specified_msg(option_path));
    }
}

template <typename EnumType>
EnumType YamlParser::read_optional_enum_option(const string &option_path, const map<string, EnumType> &enum_map,
                                               EnumType val_default) {
    string val;
    if (lookup_option_val(option_path, &val)) {
        add_requested_option(option_path);
        return lookup_enum_option_map(enum_map, val, option_path);
    } else {
        cout << "Warning: Optional enum option \'" << option_path + "\' is unspecified, setting default value\n"
             << (int)val_default << endl;
        return val_default;
    }
}

template <typename T>
void YamlParser::report_value_outside_interval(T val, optional<T> min, optional<T> max, const string &option_path) {
    static_assert(std::is_arithmetic<T>::value || std::is_same<T, string>::value,
                  "Only primitive values or std::string are allowed");

    if ((min.has_value() && val < min.value()) || (max.has_value() && val > max.value())) {
        stringstream ss;
        ss << "Option \'" << option_path << "\' with value " << val;
        if (min.has_value() && max.has_value()) {
            ss << " must lie between " << min.value() << " and " << max.value();
        } else if (!min.has_value() && max.has_value()) {
            ss << " must be smaller or equal than " << max.value();
        } else {
            assert(min.has_value() && !max.has_value());
            ss << " must be greater or equal than " << min.value();
        }
        THROW_RUNTIME_ERROR(ss.str());
    }
}

} // namespace sito