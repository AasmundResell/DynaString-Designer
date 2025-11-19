#include "sim-tools/yaml-parser.hpp"
#include <filesystem>
#include <sstream>

namespace sito {
YamlParser::YamlParser(const string &input_filename, const set<string> &all_predefined_options)
    : input_filename{input_filename}, all_predefined_options{all_predefined_options} {
    work_dir = std::filesystem::current_path();
    try {
        root_node = YAML::LoadFile(input_filename);
    } catch (const exception &e) {
        if (input_filename.size() > 4) {
            string yml_extension = input_filename.substr(input_filename.size() - 4);
            if (yml_extension != ".yml") {
                THROW_RUNTIME_ERROR("Error loading input file '" + input_filename + "'\n" +
                                    "Don't forget .yml extension in name");
            }
        }

        THROW_RUNTIME_ERROR(
            "Error loading input file '" + input_filename + "', " + string(e.what()) +
            "\nNote that the input file has to be located in the directory the where the program is called from: '" +
            work_dir.string() + "'\n");
    }
    cout << "Input file: " << input_filename << '\n';

    all_parsed_options = read_yaml_file_options_as_strings();
    /*Only check predefined options if it is defined by client code*/
    if (!all_predefined_options.empty()) {
        check_that_parsed_options_matches_predefined_options();
    }
}

void YamlParser::add_requested_option(const string &option_path) {

    auto it = requested_options.find(option_path);

    if (it != requested_options.end()) {
        /*This means that the client code has requested the same option twice*/
        assert(false);
    }
    requested_options.emplace(option_path);
}

string YamlParser::required_option_not_specified_msg(const string &option_path, const string &extra_msg) const {
    string msg = "Required option \'" + option_path + "\' not specified in the input file \'" + input_filename + "\'\n";
    msg += extra_msg.empty() ? "" : extra_msg;
    return msg;
}

void YamlParser::report_unused_options() const {
    // assert(!all_parsed_options.empty() && !requested_options.empty());

    assert(all_parsed_options.size() >= requested_options.size());
    const uint n_unused = all_parsed_options.size() - requested_options.size();

    if (n_unused > 0) {
        cout << "Warning: Unused option" << (n_unused == 1 ? "" : "s") << " found in input file:\n";

        for (const string &option : all_parsed_options) {
            if (requested_options.find(option) == requested_options.end()) {
                cout << "\'" << option << "\'\n";
            }
        }
    }
}

vector<YAML::Node> YamlParser::read_required_yaml_node_vector(const string &option_path, const string &extra_msg) {
    vector<YAML::Node> val;
    if (!lookup_option_val<vector<YAML::Node>>(option_path, &val)) {
        THROW_RUNTIME_ERROR(required_option_not_specified_msg(option_path, extra_msg));
    }
    add_requested_option(option_path);
    return val;
}

// void YamlParser::find_unused_options(const YAML::Node &node, const string &node_path_current,
//                                      set<string> &invalid_options) {
//     assert(node.IsMap());

//     for (const auto &entry : node) {
//         const string key = entry.first.as<string>();
//         assert(node_path_current[0] != '/');
//         const string node_path_updated = node_path_current.empty() ? key : node_path_current + "/" + key;
//         if (entry.second.IsMap()) {
//             find_unused_options(entry.second, node_path_updated, invalid_options);
//         } else {
//             // Leaf node found
//             auto it = comparison_tree.find(node_path_updated);
//             if (it == comparison_tree.end()) {
//                 assert(invalid_options.find(node_path_updated) == invalid_options.end());
//                 invalid_options.emplace(node_path_updated);
//             }
//         }
//     }
// }

/*Helper function*/
static void read_yaml_file_options_as_strings_recursive(const YAML::Node &node, const string &node_path_current,
                                                        vector<string> &all_parsed_options) {
    assert(node.IsMap());
    for (const auto &entry : node) {
        const string key = entry.first.as<string>();
        assert(node_path_current[0] != '/');
        const string node_path_updated = node_path_current.empty() ? key : node_path_current + "/" + key;
        if (entry.second.IsMap()) {
            // Leaf node not yet found
            read_yaml_file_options_as_strings_recursive(entry.second, node_path_updated, all_parsed_options);
        } else {
            // Leaf node found. Adding option
            all_parsed_options.push_back(node_path_updated);
        }
    }
}

vector<string> YamlParser::read_yaml_file_options_as_strings() {
    vector<string> all_parsed_options_;
    read_yaml_file_options_as_strings_recursive(root_node, "", all_parsed_options_);
    if (all_parsed_options_.empty()) {
        THROW_RUNTIME_ERROR("Empty yaml input file");
    }

    /*Check for duplicate options*/
    for (const string &option_path : all_parsed_options_) {
        const uint count = std::count(all_parsed_options_.begin(), all_parsed_options_.end(), option_path);
        assert(count >= 1);
        if (count > 1) {
            THROW_RUNTIME_ERROR("Duplicate option found: \'" + option_path + "\'");
        }
    }

    return all_parsed_options_;
}

void YamlParser::check_that_parsed_options_matches_predefined_options() const {
    assert(!all_predefined_options.empty() && !all_parsed_options.empty());

    for (const string &parsed_option_path : all_parsed_options) {
        if (all_predefined_options.find(parsed_option_path) == all_predefined_options.end()) {
            /*Illegal option found. Check if the option is placed in the wrong category*/

            assert(parsed_option_path[0] != '/' && parsed_option_path.back() != '/');
            const uint option_pos = parsed_option_path.find_last_of('/');
            const string option_name = parsed_option_path.substr(option_pos + 1);
            assert(option_name.find('/') == string::npos); /*Check that there are no '/' in the option*/

            vector<string> possible_option_paths;
            /*Check if the option is present in a different category*/
            for (const string &predefined_option_path : all_predefined_options) {
                const uint predefined_option_pos = predefined_option_path.find_last_of('/');
                const string predefined_option_name = predefined_option_path.substr(predefined_option_pos + 1);
                assert(predefined_option_name.find('/') == string::npos); /*Check that there are no '/' in the option*/
                if (option_name == predefined_option_name) {
                    possible_option_paths.push_back(predefined_option_path);
                }
            }
            string err_msg = "Illegal option \'" + parsed_option_path + " \' found";
            if (possible_option_paths.size() > 0) {
                err_msg += ", possible options paths with the same option name are:\n";
                for (const string &possible_path : possible_option_paths) {
                    err_msg += "\'" + possible_path + "\'\n";
                }
            } else
                err_msg += ".\n";

            THROW_RUNTIME_ERROR(err_msg);
        }
    }
}

// void YamlParser::report_unused_options() const {
//     set<string> invalid_options;
//     find_unused_options(root_node, "", invalid_options);

//     if (!invalid_options.empty()) {
//         string invalid_options_str;
//         for (const auto &option : invalid_options) {
//             invalid_options_str += "\'" + option + "\'\n";
//         }

//         cout << "Warning: Unused option" << (invalid_options.size() == 1 ? "" : "s") << " found in input file:\n"
//              << invalid_options_str;
//     }
// }

Vec3 YamlParser::read_required_option_Vec3(const string &option_path, optional<scalar> min, optional<scalar> max,
                                           const string &extra_msg) {
    vector<scalar> v = read_required_option_vector<scalar>(option_path, min, max, extra_msg);
    assert(v.size() == 3);
    return {v[0], v[1], v[2]};
}

Vec3 YamlParser::read_optional_option_Vec3(const string &option_path, const Vec3 &val_default, optional<scalar> min,
                                           optional<scalar> max, const string &extra_msg) {
    vector<scalar> v_default = {val_default.x(), val_default.y(), val_default.z()};
    vector<scalar> v = read_optional_option_vector(option_path, v_default, min, max, extra_msg);
    return {v[0], v[1], v[2]};
}

pair<bool, YAML::Node> YamlParser::lookup_option_node(const string &option_path) const {

    /*If predefined options are used, check that the option path is defined in predefined options*/
    if (!all_predefined_options.empty() && all_predefined_options.count(option_path) == 0) {
        assert(false); // this is a bug in the client code. Ensure that the option_path is defined in
                       // all_predefined_options
    }

    using namespace std;
    string invalid_option_path;
    stringstream ss(option_path);
    string token;
    constexpr char delimiter = '/';
    assert((option_path[0] != '/') &&
           (option_path[option_path.size() - 1] != '/')); /*Option paths should be on the form path/to/option*/

    YAML::Node node = Clone(root_node);
    while (getline(ss, token, delimiter)) { /*Loop over the whole specified option path until the value is found*/
        if (token == "")
            continue;
        assert(node.IsMap()); // Yaml file might be badly formatted
        if (!node[token]) {
            return {false, YAML::Node{}};
        }

        std::unordered_set<string> keys;
        for (auto it = node.begin(); it != node.end(); it++) {
            string key = it->first.as<string>();

            if (keys.find(key) != keys.end()) {
                invalid_option_path += key;
                THROW_RUNTIME_ERROR("Duplicate option key found: \'" + invalid_option_path + "\'");
            }
            keys.insert(key);
        }
        invalid_option_path += token;
        if (!invalid_option_path.empty())
            invalid_option_path += "/";
        node = node[token];
    }
    if (node.IsNull()) {
        THROW_RUNTIME_ERROR("Option \'" + option_path + "\' has no value");
    }
    return {true, node};
}
bool YamlParser::option_exists(const string &option_path) const {

    auto [exists, node] = lookup_option_node(option_path);
    return exists;
}
} // namespace sito