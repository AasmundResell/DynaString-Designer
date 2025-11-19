#include "config.hpp"
#include <filesystem>

path Config::get_current_output_subdir() const {
    return output_dir / to_string(n);
}

const path Config::create_output_subdir() const {
    using namespace std;
    const path output_subdir = get_current_output_subdir();
    if (filesystem::exists(output_subdir)) {
        throw runtime_error{"Subdir already exists: " + output_subdir.string()};
    }
    if (!filesystem::create_directory(output_subdir)) {
        throw runtime_error{"Couldn't create output subdir: " + output_subdir.string()};
    }
    return output_subdir;
}