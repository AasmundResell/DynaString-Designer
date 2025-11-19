#include "yaml-parser.hpp"

using std::cout;
using std::endl;
using std::exception;
using std::map;
using std::pair;
using std::runtime_error;
using std::set;
using std::string;
using std::to_string;
using std::vector;
using Index = uint32_t;
using Scalar = double;

#include <vector>
using std::vector;

enum class TimeIntegration {
    RK4,
    EXPLICIT_EULER,
    IMPLICIT_EULER
};
struct Config {
    Index N;
    Scalar v0;
    Scalar CFL;
    string mesh_type;
    vector<Scalar> nodes;
    TimeIntegration time_int;
};

map<string, TimeIntegration> time_integration_from_string{{"rk4", TimeIntegration::RK4},
                                                          {"explicit_euler", TimeIntegration::EXPLICIT_EULER},
                                                          {"implicit_euler", TimeIntegration::IMPLICIT_EULER}};

struct InputParser : public yaml_parser::YamlParserBase {

    InputParser(const string &input_file) : YamlParserBase(input_file) {}

    Config parse_config() {
        Config config;
        config.v0 = read_required_option<Scalar>("setup/v0");
        config.CFL = read_required_option<Scalar>("setup/CFL", 0.0, 1.0);
        config.N = read_optional_option("setup/N", 50);
        config.mesh_type = read_required_option<string>("setup/mesh/type");
        config.nodes = read_required_option_vector<Scalar>("setup/mesh/nodes");
        config.time_int =
            read_required_enum_option<TimeIntegration>("setup/time_integration", time_integration_from_string);
        return config;
    }
};

int main(int argc, char *argv[]) {
    if (argc != 2) {
        THROW_RUNTIME_ERROR("Specify the yaml input file\n");
    }

    try {
        const string input_file = argv[1];

        InputParser ip{input_file};
        Config config = ip.parse_config();
        ip.report_invalid_options();
        cout << "ho?" << endl;

    } catch (const std::exception &e) {
        cerr << "Test failed: " << e.what() << endl;
        exit(EXIT_FAILURE);
    }
    cout << "Test succeeded\n";
}