#pragma once
#include "config.hpp"
#include "hole/hole.hpp"
#include "pipe/pipe.hpp"
#include "utils.hpp"
#include <yaml-cpp/yaml.h>

class InputParser {

    YAML::Node root_node;
    string input_filename_;

  public:
    InputParser(const string &input_filename);
    ~InputParser();

    Config create_config(bool gui_enabled) const;

    Hole create_hole(Config &config, ArenaBump &arena_h) const;
    Pipe create_pipe(Config &config, vector<PipeComponent> &pipe_assembly, PipeRenderComponents *pipe_render_components,
                     ArenaBump &arena_h) const;
    void parse_drill_pipe_component(const Config &config, const YAML::Node &component_node,
                                    PipeComponent &pipe_component) const;
    void parse_mwd_component(const Config &config, const YAML::Node &component_node,
                             PipeComponent &pipe_component) const;
    void parse_stabilizer_component(const Config &config, const YAML::Node &component_node,
                                    PipeComponent &pipe_component) const;
    void parse_sub_component(const Config &config, const YAML::Node &component_node,
                             PipeComponent &pipe_component) const;
    void parse_casing_component(const Config &config, const YAML::Node &component_node,
                                PipeComponent &pipe_component) const;
    void parse_steering_component(const Config &config, const YAML::Node &component_node, PipeComponent &pipe_component,
                                  bool &steering_already_defined) const;

    void parse_centralizers(Config &config, const vector<PipeComponent> &pipe_assembly,
                            vector<Centralizer> &centralizers, scalar L_pipe) const;

    void validate_pipe_component_keys(const YAML::Node &component_node, const set<string> &type_keys) const;

  private:
    Config parse_yaml_config_options(bool gui_enabled) const;
};