#pragma once
#include "gui/graphics/includes-graphics.hpp"
#include "gui/gui-manager-base.hpp"
#include "gui/plot-utils.hpp"
#include "gui/ui/graph-utils.hpp"
#include "imgui-manager.hpp"
#include "misc/config.hpp"

class ImGuiFileChooser;

enum class SettingsViewMode {
    NONE = -1,
    HOME = 0,
    PHYSICS_SETTINGS,
    GRAPHICS_SETTINGS,
    GRAPH_PLOTS_SETTINGS,
    COUNT
};

enum class PlotTileType {
    ORBITAL_PLOT,
    BIT_ROCK_PLOT,
    SPATIAL_GRAPH_PLOT,
    MISC_SPATIAL_GRAPH_PLOT,
    FLUID_SPATIAL_GRAPH_PLOT,
    KEY_QUANTITIES_PLOT
};

struct PlotTile {
    PlotTileType type;
    string title;
    uint width_tiles = 1;  // Number of tiles horizontally
    uint height_tiles = 1; // Number of tiles vertically
    bool visible = true;

    // Position in the grid (calculated automatically)
    uint grid_row = 0;
    uint grid_col = 0;

    // For specific plot types
    uint orbital_plot_index = 0;  // Which orbital plot (if applicable)
    uint spatial_graph_index = 0; // Which spatial graph (if applicable)
    uint plot_group_index = 0;    // Which plot group (if applicable)
};

struct UI_settings {
    bool show_walls = false;
    bool show_bit_rock_plot = false;
    bool pipe_fully_fed_message = false;
    bool pipe_fully_pulled_message = false;
    float slider_S_orbital{0.0f};
    SettingsViewMode current_settings_mode = SettingsViewMode::NONE;
};

static const std::array<const char *, 5> settings_categories = {"Home", "Physics Settings", "Graphics Settings",
                                                                "Graph Plots Settings"};
static const std::array<const char *, 5> view_categories = {"Graphics View", "Graph View", "Well Design"};

struct UI_utils {
    static constexpr uint tiles_per_column = 3;
    static constexpr float ICON_SCALE = 0.6f;
    static constexpr float VIRTUAL_MENU_BAR_WIDTH = 75.0f;
    static constexpr float VIRTUAL_MENU_BAR_WIDTH_EXPANDED = 350.0f;
    static constexpr float VIRTUAL_MENU_BAR_VERT_SPACING = 1.0f;
    static constexpr float VIRTUAL_SETTINGS_WIDTH = 200.0f;
    static constexpr float VIRTUAL_INDENT_SIZE = 10.0f;

    float scale = 1.0f; // Overall UI scale factor; defined as 1.0 for 1080p
    float MENU_BAR_WIDTH = VIRTUAL_MENU_BAR_WIDTH;
    float MENU_BAR_WIDTH_EXPANDED = VIRTUAL_MENU_BAR_WIDTH_EXPANDED;
    float MENU_BAR_VERT_SPACING = VIRTUAL_MENU_BAR_VERT_SPACING;
    float SETTINGS_WIDTH = VIRTUAL_SETTINGS_WIDTH;
    float INDENT_SIZE = VIRTUAL_INDENT_SIZE;

    std::vector<GLuint> sidebar_textures;
    std::vector<ImVec2> sidebar_texture_sizes;

    void set_scale(float framebuffer_width, float framebuffer_height) {
        float scale_x = framebuffer_width / float(sito::VIRTUAL_WIDTH);
        float scale_y = framebuffer_height / float(sito::VIRTUAL_HEIGHT);
        scale = std::min(scale_x, scale_y);
        MENU_BAR_WIDTH = VIRTUAL_MENU_BAR_WIDTH * scale;
        MENU_BAR_WIDTH_EXPANDED = VIRTUAL_MENU_BAR_WIDTH_EXPANDED * scale;
        MENU_BAR_VERT_SPACING = VIRTUAL_MENU_BAR_VERT_SPACING * scale;
        SETTINGS_WIDTH = VIRTUAL_SETTINGS_WIDTH * scale;
        INDENT_SIZE = VIRTUAL_INDENT_SIZE * scale;
    }
};

class UI_Manager {
  public:
    UI_Manager(GLFWwindow *window, uint N_br_cells, float screen_width, float screen_height);
    ~UI_Manager();
    UI_settings ui_settings;
    UI_utils ui_utils;
    unique_ptr<ImGuiManager> imgui_manager;
    HolePlotData hole_plot_data;
    BitRockPlot bit_rock_plot;

    // Tile-based plotting system
    vector<PlotTile> plot_tiles;
    vector<scalar> s_nodewise;    // (N)nodewise s-axis values
    vector<scalar> s_elementwise; // (N + 1) elementwise s-axis values + start and end node

    void render_imgui();

    void init_simulation(Simulator *simulator, const array<SpatialGraph, N_SG> &spatial_graphs,
                         const array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                         const array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs);
    void sync_tiles_with_checkbox_states(Config &config, const array<SpatialGraph, N_SG> &spatial_graphs,
                                         const array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                         const array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs);
    void initialize_default_plot_tiles();
    void call_graph_plot_window(Config &config, const ConfigDynamic &conf_dyn, const curvlin::PipeSolver &pipesolver,
                                const array<SpatialGraph, N_SG> &spatial_graphs,
                                const array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                const array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs, const Pipe &pipe,
                                const Hole &hole, const byte *buf, const BitRockData &bit_rock_data);
    void render_plot_tile(const PlotTile &tile, Config &config, const ConfigDynamic &conf_dyn,
                          const curvlin::PipeSolver &pipesolver, const array<SpatialGraph, N_SG> &spatial_graphs,
                          const array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                          const array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs, const Pipe &pipe,
                          const Hole &hole, const byte *buf, const BitRockData &bit_rock_data, const ImVec2 &tile_size);
    void calculate_tile_positions(); // Calculate positions for all tiles based on their sizes
    void render_side_menu_bar(Simulator *simulator, ConfigGraphics &config_graphics, string *input_file,
                              std::atomic<bool> &start_simulator, std::atomic<bool> &restart_solver,
                              array<SpatialGraph, N_SG> &spatial_graphs,
                              array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                              array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs);
    size_t call_load_sidebar_textures(const std::vector<std::string> &paths);

    void call_imgui_popup_message(const char *message, bool *show_message);

    void call_imgui_popup_message_with_cooldown(const char *message, bool *show_message, bool *trigger_condition,
                                                float cooldown_time = 2.0f);
    void call_imgui_hole_quantities_control_and_plot_window();

    void call_imgui_display_pipe_data(const vector<PipeComponent> &pipe);

    UI_Manager(GLFWwindow *window, float screen_width, float screen_height);

    void new_frame();
    void call_imgui_user_control_panel(ConfigGraphics &config_graphics);
    void call_file_chooser(string *input_file, std::atomic<bool> &start_simulator, bool running);

  private:
    unique_ptr<ImGuiFileChooser> file_chooser;
    array<PlotGroup, 6> plot_groups;
    vector<OrbitalPlot> orbital_plots;

    ImGuiWindowFlags imgui_set_wiew_port();
    void free_sidebar_textures();

    void render_main_settings(Simulator *simulator, string *input_file, std::atomic<bool> &start_simulator,
                              std::atomic<bool> &restart_solver);

    void render_physics_settings(Simulator *simulator);

    void render_graphics_settings(Simulator *simulator, ConfigGraphics &config_graphics,
                                  array<SpatialGraph, N_SG> &spatial_graphs,
                                  array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                  array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs);

    void render_graph_plots_settings(Simulator *simulator, array<SpatialGraph, N_SG> &spatial_graphs,
                                     array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                     array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs);

    void call_imgui_orbital_subplot(OrbitalPlot &plot, Config &config, const curvlin::PipeSolver &pipesolver,
                                    const Pipe &pipe, const Hole &hole, const byte *buf);
    void call_imgui_spatial_graph_plot_control_panel(Config &config, array<SpatialGraph, N_SG> &spatial_graphs,
                                                     array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                                     array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs);
    void call_imgui_solver_control_panel(Config &config, std::atomic<bool> &restart_solver);

    void call_imgui_physics_control_panel(Config &config);

    void call_imgui_top_control_control_panel(Config &config);

    void call_imgui_bottom_bc_control_panel(Config &config);

    void call_imgui_key_quantities_plot_control_panel(Config &config, array<SpatialGraph, N_SG> &spatial_graphs,
                                                      array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                                      array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs);

    void call_imgui_orbital_plot_control_panel(Config &config, const Pipe &pipe, const uint top_node, const byte *buf);

    void plot_single_graph(const scalar *y_data, const scalar *_data, uint N_start, uint N_end, scalar y_min,
                           scalar y_max, scalar s_min, scalar s_max, const string &y_label, const string &legend,
                           const string &plot_title);

    void update_imgui_mouse_status();
    void call_imgui_spatial_plot_control_limit_selection(string name, float &y_min, float &y_max, float &s_min,
                                                         float &s_max, bool &auto_size, bool &reset_scale);

    void call_imgui_contour_plot_control_panel(Config &config, ConfigGraphics &config_graphics,
                                               array<SpatialGraph, N_SG> &spatial_graphs,
                                               array<MiscSpatialGraph, N_MG> &misc_spatial_graphs,
                                               array<FluidSpatialGraph, N_FG> &fluid_spatial_graphs);

    void call_imgui_key_quantity_plot_control_limit_selection(DrillingQuantityGraph &plot, const char *name);
    DrillingQuantityGraph &get_graph(DrillingVariable var);
};

class ImGuiFileChooser {
  public:
    string selected_file;
    ImGuiFileChooser();

    void call_imgui_file_chooser(string *input_file, bool running);

  private:
    std::filesystem::path current_path;
};
