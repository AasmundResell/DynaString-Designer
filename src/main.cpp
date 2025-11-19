#include "gui/gui-manager-base.hpp"
#include "gui/ui/imgui-manager.hpp"
#include "simulator.hpp"
#include <thread>

void print_usage() {
    cout << "Usage: DynaString [OPTIONS]\n"
         << "Options:\n"
         << "  -i --input FILENAME    Input YAML file path\n"
         << "  -g --gui               Enable GUI\n"
         << "  -h --help              Display this help message\n";
}

bool parse_args(int argc, char *argv[], string *input_file, bool *gui_enabled, std::atomic<bool> &start_simulator) {
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            print_usage();
            return false;
        } else if (arg == "--gui" || arg == "-g") {
            *gui_enabled = true;
        } else if (arg == "--input" || arg == "-i") {
            if (i + 1 < argc) {
                *input_file = argv[++i];
                start_simulator = true;
            } else {
                cerr << "Error: --input requires a file path\n";
                return false;
            }
        } else {
            cerr << "Unknown argument: " << arg << "\n";
            print_usage();
            return false;
        }
    }

    if (input_file->empty() && !gui_enabled) {
        cerr << "Error: --input FILENAME is required when --gui is not specified\n";
        return false;
    }
    return true;
}
void run_program(string &input_file, GUI_ManagerBase *gui_manager, std::atomic<bool> &start_simulator) {

    using clock = std::chrono::steady_clock;

    ConfigGUI config_gui{};
    config_gui.gui_enabled = gui_manager != nullptr;

    unique_ptr<Simulator> simulator = nullptr;

    std::thread solver_thread;
    std::atomic<bool> stop_program{false};
    std::atomic<bool> stop_solver{false};
    std::atomic<bool> restart_solver = false;
    std::atomic<bool> restart_program = false;

    scalar last_frame_time;
    scalar current_time;
    bool draw_min = false;

    while (!stop_program) {

        // Simulator creation / restart requested
        if (start_simulator) {
            start_simulator = false;

            // Stop existing solver thread if running
            if (solver_thread.joinable()) {
                stop_solver = true;
                solver_thread.join();
                stop_solver = false;
            }

            simulator.reset();

            assert(!input_file.empty());
            PipeRenderComponents pipe_render_components{};
            simulator = make_unique<Simulator>(input_file, config_gui, &pipe_render_components);

            if (gui_manager != nullptr) {
                gui_manager->init_simulation(simulator.get(), pipe_render_components);
            }

            solver_thread = std::thread([&] {
                while (!stop_solver && !stop_program) {
                    simulator->progress_solver(config_gui, stop_program);
                    simulator->sleep_solver(config_gui.gui_enabled);
                }
            });
        }

        // GUI Loop
        if (gui_manager != nullptr) {
            gui_manager->render(simulator.get(), &input_file, stop_program, start_simulator, restart_solver);
            last_frame_time = current_time;

            if (restart_solver) {
                restart_solver = false;
                if (simulator != nullptr) {
                    simulator->stop_timer();
                    simulator->restart_solver();
                }
            }
            if (start_simulator) {
                assert(!input_file.empty());
                if (simulator != nullptr) {
                    stop_solver = true;
                }
            }
        }

        if (stop_program) {
            if (simulator != nullptr) {
                stop_solver = true;
            }
        }

        if (stop_solver) {
            stop_solver = false;
            assert(simulator != nullptr);
            simulator->stop_timer();
        }
    }

    if (solver_thread.joinable())
        solver_thread.join();

    // If we ran without GUI, print final timing ratio
    if (!config_gui.gui_enabled) {
        simulator->stop_timer();
    }
    simulator.reset();
}

int main(int argc, char *argv[]) {
    try {
        string input_file;
        bool gui_enabled = false;
        std::atomic<bool> start_simulator = false;
        printf("\n\n////////////////////////////////////////////\n"
               "////////////// DynaString //////////////\n"
               "////////////////////////////////////////////\n\n");

        if (!parse_args(argc, argv, &input_file, &gui_enabled, start_simulator)) {
            return EXIT_FAILURE;
        }

        // test_arena_copy();

        unique_ptr<GUI_ManagerBase> gui_manager = nullptr;

        if (gui_enabled) {
            gui_manager = GUI_ManagerBase::create();
            printf("Created GUI manager\n");
        }

        run_program(input_file, gui_manager.get(), start_simulator);

    } catch (exception &e) {
        cerr << "Exception caught:\n" << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
