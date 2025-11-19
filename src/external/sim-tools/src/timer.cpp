#include "sim-tools/timer.hpp"

namespace sito {

double Timer::get_elapsed_time_sec() const {
    using std::chrono::duration_cast;

    Time time = is_running ? Clock::now() : end_time;
    auto total_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(time - start_time);
    return total_seconds.count();
}

void Timer::print_elapsed_time() const {
    cout << "Elapsed time = " << get_elapsed_time_sec() << " sec\n";
};

void Timer::start_counter() {
    start_time = Clock::now();
    is_running = true;
}
void Timer::stop_counter() {
    end_time = Clock::now();
    is_running = false;
}

} // namespace sito