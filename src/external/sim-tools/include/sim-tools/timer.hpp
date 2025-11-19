#include "includes.hpp"
#include <chrono>
namespace sito {
class Timer {
    using Clock = std::chrono::high_resolution_clock;
    using Time = std::chrono::_V2::system_clock::time_point;
    Time start_time, end_time;
    bool is_running{false};

  public:
    void start_counter();
    void stop_counter();
    double get_elapsed_time_sec() const;
    void print_elapsed_time() const;
};

} // namespace sito