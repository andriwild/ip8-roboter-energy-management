#include <string>
#include <chrono>
#include <sstream>


std::string getFormattedDateTime() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::tm tm_now = *std::localtime(&time_t_now);
    
    std::stringstream ss;
    ss << std::put_time(&tm_now, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}