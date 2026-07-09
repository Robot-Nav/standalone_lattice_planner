// Lattice Planner - Logging Facade
// Replaces RCLCPP_INFO/WARN/ERROR/DEBUG macros with a simple standalone logger.

#ifndef LATTICE_PLANNER__CORE__LOGGER_HPP_
#define LATTICE_PLANNER__CORE__LOGGER_HPP_

#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

namespace lattice_planner {

enum class LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3 };

/** @brief Singleton logger that writes to stdout/stderr.
 *
 *  Thread-safe. Set the level via setLevel() to filter messages.
 *  Replace RCLCPP_* macros; usage: LP_LOG_INFO("message " << value);
 */
class Logger {
public:
  static Logger & instance() {
    static Logger inst;
    return inst;
  }

  void setLevel(LogLevel level) { level_ = level; }

  LogLevel getLevel() const { return level_; }

  void log(LogLevel level, const std::string & msg) {
    if (static_cast<int>(level) < static_cast<int>(level_)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostream & os = (level == LogLevel::ERROR) ? std::cerr : std::cout;
    os << levelToString(level) << msg << std::endl;
  }

private:
  Logger() : level_(LogLevel::INFO) {}

  const char * levelToString(LogLevel level) {
    switch (level) {
      case LogLevel::DEBUG: return "[DEBUG] ";
      case LogLevel::INFO:  return "[INFO]  ";
      case LogLevel::WARN:  return "[WARN]  ";
      case LogLevel::ERROR: return "[ERROR] ";
    }
    return "[?]     ";
  }

  LogLevel level_;
  std::mutex mutex_;
};

}  // namespace lattice_planner

// Stream-based logging macros (accept << chaining like RCLCPP_INFO)
#define LP_LOG_STREAM(level, msg)                                        \
  do {                                                                   \
    std::ostringstream _lp_ss;                                           \
    _lp_ss << msg;                                                       \
    ::lattice_planner::Logger::instance().log(level, _lp_ss.str());      \
  } while (0)

#define LP_LOG_DEBUG(msg) LP_LOG_STREAM(::lattice_planner::LogLevel::DEBUG, msg)
#define LP_LOG_INFO(msg)  LP_LOG_STREAM(::lattice_planner::LogLevel::INFO, msg)
#define LP_LOG_WARN(msg)  LP_LOG_STREAM(::lattice_planner::LogLevel::WARN, msg)
#define LP_LOG_ERROR(msg) LP_LOG_STREAM(::lattice_planner::LogLevel::ERROR, msg)

#endif  // LATTICE_PLANNER__CORE__LOGGER_HPP_
