#ifndef LOG_H
#define LOG_H

#include <string>
#include <iostream>

namespace utils {
    // Log level enum
    enum class LogLevel {
        INFO,
        WARNING,
        ERROR
    };

    // Log function
    inline void log(LogLevel level, const std::string& message) {
        switch (level) {
            case LogLevel::INFO:
                std::cout << "[INFO] " << message << std::endl;
                break;
            case LogLevel::WARNING:
                std::cout << "\033[33m[WARNING] " << message << "\033[0m" << std::endl;
                break;
            case LogLevel::ERROR:
                std::cout << "\033[31m[ERROR] " << message << "\033[0m" << std::endl;
                break;
        }
    }

    // Convenient utility functions
    inline void log_info(const std::string& message) {
        log(LogLevel::INFO, message);
    }

    inline void log_warning(const std::string& message) {
        log(LogLevel::WARNING, message);
    }

    inline void log_error(const std::string& message) {
        log(LogLevel::ERROR, message);
    }
}

#endif // LOG_H 