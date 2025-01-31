#ifndef LOG_H
#define LOG_H

#include <string>
#include <iostream>
#include <vector>
#include "../Vertex.h"
#include "../cbs/CBS.h"
#include "../jps/JPS.h"
#include <sstream>

namespace logger {
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

    // Convert vector to string representation
    template<typename T>
    inline std::string vectorToString(const std::vector<T>& vec) {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            ss << vec[i];
            if (i < vec.size() - 1) ss << ", ";
        }
        ss << "]";
        return ss.str();
    }

    // Specialization for Vertex
    template<>
    inline std::string vectorToString(const std::vector<Vertex>& vec) {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            ss << "(" << vec[i].x << "," << vec[i].y << ")";
            if (i < vec.size() - 1) ss << ", ";
        }
        ss << "]";
        return ss.str();
    }

    // 打印约束信息的函数
    inline void print_constraints(const std::vector<Constraint>& constraints, 
                                const std::string& prefix = "") {
        if (!prefix.empty()) {
            log_info(prefix);
        }
        
        if (constraints.empty()) {
            log_info("No constraints");
            return;
        }

        std::stringstream ss;
        ss << "Constraint list: [";
        for (size_t i = 0; i < constraints.size(); ++i) {
            const auto& constraint = constraints[i];
            ss << "Agent " << constraint.agent 
               << " cannot reach position (" << constraint.vertex.x 
               << "," << constraint.vertex.y << ") at time " 
               << constraint.time;
               
            if (i < constraints.size() - 1) {
                ss << ", ";
            }
        }
        ss << "]";
        log_info(ss.str());
    }

    // 打印单个约束的函数
    inline void print_constraint(const Constraint& constraint, 
                               const std::string& prefix = "") {
        if (!prefix.empty()) {
            log_info(prefix);
        }
        
        std::stringstream ss;
        ss << "Constraint: Agent " << constraint.agent 
           << " cannot reach position (" << constraint.vertex.x 
           << "," << constraint.vertex.y << ") at time " 
           << constraint.time;
        log_info(ss.str());
    }

    // 打印单个区间的函数
    inline void print_interval(const Interval& interval, const std::string& prefix = "") {
        if (!prefix.empty()) {
            log_info(prefix);
        }
        
        std::stringstream ss;
        ss << "Interval: ";
        ss << "Start(" << interval.get_start().x << "," << interval.get_start().y << ") -> ";
        ss << "End(" << interval.get_end().x << "," << interval.get_end().y << ")";
        ss << " Jump points: ";
        for (size_t i = 0; i < interval.jump_points.size(); ++i) {
            ss << "(" << interval.jump_points[i].x << "," << interval.jump_points[i].y << ")";
            if (i < interval.jump_points.size() - 1) {
                ss << " -> ";
            }
        }
        log_info(ss.str());
    }

    // 打印区间列表的函数
    inline void print_intervals(const std::vector<Interval>& intervals, 
                              const std::string& prefix = "") {
        if (!prefix.empty()) {
            log_info(prefix);
        }
        
        if (intervals.empty()) {
            log_info("No symmetric intervals");
            return;
        }

        log_info("Symmetric interval list:");
        for (size_t i = 0; i < intervals.size(); ++i) {
            std::stringstream ss;
            ss << "Interval " << i + 1 << ": ";
            ss << "Start(" << intervals[i].get_start().x << "," << intervals[i].get_start().y << ") -> ";
            ss << "End(" << intervals[i].get_end().x << "," << intervals[i].get_end().y << ")";
            log_info(ss.str());
            
            ss.str("");
            ss << "    Jump points: ";
            for (size_t j = 0; j < intervals[i].jump_points.size(); ++j) {
                ss << "(" << intervals[i].jump_points[j].x << "," << intervals[i].jump_points[j].y << ")";
                if (j < intervals[i].jump_points.size() - 1) {
                    ss << " -> ";
                }
            }
            log_info(ss.str());
        }
    }
}

#endif // LOG_H 