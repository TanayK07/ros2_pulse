// Copyright 2026 ros2_pulse contributors
//
// Incremental line reader for a growing jsonl log (`tail -f`, minus the process). Each poll()
// returns the complete lines appended since the previous poll; a trailing partial line is held
// until its newline arrives. A file that shrinks below the last offset (rotated in place) is
// re-read from the top; a file that does not exist yet yields nothing until it appears.

#ifndef ROS2_PULSE__CORE__LINE_FOLLOWER_HPP_
#define ROS2_PULSE__CORE__LINE_FOLLOWER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace ros2_pulse::core {

class LineFollower {
public:
    explicit LineFollower(std::string path);

    /// Complete lines appended since the last call, without their '\n'. Never throws.
    auto poll() -> std::vector<std::string>;

    auto path() const -> const std::string& { return m_path; }

private:
    std::string m_path;
    std::uint64_t m_offset{0};
    std::string m_pending;  // bytes after the last newline, waiting for the rest of the line
};

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__LINE_FOLLOWER_HPP_
