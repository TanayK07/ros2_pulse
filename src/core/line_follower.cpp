// Copyright 2026 ros2_pulse contributors
//
// See line_follower.hpp for the contract.

#include "ros2_pulse/core/line_follower.hpp"

#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace ros2_pulse::core {

LineFollower::LineFollower(std::string path) : m_path(std::move(path)) {}

auto LineFollower::poll() -> std::vector<std::string> {
    std::vector<std::string> out;
    std::ifstream f(m_path, std::ios::binary);
    if (!f) return out;

    f.seekg(0, std::ios::end);
    if (!f) return out;
    const auto size = static_cast<std::uint64_t>(f.tellg());
    if (size < m_offset) {
        // Shrunk below where we were: rotated in place. Anything held from the old file is gone.
        m_offset = 0;
        m_pending.clear();
    }
    if (size == m_offset) return out;

    f.seekg(static_cast<std::streamoff>(m_offset));
    std::string chunk(static_cast<std::size_t>(size - m_offset), '\0');
    f.read(chunk.data(), static_cast<std::streamsize>(chunk.size()));
    chunk.resize(static_cast<std::size_t>(f.gcount()));
    m_offset += chunk.size();
    m_pending += chunk;

    std::size_t start = 0;
    for (;;) {
        const auto nl = m_pending.find('\n', start);
        if (nl == std::string::npos) break;
        out.push_back(m_pending.substr(start, nl - start));
        start = nl + 1;
    }
    m_pending.erase(0, start);
    return out;
}

}  // namespace ros2_pulse::core
