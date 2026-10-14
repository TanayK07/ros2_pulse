// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#include "ros2_pulse/core/tracetools_version.hpp"

#include <cctype>

namespace ros2_pulse::core {

namespace {

auto trim(std::string_view s) -> std::string_view {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front())) != 0) {
        s.remove_prefix(1);
    }
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())) != 0) {
        s.remove_suffix(1);
    }
    return s;
}

}  // namespace

auto parsePackageVersion(std::string_view xml) -> std::optional<std::string> {
    constexpr std::string_view kOpen = "<version>";
    constexpr std::string_view kClose = "</version>";
    const auto open = xml.find(kOpen);
    if (open == std::string_view::npos) {
        return std::nullopt;
    }
    const auto begin = open + kOpen.size();
    const auto close = xml.find(kClose, begin);
    if (close == std::string_view::npos) {
        return std::nullopt;
    }
    const auto text = trim(xml.substr(begin, close - begin));
    if (text.empty()) {
        return std::nullopt;
    }
    return std::string(text);
}

auto versionMajor(std::string_view version) noexcept -> std::optional<int> {
    int major = 0;
    std::size_t digits = 0;
    for (const char c : version) {
        if (c < '0' || c > '9') {
            break;
        }
        if (++digits > 6) {
            return std::nullopt;
        }
        major = major * 10 + (c - '0');
    }
    if (digits == 0) {
        return std::nullopt;
    }
    return major;
}

auto isTestedTracetoolsMajor(int major) noexcept -> bool {
    return major == 4 || major == 8 || major == 9;
}

auto tracetoolsPackageXmlPath(std::string_view lib_path) -> std::string {
    const auto file_slash = lib_path.rfind('/');
    if (file_slash == std::string_view::npos || file_slash == 0) {
        return {};
    }
    const auto dir_slash = lib_path.rfind('/', file_slash - 1);
    if (dir_slash == std::string_view::npos) {
        return {};
    }
    return std::string(lib_path.substr(0, dir_slash)) + "/share/tracetools/package.xml";
}

}  // namespace ros2_pulse::core
