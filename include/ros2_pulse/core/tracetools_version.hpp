// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef ROS2_PULSE__CORE__TRACETOOLS_VERSION_HPP_
#define ROS2_PULSE__CORE__TRACETOOLS_VERSION_HPP_

#include <optional>
#include <string>
#include <string_view>

namespace ros2_pulse::core {

/// @brief The `<version>` text of a package.xml, trimmed; nullopt if there is none.
///
/// The probe cannot read the version the `rcl_init` tracepoint records: tracetools adds it
/// inside its own LTTng event, the interposer only ever sees `context_handle`. So it reads the
/// tracetools package.xml next to the libtracetools.so that serves its hooks instead.
auto parsePackageVersion(std::string_view xml) -> std::optional<std::string>;

/// @brief Leading major number of a version string ("4.1.2" -> 4); nullopt if it has none.
auto versionMajor(std::string_view version) noexcept -> std::optional<int>;

/// @brief Whether CI runs the probe against this tracetools major.
///
/// 4 is Humble, 8 is Jazzy through Lyrical, 9 is Rolling. Upstream promises tracepoint
/// stability only within a distro (with tick-tock deprecation across distros), and in
/// practice adds tracepoints and deprecates arguments without removing them, so an unknown
/// major is worth a warning, not a refusal.
auto isTestedTracetoolsMajor(int major) noexcept -> bool;

/// @brief package.xml path for the tracetools that @p lib_path (a libtracetools.so) belongs to.
///
/// `<prefix>/lib/libtracetools.so` -> `<prefix>/share/tracetools/package.xml`, the layout of
/// both /opt/ros and a colcon install. Empty when @p lib_path has no `<prefix>/<dir>/` shape.
auto tracetoolsPackageXmlPath(std::string_view lib_path) -> std::string;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__TRACETOOLS_VERSION_HPP_
