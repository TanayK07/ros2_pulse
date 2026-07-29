// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef ROS2_PULSE__CORE__ENV_CONFIG_HPP_
#define ROS2_PULSE__CORE__ENV_CONFIG_HPP_

namespace ros2_pulse::core {

/// @brief Parse a publish-period (in seconds) out of a raw environment-variable string.
///
/// `noexcept` by contract: this runs inside the probe's Meyers-singleton constructor, which is
/// reached from an `extern "C"` tracepoint called by rclcpp. An exception escaping here would
/// unwind across that boundary into code not compiled to expect it and terminate the host process
/// (see docs/issues/issue-5-env-parse-safety.md / KNOWN_ISSUES.md #5).
///
/// Any input that is null (unset), empty / whitespace-only, non-numeric, carries trailing junk,
/// is non-finite (`nan` / `inf`), or is `<= 0` yields @p def instead of throwing. Surrounding
/// whitespace is tolerated; the numeric grammar is locale-independent (decimal point is always `.`).
///
/// @param raw  the raw value (e.g. from `std::getenv`); `nullptr` means "unset".
/// @param def  fallback returned for any unusable input; the caller must pass a sane default.
/// @return the parsed seconds on success, otherwise @p def.
double parsePeriodSeconds(const char* raw, double def) noexcept;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__ENV_CONFIG_HPP_
