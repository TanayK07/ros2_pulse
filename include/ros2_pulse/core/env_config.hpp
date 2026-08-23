// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef ROS2_PULSE__CORE__ENV_CONFIG_HPP_
#define ROS2_PULSE__CORE__ENV_CONFIG_HPP_

#include <optional>

namespace ros2_pulse::core {

/// @brief Parse a publish-period (in seconds) out of a raw environment-variable string.
///
/// `noexcept` by contract: this runs inside the probe's Meyers-singleton constructor, which is
/// reached from an `extern "C"` tracepoint called by rclcpp. An exception escaping here would
/// unwind across that boundary into code not compiled to expect it and terminate the host process
/// (see KNOWN_ISSUES.md #5).
///
/// Any input that is null (unset), empty / whitespace-only, non-numeric, carries trailing junk,
/// is non-finite (`nan` / `inf`), or is `<= 0` yields @p def instead of throwing. Surrounding
/// whitespace is tolerated; the numeric grammar is locale-independent (decimal point is always `.`).
///
/// @param raw  the raw value (e.g. from `std::getenv`); `nullptr` means "unset".
/// @param def  fallback returned for any unusable input; the caller must pass a sane default.
/// @return the parsed seconds on success, otherwise @p def.
double parsePeriodSeconds(const char* raw, double def) noexcept;

/// @brief Parse a byte-size cap (ROS_TOPIC_STATS_MAX_BYTES) out of a raw env-var string.
///
/// Same `noexcept` contract and rationale as parsePeriodSeconds (KNOWN_ISSUES #5): reached from
/// the tracepoint-driven singleton constructor, must never throw.
///
/// `"0"` is a VALID value meaning "rotation disabled", distinct from unusable input. Unset,
/// empty, non-integral (`"1.5"`), negative, or trailing-junk input yields @p def. Surrounding
/// whitespace is tolerated.
unsigned long long parseMaxBytes(const char* raw, unsigned long long def) noexcept;

/// On-disk output format selector (ROS_TOPIC_STATS_FORMAT, ROADMAP R6).
enum class eStatsFormat {
    kText,   ///< the historical line-oriented block format (default, byte-identical to pre-R6)
    kJsonl,  ///< JSON Lines: one JSON object per window, one line, '\n'-terminated (jsonlines.org)
};

/// @brief Parse ROS_TOPIC_STATS_FORMAT out of a raw env-var string.
///
/// Same `noexcept` contract as the parsers above (KNOWN_ISSUES #5): reached from the
/// tracepoint-driven singleton constructor, must never throw.
///
/// Unset / empty / whitespace-only means "not asked for" and yields kText, the format an
/// operator gets without touching anything must be byte-identical to what every existing
/// consumer parses. `"text"` and `"jsonl"` (exact, lowercase, the documented forms, matching
/// the strict `"1"`-only opt-in flags) select explicitly; surrounding whitespace is tolerated
/// like the other env parsers because these are exported via the shell.
///
/// Any OTHER value returns std::nullopt so the caller can warn once and fall back to text,
/// distinct from the silent kText default, because a typo ("json", "JSONL") is an operator
/// asking for something and not getting it, which must be said out loud (the spec-file error
/// philosophy: never crash the host, never silently ignore a request).
auto parseStatsFormat(const char* raw) noexcept -> std::optional<eStatsFormat>;

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__ENV_CONFIG_HPP_
