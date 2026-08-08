// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License").

#include "ros2_pulse/core/env_config.hpp"

#include <charconv>
#include <cmath>
#include <system_error>

namespace ros2_pulse::core {

namespace {
// ASCII whitespace check that never sees a negative value (unlike std::isspace with a raw char).
auto isAsciiSpace(char c) -> bool {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v';
}
}  // namespace

double parsePeriodSeconds(const char* raw, double def) noexcept {
    if (raw == nullptr) {
        return def;  // variable unset
    }

    const char* first = raw;
    const char* last = raw;
    while (*last != '\0') {
        ++last;
    }

    // Trim surrounding whitespace — operators export these via the shell, where a stray space is
    // easy to introduce. Interior / trailing garbage is still rejected below.
    while (first != last && isAsciiSpace(*first)) {
        ++first;
    }
    while (last != first && isAsciiSpace(*(last - 1))) {
        --last;
    }
    if (first == last) {
        return def;  // empty or whitespace-only
    }

    double value = 0.0;
    const std::from_chars_result res = std::from_chars(first, last, value);
    // Require a clean parse that consumed the WHOLE trimmed token (rejects "5abc", "1,5", ...).
    if (res.ec != std::errc{} || res.ptr != last) {
        return def;
    }
    // from_chars accepts "inf"/"nan" like strtod; a publish period must be finite and positive
    // (<= 0 would make the flush timer spin).
    if (!std::isfinite(value) || value <= 0.0) {
        return def;
    }
    return value;
}

unsigned long long parseMaxBytes(const char* raw, unsigned long long def) noexcept {
    if (raw == nullptr) {
        return def;  // variable unset
    }

    const char* first = raw;
    const char* last = raw;
    while (*last != '\0') {
        ++last;
    }
    while (first != last && isAsciiSpace(*first)) {
        ++first;
    }
    while (last != first && isAsciiSpace(*(last - 1))) {
        --last;
    }
    if (first == last) {
        return def;  // empty or whitespace-only
    }

    unsigned long long value = 0;
    const std::from_chars_result res = std::from_chars(first, last, value);
    // Whole-token integral parse only: rejects "512abc", "1.5", and (via unsigned) "-1".
    if (res.ec != std::errc{} || res.ptr != last) {
        return def;
    }
    return value;  // 0 is legitimate: rotation disabled
}

auto parseStatsFormat(const char* raw) noexcept -> std::optional<eStatsFormat> {
    // TODO(jsonl): stub — implemented with the R6 jsonl emitter.
    (void)raw;
    return std::nullopt;
}

}  // namespace ros2_pulse::core
