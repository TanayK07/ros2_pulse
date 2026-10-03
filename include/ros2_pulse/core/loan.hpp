// Copyright 2026 ros2_pulse contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ROS2_PULSE__CORE__LOAN_HPP_
#define ROS2_PULSE__CORE__LOAN_HPP_

// Loaned-message counting helpers (rclcpp#3153 follow-up), pure and ROS-free so the decisions
// the probe's rcl wrappers make are unit-testable.
//
// Why the probe wraps rcl functions here instead of reading a tracepoint: at the tracepoint
// level a loan is indistinguishable from a copy. Jazzy+ fires the plain rcl_publish tracepoint
// inside rcl_publish_loaned_message; rcl_take_loaned_message fires the plain rcl_take tracepoint
// only on Rolling; Humble traces neither loaned call. rclcpp calls rcl_publish_loaned_message
// only when publisher->can_loan_messages() is true and rcl_take_loaned_message only when
// subscription->can_loan_messages() is true (rclcpp/publisher.hpp publish(LoanedMessage&&),
// rclcpp/executor.cpp execute_subscription), so a successful call IS a middleware loan.

#include <atomic>
#include <cstdint>

namespace ros2_pulse::core {

/// rcl_ret_t RCL_RET_OK (rcl/types.h). The same value on every distro; mirrored here so the
/// core stays free of rcl headers.
constexpr int kRclRetOk = 0;

/// A loaned publish counts only when rcl_publish_loaned_message returned RCL_RET_OK.
constexpr auto countLoanedPublish(int rcl_ret) -> bool { return rcl_ret == kRclRetOk; }

/// A loaned take counts only when rcl_take_loaned_message returned RCL_RET_OK AND produced a
/// message. RCL_RET_SUBSCRIPTION_TAKE_FAILED (nothing available, a spurious wake-up) is normal
/// and is not a delivery.
constexpr auto countLoanedTake(int rcl_ret, const void* loaned_message) -> bool {
    return rcl_ret == kRclRetOk && loaned_message != nullptr;
}

// ---- does rcl_publish_loaned_message fire the plain rcl_publish tracepoint? ----
//
// On Jazzy+ it does, so the tracepoint interposer has already added the publish to the topic's
// total and the wrapper must count only the loan; on Humble it does not, and the wrapper must
// add the publish to the total too. That is a property of the process's one librcl, so the
// probe LEARNS it from the first successful loaned publish and caches it process-wide: no
// distro sniffing, one binary correct everywhere, no double count.
//
// Learning uses a thread-local latch: the wrapper opens a LoanedPublishScope around the real
// call, and the ros_trace_rcl_publish interposer marks it via noteRclPublishTracepoint(). The
// latch is gated by a count of calls currently learning, nonzero only during the first loaned
// publish(es) of a process, so the steady-state cost on EVERY ordinary publish, in a process
// that loans or not, is one relaxed load of a global that is otherwise only read. Rejected
// alternatives, each measured with paired n=10 trials on the real .so: a thread-local check on
// every publish (+1.4 ns per publish with the default TLS model, +0.45 ns with initial-exec),
// and a "loan in flight" counter on every loaned publish (one shared RMW each, 340 ns per loaned
// publish under 8 publishing threads).

enum class eLoanTracepoint : int { kUnknown = -1, kSilent = 0, kFires = 1 };

namespace detail {
inline std::atomic<int> g_loan_tracepoint{static_cast<int>(eLoanTracepoint::kUnknown)};
// Loaned calls currently in their LEARNING phase (only the first few per process). Gates the
// TLS read below, so neither a process that never loans nor one that already learned pays it.
inline std::atomic<int> g_learning{0};
inline thread_local bool t_in_loaned_publish = false;
inline thread_local bool t_loaned_tracepoint_fired = false;
}  // namespace detail

/// Called from the ros_trace_rcl_publish interposer on every publish. A learning call's own
/// increment of g_learning is sequenced before its tracepoint, so it always reaches the TLS
/// check; any other thread that sees it non-zero merely reads its own (false) flag.
inline void noteRclPublishTracepoint() {
    if (detail::g_learning.load(std::memory_order_relaxed) != 0 &&
        detail::t_in_loaned_publish) {
        detail::t_loaned_tracepoint_fired = true;
    }
}

/// The cached mode (kUnknown until the first successful loaned publish).
inline auto loanTracepointMode() -> eLoanTracepoint {
    return static_cast<eLoanTracepoint>(detail::g_loan_tracepoint.load(std::memory_order_relaxed));
}

/// Test-only: forget the learned mode (each unit test simulates a different librcl).
inline void resetLoanTracepointModeForTest() {
    detail::g_loan_tracepoint.store(static_cast<int>(eLoanTracepoint::kUnknown));
}

/// RAII latch around one forwarded rcl_publish_loaned_message call (learning phase only).
/// rcl_publish_loaned_message cannot re-enter itself on one thread, so one flag pair suffices.
class LoanedPublishScope {
public:
    LoanedPublishScope() {
        detail::t_loaned_tracepoint_fired = false;
        detail::t_in_loaned_publish = true;
        detail::g_learning.fetch_add(1, std::memory_order_relaxed);
    }
    ~LoanedPublishScope() {
        detail::g_learning.fetch_sub(1, std::memory_order_relaxed);
        detail::t_in_loaned_publish = false;
    }
    LoanedPublishScope(LoanedPublishScope const&) = delete;
    auto operator=(LoanedPublishScope const&) -> LoanedPublishScope& = delete;

    /// True when the plain rcl_publish tracepoint fired since this scope opened.
    auto tracepointFired() const -> bool { return detail::t_loaned_tracepoint_fired; }
};

/// Forward one rcl_publish_loaned_message call (@p call returns its rcl_ret_t) and report in
/// @p counted_by_tracepoint whether the publish TOTAL was already counted by the tracepoint.
/// Learns and caches the mode on the first RCL_RET_OK; a failed call never teaches anything,
/// because rcl returns early (before the tracepoint) on an invalid publisher or null message.
template <typename Call>
auto forwardLoanedPublish(Call&& call, bool& counted_by_tracepoint) -> int32_t {
    const auto mode = loanTracepointMode();
    if (mode != eLoanTracepoint::kUnknown) {
        counted_by_tracepoint = mode == eLoanTracepoint::kFires;
        return call();
    }
    int32_t ret = 0;
    bool fired = false;
    {
        LoanedPublishScope scope;
        ret = call();
        fired = scope.tracepointFired();
    }
    // Several threads can be learning at once. Their observations agree (one librcl per
    // process, and each learner's own g_learning increment keeps its latch armed), but only the
    // first learner publishes the mode (CAS from kUnknown) and every later observer defers to
    // it, so the cached mode can never flap even if an observation were wrong.
    int learned = static_cast<int>(eLoanTracepoint::kUnknown);
    if (ret == kRclRetOk) {
        const int seen = static_cast<int>(fired ? eLoanTracepoint::kFires : eLoanTracepoint::kSilent);
        if (detail::g_loan_tracepoint.compare_exchange_strong(learned, seen,
                                                              std::memory_order_relaxed)) {
            learned = seen;
        }
    } else {
        learned = detail::g_loan_tracepoint.load(std::memory_order_relaxed);
    }
    if (learned != static_cast<int>(eLoanTracepoint::kUnknown)) {
        fired = learned == static_cast<int>(eLoanTracepoint::kFires);
    }
    counted_by_tracepoint = fired;
    return ret;
}

}  // namespace ros2_pulse::core

#endif  // ROS2_PULSE__CORE__LOAN_HPP_
