// Copyright 2026 ros2_pulse contributors
//
// Test nodes for the ros2_pulse integration test. Modes:
//   talker     : publishes std_msgs/String on /chatter at 50 Hz (separate process)
//   stall_pub <ms> <at_s> : same, but freezes the executor once mid-run for <ms> (ROADMAP R5)
//   listener   : subscribes /chatter (separate process)          -> inter-process receive
//   talker_fixed / listener_fixed : same pair on /chatter_fixed with std_msgs/UInt64, a
//                FIXED-SIZE (self-contained) type, so CycloneDDS+iceoryx is allowed to carry
//                it over shared memory; String would silently fall back to loopback UDP and
//                the RMW-matrix SHM leg (test/rmw/) would prove nothing.
//   intra      : single process, intra-process comms ON, self pub+sub on /intra_topic at 50 Hz
//                                                                -> intra-process receive
//   exit_storm : detached threads hammer the ros_trace_* interposers while main() returns
//                                                                -> KNOWN_ISSUES #9 exit hazard
//   fork_pub   : fork() WITHOUT exec; the child publishes via the interposers and must flush
//                its own windows                                 -> KNOWN_ISSUES #10

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

// exit_storm / fork_pub drive the probe's interposed tracepoints directly (they resolve to
// libros2_pulse.so under LD_PRELOAD, or to tracetools' real no-op functions otherwise). No rcl
// machinery is involved: DDS state is not fork-safe and is irrelevant to the probe path, so the
// ONLY thing that can fail is the probe itself.
extern "C" void ros_trace_rcl_publish(const void* pub_handle, const void* message);
extern "C" void ros_trace_callback_start(const void* callback, bool is_intra_process);
extern "C" void ros_trace_rcl_node_init(const void* node_handle, const void* rmw_handle,
                                        const char* name, const char* ns);
extern "C" void ros_trace_rcl_publisher_init(const void* pub_handle, const void* node_handle,
                                             const void* rmw_pub, const char* topic, size_t depth);

using namespace std::chrono_literals;

static auto qos() -> rclcpp::QoS { return rclcpp::QoS(rclcpp::KeepLast(10)); }

class Talker : public rclcpp::Node {
public:
    explicit Talker(const rclcpp::NodeOptions& o) : Node("poc_talker", o) {
        m_pub = create_publisher<std_msgs::msg::String>("chatter", qos());
        m_timer = create_wall_timer(20ms, [this]() {
            std_msgs::msg::String m;
            m.data = "hello " + std::to_string(m_n++);
            m_pub->publish(m);
        });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    size_t m_n = 0;
};

// Like Talker, but freezes the executor once, mid-run, by sleeping INSIDE the timer callback.
// Under the default single-threaded spin this blocks everything, so publishing stops for exactly
// the requested duration, deterministic, unlike killing a process and racing the flush boundary.
// The point is a stall a windowed mean cannot see: 800 ms out of a 5 s window at 50 Hz still
// averages ~42 Hz, so a min_hz rule passes while the gap detector fires (ROADMAP R5).
class StallTalker : public rclcpp::Node {
public:
    StallTalker(const rclcpp::NodeOptions& o, int stall_ms, double stall_at_s)
        : Node("poc_talker", o), m_stall_ms(stall_ms),
          m_stall_at(std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(static_cast<long>(stall_at_s * 1000.0))) {
        m_pub = create_publisher<std_msgs::msg::String>("chatter", qos());
        m_timer = create_wall_timer(20ms, [this]() {
            if (!m_stalled && std::chrono::steady_clock::now() >= m_stall_at) {
                m_stalled = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(m_stall_ms));
            }
            std_msgs::msg::String m;
            m.data = "hello " + std::to_string(m_n++);
            m_pub->publish(m);
        });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    size_t m_n = 0;
    int m_stall_ms;
    std::chrono::steady_clock::time_point m_stall_at;
    bool m_stalled = false;
};

// Fixed-size pair for the RMW-matrix SHM leg (test/rmw/run_rmw_matrix.sh). std_msgs/UInt64 is
// self-contained, so CycloneDDS 0.10+iceoryx may move it through a RouDi shared-memory segment;
// a String publisher would be quietly ineligible and the leg would measure plain loopback UDP.
class FixedTalker : public rclcpp::Node {
public:
    explicit FixedTalker(const rclcpp::NodeOptions& o) : Node("poc_talker_fixed", o) {
        m_pub = create_publisher<std_msgs::msg::UInt64>("chatter_fixed", qos());
        m_timer = create_wall_timer(20ms, [this]() {
            std_msgs::msg::UInt64 m;
            m.data = m_n++;
            m_pub->publish(m);
        });
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    uint64_t m_n = 0;
};

class FixedListener : public rclcpp::Node {
public:
    explicit FixedListener(const rclcpp::NodeOptions& o) : Node("poc_listener_fixed", o) {
        m_sub = create_subscription<std_msgs::msg::UInt64>(
            "chatter_fixed", qos(), [this](std_msgs::msg::UInt64::SharedPtr) { ++m_got; });
    }

private:
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr m_sub;
    size_t m_got = 0;
};

class Listener : public rclcpp::Node {
public:
    explicit Listener(const rclcpp::NodeOptions& o) : Node("poc_listener", o) {
        m_sub = create_subscription<std_msgs::msg::String>(
            "chatter", qos(), [this](std_msgs::msg::String::SharedPtr) { ++m_got; });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub;
    size_t m_got = 0;
};

class IntraNode : public rclcpp::Node {
public:
    explicit IntraNode(const rclcpp::NodeOptions& o) : Node("poc_intra", o) {
        m_pub = create_publisher<std_msgs::msg::String>("intra_topic", qos());
        m_sub = create_subscription<std_msgs::msg::String>(
            "intra_topic", qos(), [this](std_msgs::msg::String::SharedPtr) { ++m_got; });
        m_timer = create_wall_timer(20ms, [this]() {
            auto m = std::make_unique<std_msgs::msg::String>();
            m->data = "intra " + std::to_string(m_n++);
            m_pub->publish(std::move(m));  // unique_ptr publish = intra-process fast path
        });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    size_t m_n = 0, m_got = 0;
};

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "talker";

    if (mode == "fork_pub") {
        // fork()-without-exec hazard (KNOWN_ISSUES #10). Synthetic graph via direct interposer
        // calls: arm the probe, register a publisher, then fork. The CHILD publishes ~500
        // messages over ~2.5 s and exits normally, its counts must appear in the output file
        // (periodic and/or final window). The parent publishes nothing and waits.
        //
        // Deliberately NO rclcpp::init in this mode: forking after rclcpp::init wedges the
        // child inside rclcpp/DDS fork handlers even WITHOUT the probe preloaded (verified
        // with a no-preload control), and the probe path needs no rcl machinery anyway.
        const auto* node_h = reinterpret_cast<const void*>(0xF0);
        const auto* pub_h = reinterpret_cast<const void*>(0xF1);
        ros_trace_rcl_node_init(node_h, nullptr, "forker", "");
        ros_trace_rcl_publisher_init(pub_h, node_h, nullptr, "/forked", 10);

        const pid_t pid = fork();
        if (pid == 0) {
            for (int i = 0; i < 500; ++i) {
                ros_trace_rcl_publish(pub_h, nullptr);
                std::this_thread::sleep_for(5ms);
            }
            return 0;  // normal exit: atexit final flush must persist the child's counts
        }
        int status = 0;
        waitpid(pid, &status, 0);
        return WIFEXITED(status) ? WEXITSTATUS(status) : 3;
    }

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions plain;
    rclcpp::NodeOptions ipc;
    ipc.use_intra_process_comms(true);

    rclcpp::Node::SharedPtr node;
    if (mode == "talker") {
        node = std::make_shared<Talker>(plain);
    } else if (mode == "stall_pub") {
        const int stall_ms = argc > 2 ? std::atoi(argv[2]) : 800;
        const double stall_at_s = argc > 3 ? std::atof(argv[3]) : 7.0;
        node = std::make_shared<StallTalker>(plain, stall_ms, stall_at_s);
    } else if (mode == "listener") {
        node = std::make_shared<Listener>(plain);
    } else if (mode == "talker_fixed") {
        node = std::make_shared<FixedTalker>(plain);
    } else if (mode == "listener_fixed") {
        node = std::make_shared<FixedListener>(plain);
    } else if (mode == "intra") {
        node = std::make_shared<IntraNode>(ipc);
    } else if (mode == "exit_storm") {
        // Straggler-tracepoint exit hazard (KNOWN_ISSUES #9): a node init arms the probe, then
        // detached threads hammer the interposers with unknown handles and are STILL RUNNING
        // when main() returns and static destruction begins. The threads never touch rcl, so a
        // non-zero exit (SIGSEGV/SIGABRT) can only come from the probe's teardown behaviour.
        node = std::make_shared<rclcpp::Node>("exit_storm", plain);
        for (int t = 0; t < 4; ++t) {
            std::thread([] {
                while (true) {
                    ros_trace_rcl_publish(reinterpret_cast<const void*>(0x5751), nullptr);
                    ros_trace_callback_start(reinterpret_cast<const void*>(0x5752), false);
                }
            }).detach();
        }
        std::this_thread::sleep_for(400ms);
        rclcpp::shutdown();
        return 0;
    } else {
        fprintf(stderr, "unknown mode %s\n", mode.c_str());
        return 2;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
