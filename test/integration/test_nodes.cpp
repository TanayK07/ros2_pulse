// Copyright 2026 ros2_pulse contributors
//
// Test nodes for the ros2_pulse integration test. Modes:
//   talker     : publishes std_msgs/String on /chatter at 50 Hz (separate process)
//   listener   : subscribes /chatter (separate process)          -> inter-process receive
//   intra      : single process, intra-process comms ON, self pub+sub on /intra_topic at 50 Hz
//                                                                -> intra-process receive
//   exit_storm : detached threads hammer the ros_trace_* interposers while main() returns
//                                                                -> KNOWN_ISSUES #9 exit hazard

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// exit_storm drives the probe's interposed tracepoints directly (they resolve to
// libros2_pulse.so under LD_PRELOAD, or to tracetools' real no-op functions otherwise). No rcl
// machinery is involved, so the ONLY thing that can crash at exit is the probe itself.
extern "C" void ros_trace_rcl_publish(const void* pub_handle, const void* message);
extern "C" void ros_trace_callback_start(const void* callback, bool is_intra_process);

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
    rclcpp::init(argc, argv);
    std::string mode = argc > 1 ? argv[1] : "talker";

    rclcpp::NodeOptions plain;
    rclcpp::NodeOptions ipc;
    ipc.use_intra_process_comms(true);

    rclcpp::Node::SharedPtr node;
    if (mode == "talker") {
        node = std::make_shared<Talker>(plain);
    } else if (mode == "listener") {
        node = std::make_shared<Listener>(plain);
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
