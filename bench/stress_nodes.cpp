// Multi-topic stress harness for ros2_pulse overhead measurement.
// Each process runs for a fixed duration, then prints its own CPU cost (getrusage) + message
// counts so we can diff probe-on vs probe-off precisely without external tooling.
//
// Modes:
//   pubfarm  <dur> <n_light> <n_heavy>   inter-process publishers: light=small@100Hz, heavy=~100KB@50Hz
//   subfarm  <dur> <n_light> <n_heavy>   subscribes to all pubfarm topics (MultiThreadedExecutor)
//   intrafarm<dur> <n>                   single proc, n topics pub+sub intra-process @100Hz
#include <sys/resource.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::ByteMultiArray;
using std_msgs::msg::String;

static auto qos() -> rclcpp::QoS { return rclcpp::QoS(rclcpp::KeepLast(10)); }

static void printCpu(const char* tag, uint64_t msgs) {
    struct rusage ru {};
    getrusage(RUSAGE_SELF, &ru);
    double cpu = (ru.ru_utime.tv_sec + ru.ru_utime.tv_usec / 1e6) +
                 (ru.ru_stime.tv_sec + ru.ru_stime.tv_usec / 1e6);
    std::fprintf(stderr, "RESULT %s cpu_s=%.3f msgs=%llu\n", tag, cpu,
                 static_cast<unsigned long long>(msgs));
}

class PubFarm : public rclcpp::Node {
public:
    PubFarm(int n_light, int n_heavy) : Node("pubfarm") {
        // Reentrant group: under MultiThreadedExecutor::spin() the two timers can fire on
        // separate worker threads concurrently, genuinely exercising the probe under contention.
        m_cbg = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        for (int i = 0; i < n_light; i++) {
            auto p = create_publisher<String>("/light_" + std::to_string(i), qos());
            m_light.push_back(p);
        }
        for (int i = 0; i < n_heavy; i++) {
            auto p = create_publisher<ByteMultiArray>("/heavy_" + std::to_string(i), qos());
            m_heavy.push_back(p);
        }
        m_heavy_payload.assign(100 * 1024, 0);  // ~100 KB
        m_t_light = create_wall_timer(
            10ms,
            [this]() {  // 100 Hz
                String m;
                m.data = "x";
                for (auto& p : m_light) {
                    p->publish(m);
                    m_msgs.fetch_add(1, std::memory_order_relaxed);
                }
            },
            m_cbg);
        m_t_heavy = create_wall_timer(
            20ms,
            [this]() {  // 50 Hz
                ByteMultiArray m;
                m.data = m_heavy_payload;
                for (auto& p : m_heavy) {
                    p->publish(m);
                    m_msgs.fetch_add(1, std::memory_order_relaxed);
                }
            },
            m_cbg);
    }
    std::atomic<uint64_t> m_msgs{0};

private:
    rclcpp::CallbackGroup::SharedPtr m_cbg;
    std::vector<rclcpp::Publisher<String>::SharedPtr> m_light;
    std::vector<rclcpp::Publisher<ByteMultiArray>::SharedPtr> m_heavy;
    std::vector<uint8_t> m_heavy_payload;
    rclcpp::TimerBase::SharedPtr m_t_light, m_t_heavy;
};

class SubFarm : public rclcpp::Node {
public:
    SubFarm(int n_light, int n_heavy) : Node("subfarm") {
        // Reentrant group so subscription callbacks run concurrently across worker threads.
        m_cbg = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions opts;
        opts.callback_group = m_cbg;
        for (int i = 0; i < n_light; i++) {
            m_subs_l.push_back(create_subscription<String>(
                "/light_" + std::to_string(i), qos(),
                [this](String::SharedPtr) { m_msgs.fetch_add(1, std::memory_order_relaxed); },
                opts));
        }
        for (int i = 0; i < n_heavy; i++) {
            m_subs_h.push_back(create_subscription<ByteMultiArray>(
                "/heavy_" + std::to_string(i), qos(),
                [this](ByteMultiArray::SharedPtr) { m_msgs.fetch_add(1, std::memory_order_relaxed); },
                opts));
        }
    }
    std::atomic<uint64_t> m_msgs{0};

private:
    rclcpp::CallbackGroup::SharedPtr m_cbg;
    std::vector<rclcpp::Subscription<String>::SharedPtr> m_subs_l;
    std::vector<rclcpp::Subscription<ByteMultiArray>::SharedPtr> m_subs_h;
};

class IntraFarm : public rclcpp::Node {
public:
    explicit IntraFarm(int n) : Node("intrafarm", rclcpp::NodeOptions().use_intra_process_comms(true)) {
        // Reentrant group so the timer and the n subscription callbacks can run in parallel.
        m_cbg = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions opts;
        opts.callback_group = m_cbg;
        for (int i = 0; i < n; i++) {
            auto topic = "/intra_" + std::to_string(i);
            m_pubs.push_back(create_publisher<String>(topic, qos()));
            m_subs.push_back(create_subscription<String>(
                topic, qos(),
                [this](String::SharedPtr) { m_msgs.fetch_add(1, std::memory_order_relaxed); }, opts));
        }
        m_timer = create_wall_timer(
            10ms,
            [this]() {  // 100 Hz each
                for (auto& p : m_pubs) {
                    auto m = std::make_unique<String>();
                    m->data = "x";
                    p->publish(std::move(m));
                    m_pub_msgs.fetch_add(1, std::memory_order_relaxed);
                }
            },
            m_cbg);
    }
    std::atomic<uint64_t> m_msgs{0}, m_pub_msgs{0};

private:
    rclcpp::CallbackGroup::SharedPtr m_cbg;
    std::vector<rclcpp::Publisher<String>::SharedPtr> m_pubs;
    std::vector<rclcpp::Subscription<String>::SharedPtr> m_subs;
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string mode = argc > 1 ? argv[1] : "pubfarm";
    int dur = argc > 2 ? std::atoi(argv[2]) : 10;

    auto spin_until = [&](rclcpp::Node::SharedPtr node, auto* farm, const char* tag) {
        // Event-driven: spin() blocks/sleeps between timer fires (no busy-wait) and dispatches
        // Reentrant callbacks across worker threads. A one-shot wall timer cancels after `dur`,
        // so getrusage() reflects real work + probe cost, not empty-poll CPU.
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(node);
        auto cancel_timer =
            node->create_wall_timer(std::chrono::seconds(dur), [&exec]() { exec.cancel(); });
        exec.spin();
        uint64_t msgs = farm->m_msgs.load(std::memory_order_relaxed);
        printCpu(tag, msgs);
    };

    if (mode == "pubfarm") {
        int nl = argc > 3 ? std::atoi(argv[3]) : 20;
        int nh = argc > 4 ? std::atoi(argv[4]) : 5;
        auto n = std::make_shared<PubFarm>(nl, nh);
        spin_until(n, n.get(), "pubfarm");
    } else if (mode == "subfarm") {
        int nl = argc > 3 ? std::atoi(argv[3]) : 20;
        int nh = argc > 4 ? std::atoi(argv[4]) : 5;
        auto n = std::make_shared<SubFarm>(nl, nh);
        spin_until(n, n.get(), "subfarm");
    } else if (mode == "intrafarm") {
        int nn = argc > 3 ? std::atoi(argv[3]) : 10;
        auto n = std::make_shared<IntraFarm>(nn);
        spin_until(n, n.get(), "intrafarm");
    } else {
        std::fprintf(stderr, "unknown mode %s\n", mode.c_str());
        return 2;
    }
    rclcpp::shutdown();
    return 0;
}
