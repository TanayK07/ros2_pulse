// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the bounded spec-file reader (ROADMAP R1). The probe loads specs inside a
// tracepoint-reached constructor, so an operator typo in ROS_TOPIC_STATS_EXPECTED must degrade,
// never stall or balloon, the host process. Touches the filesystem (unlike test_rate_spec.cpp,
// which stays pure) but needs no ROS. Registers into the shared test binary (no main()).

#include <gtest/gtest.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ros2_pulse/core/rate_spec.hpp"

using ros2_pulse::core::kMaxSpecBytes;
using ros2_pulse::core::parseRateSpec;
using ros2_pulse::core::readSpecFile;

namespace {

// A unique scratch directory per test, removed on destruction.
class ScratchDir {
public:
    ScratchDir() {
        char tmpl[] = "/tmp/ros2_pulse_spec_io_XXXXXX";
        const char* made = ::mkdtemp(tmpl);
        m_dir = made != nullptr ? made : "";
    }
    ~ScratchDir() {
        if (!m_dir.empty()) {
            for (const auto& p : m_made) {
                ::unlink(p.c_str());
            }
            ::rmdir(m_dir.c_str());
        }
    }
    ScratchDir(const ScratchDir&) = delete;
    auto operator=(const ScratchDir&) -> ScratchDir& = delete;

    auto path(const std::string& name) -> std::string {
        const std::string p = m_dir + "/" + name;
        m_made.push_back(p);
        return p;
    }

    auto writeFile(const std::string& name, const std::string& body) -> std::string {
        const std::string p = path(name);
        std::FILE* f = std::fopen(p.c_str(), "w");
        EXPECT_NE(f, nullptr) << p;
        if (f != nullptr) {
            std::fwrite(body.data(), 1, body.size(), f);
            std::fclose(f);
        }
        return p;
    }

    auto dir() const -> const std::string& { return m_dir; }

private:
    std::string m_dir;
    std::vector<std::string> m_made;
};

}  // namespace

// The happy path: a real spec round-trips into the parser unchanged.
TEST(SpecIo, ReadsRegularFileAndParses) {
    ScratchDir d;
    const std::string body = "topics:\n  /scan: {min_hz: 18, max_hz: 22}\n";
    const std::string p = d.writeFile("good.yaml", body);

    std::string text;
    std::string err;
    ASSERT_TRUE(readSpecFile(p.c_str(), text, err)) << err;
    EXPECT_EQ(text, body);

    auto spec = parseRateSpec(text, err);
    ASSERT_TRUE(spec.has_value()) << err;
    EXPECT_EQ(spec->topics.size(), 1u);
}

// Exactly at the cap is still a spec; one byte past it is refused without being read.
TEST(SpecIo, AcceptsAtCapRejectsOverCap) {
    ScratchDir d;
    std::string text;
    std::string err;

    const std::string at_cap = d.writeFile("at_cap.yaml", std::string(kMaxSpecBytes, '#'));
    EXPECT_TRUE(readSpecFile(at_cap.c_str(), text, err)) << err;
    EXPECT_EQ(text.size(), kMaxSpecBytes);

    text.clear();
    const std::string over = d.writeFile("over.yaml", std::string(kMaxSpecBytes + 1, '#'));
    EXPECT_FALSE(readSpecFile(over.c_str(), text, err));
    EXPECT_NE(err.find("larger than"), std::string::npos) << err;
    EXPECT_TRUE(text.empty()) << "refused files must not be read into memory";
}

// A directory opens fine and reads EISDIR, which used to yield empty text that parses as a
// perfectly valid zero-rule spec, alerting silently armed as a permanent no-op.
TEST(SpecIo, RejectsDirectory) {
    ScratchDir d;
    std::string text;
    std::string err;
    EXPECT_FALSE(readSpecFile(d.dir().c_str(), text, err));
    EXPECT_NE(err.find("not a regular file"), std::string::npos) << err;
}

// /dev/zero never signals EOF: an unbounded read loop here never returns.
TEST(SpecIo, RejectsCharacterDeviceWithoutHanging) {
    std::string text;
    std::string err;
    EXPECT_FALSE(readSpecFile("/dev/zero", text, err));
    EXPECT_NE(err.find("not a regular file"), std::string::npos) << err;
    EXPECT_TRUE(text.empty());
}

// A FIFO with no writer blocks forever in a plain open(); O_NONBLOCK turns that into a refusal.
// If this test hangs, the O_NONBLOCK has been lost.
TEST(SpecIo, RejectsFifoWithoutBlocking) {
    ScratchDir d;
    const std::string p = d.path("spec.fifo");
    ASSERT_EQ(::mkfifo(p.c_str(), 0600), 0) << std::strerror(errno);

    std::string text;
    std::string err;
    EXPECT_FALSE(readSpecFile(p.c_str(), text, err));
    EXPECT_NE(err.find("not a regular file"), std::string::npos) << err;
}

// A missing path (the common typo) reports the OS reason, not a generic failure.
TEST(SpecIo, RejectsMissingPathWithErrno) {
    ScratchDir d;
    std::string text;
    std::string err;
    EXPECT_FALSE(readSpecFile((d.dir() + "/absent.yaml").c_str(), text, err));
    EXPECT_FALSE(err.empty());

    EXPECT_FALSE(readSpecFile(nullptr, text, err));
    EXPECT_FALSE(readSpecFile("", text, err));
}

// An empty spec file reads fine and parses fine, into zero rules. The reader's job is to hand
// that up; refusing to ARM a no-op spec is the caller's (see loadRateSpec).
TEST(SpecIo, EmptyFileReadsAndParsesToZeroRules) {
    ScratchDir d;
    const std::string p = d.writeFile("empty.yaml", "");

    std::string text;
    std::string err;
    ASSERT_TRUE(readSpecFile(p.c_str(), text, err)) << err;
    EXPECT_TRUE(text.empty());

    auto spec = parseRateSpec(text, err);
    ASSERT_TRUE(spec.has_value()) << err;
    EXPECT_TRUE(spec->topics.empty());
    EXPECT_TRUE(spec->nodes.empty());
}
