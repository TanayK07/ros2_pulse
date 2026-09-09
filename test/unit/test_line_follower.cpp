// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the incremental line reader behind pulse_bridge: poll() hands back only the
// complete, newly appended lines of a growing jsonl log, survives truncation (log rotation
// in place) and a file that does not exist yet. Registers into the shared test binary.

#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "ros2_pulse/core/line_follower.hpp"

using ros2_pulse::core::LineFollower;

namespace {

class TempFile {
public:
    TempFile() {
        char tmpl[] = "/tmp/pulse_follower_XXXXXX";
        const int fd = mkstemp(tmpl);
        EXPECT_GE(fd, 0);
        close(fd);
        m_path = tmpl;
    }
    ~TempFile() { std::remove(m_path.c_str()); }
    auto path() const -> const std::string& { return m_path; }
    void append(const std::string& s) const {
        std::ofstream f(m_path, std::ios::app | std::ios::binary);
        f << s;
    }
    void rewrite(const std::string& s) const {
        std::ofstream f(m_path, std::ios::trunc | std::ios::binary);
        f << s;
    }

private:
    std::string m_path;
};

}  // namespace

TEST(LineFollower, ReturnsOnlyCompleteLines) {
    TempFile f;
    f.append("{\"a\":1}\n{\"b\":2}\n{\"partial\"");
    LineFollower lf(f.path());
    const auto lines = lf.poll();
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(lines[0], "{\"a\":1}");
    EXPECT_EQ(lines[1], "{\"b\":2}");
}

TEST(LineFollower, HeldPartialLineCompletesOnNextPoll) {
    TempFile f;
    f.append("{\"a\":1}\n{\"par");
    LineFollower lf(f.path());
    ASSERT_EQ(lf.poll().size(), 1u);
    f.append("tial\":2}\n");
    const auto lines = lf.poll();
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "{\"partial\":2}");
}

TEST(LineFollower, SecondPollReturnsOnlyNewLines) {
    TempFile f;
    f.append("one\n");
    LineFollower lf(f.path());
    ASSERT_EQ(lf.poll().size(), 1u);
    EXPECT_TRUE(lf.poll().empty());
    f.append("two\nthree\n");
    const auto lines = lf.poll();
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(lines[0], "two");
    EXPECT_EQ(lines[1], "three");
}

TEST(LineFollower, TruncationRestartsFromTheTop) {
    TempFile f;
    f.append("old-1\nold-2\nold-3\n");
    LineFollower lf(f.path());
    ASSERT_EQ(lf.poll().size(), 3u);
    f.rewrite("new-1\n");  // rotated in place, shorter than the old offset
    const auto lines = lf.poll();
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "new-1");
}

TEST(LineFollower, MissingFileYieldsNothingAndNoThrow) {
    LineFollower lf("/nonexistent/dir/pulse.log");
    EXPECT_NO_THROW({ EXPECT_TRUE(lf.poll().empty()); });
}

TEST(LineFollower, FileAppearingLaterIsPickedUp) {
    TempFile f;
    const std::string path = f.path();
    std::remove(path.c_str());
    LineFollower lf(path);
    EXPECT_TRUE(lf.poll().empty());
    f.append("late\n");
    const auto lines = lf.poll();
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "late");
}
