// Copyright 2026 ros2_pulse contributors
//
// Unit tests for the tracetools version check (probe startup warning on an untested distro).
// No ROS / tracetools dependency.

#include <gtest/gtest.h>

#include "ros2_pulse/core/tracetools_version.hpp"

using ros2_pulse::core::isTestedTracetoolsMajor;
using ros2_pulse::core::parsePackageVersion;
using ros2_pulse::core::tracetoolsPackageXmlPath;
using ros2_pulse::core::versionMajor;

TEST(TracetoolsVersion, ParsesHumblePackageXml) {
    const char* xml =
        "<?xml version=\"1.0\"?>\n"
        "<package format=\"3\">\n"
        "  <name>tracetools</name>\n"
        "  <version>4.1.2</version>\n"
        "</package>\n";
    EXPECT_EQ(parsePackageVersion(xml), "4.1.2");
}

TEST(TracetoolsVersion, TrimsWhitespaceInsideTheTag) {
    EXPECT_EQ(parsePackageVersion("<version>\n  8.2.6 \n</version>"), "8.2.6");
}

TEST(TracetoolsVersion, RejectsMissingOrEmptyVersion) {
    EXPECT_FALSE(parsePackageVersion("<package><name>x</name></package>").has_value());
    EXPECT_FALSE(parsePackageVersion("<version>4.1.2").has_value());
    EXPECT_FALSE(parsePackageVersion("<version>  </version>").has_value());
    EXPECT_FALSE(parsePackageVersion("").has_value());
}

TEST(TracetoolsVersion, IgnoresTheXmlDeclarationVersionAttribute) {
    // `<?xml version="1.0"?>` is an attribute, not the tag; it must not be picked up.
    EXPECT_EQ(parsePackageVersion("<?xml version=\"1.0\"?><version>9.0.0</version>"), "9.0.0");
}

TEST(TracetoolsVersion, MajorOfReleasedVersions) {
    EXPECT_EQ(versionMajor("4.1.2"), 4);
    EXPECT_EQ(versionMajor("8.10.2"), 8);
    EXPECT_EQ(versionMajor("10.0.0"), 10);
    EXPECT_FALSE(versionMajor("").has_value());
    EXPECT_FALSE(versionMajor("v4").has_value());
    EXPECT_FALSE(versionMajor("99999999999").has_value());
}

TEST(TracetoolsVersion, TestedMajorsMatchCi) {
    EXPECT_TRUE(isTestedTracetoolsMajor(4));   // Humble
    EXPECT_TRUE(isTestedTracetoolsMajor(8));   // Jazzy, Kilted, Lyrical
    EXPECT_TRUE(isTestedTracetoolsMajor(9));   // Rolling
    EXPECT_FALSE(isTestedTracetoolsMajor(3));  // Galactic and older
    EXPECT_FALSE(isTestedTracetoolsMajor(10));
}

TEST(TracetoolsVersion, PackageXmlPathFromLibPath) {
    EXPECT_EQ(tracetoolsPackageXmlPath("/opt/ros/humble/lib/libtracetools.so"),
              "/opt/ros/humble/share/tracetools/package.xml");
    EXPECT_EQ(tracetoolsPackageXmlPath("/ws/install/tracetools/lib/libtracetools.so"),
              "/ws/install/tracetools/share/tracetools/package.xml");
    EXPECT_EQ(tracetoolsPackageXmlPath("libtracetools.so"), "");
    EXPECT_EQ(tracetoolsPackageXmlPath("/libtracetools.so"), "");
}
