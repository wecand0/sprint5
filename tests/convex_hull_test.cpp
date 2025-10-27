
#include "convex_hull.hpp"
#include <gtest/gtest.h>
#include <vector>

using namespace geometry;
using namespace geometry::convex_hull;

TEST(convex_hull_test, graham_scan_good) {
    std::vector<Point2D> points = {{50., 100.}, {55., 50.}, {100., 0.}, {50., 45.}, {0., 0.}, {45., 50.}};

    auto actual = std::vector<Point2D>{{0., 0.}, {100., 0.}, {50., 100.}};
    auto expected = GrahamScan(points);
    EXPECT_TRUE(expected.has_value());
    EXPECT_EQ(actual, expected);
}

TEST(convex_hull_test, graham_scan_fail) {
    std::vector<Point2D> points = {{50., 0.}, {55., 50.}};

    auto actual = GeometryError::InsufficientPoints;
    auto expected = GrahamScan(points);
    EXPECT_FALSE(expected.has_value());
    EXPECT_EQ(actual, expected.error());
}
