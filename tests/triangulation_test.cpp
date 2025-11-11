#include "triangulation.hpp"
#include <gtest/gtest.h>

using namespace geometry;
using namespace geometry::triangulation;

TEST(triangulation_test, delaunay_good) {
    std::vector<Point2D> points = {{0., 0.}, {100., 0.}, {100., 100.}, {0., 100.}};

    auto actual =
        std::vector<DelaunayTriangle>{{{0., 0.}, {100., 0.}, {0., 100.}}, {{100., 0.}, {100., 100.}, {0., 100.}}};
    auto expected = DelaunayTriangulation(points);
    EXPECT_TRUE(expected.has_value());
    EXPECT_EQ(actual, expected);
}

TEST(triangulation_test, delaunay_fail) {
    std::vector<Point2D> points = {{0., 0.}, {100., 0.}};

    auto actual = GeometryError::InsufficientPoints;
    auto expected = DelaunayTriangulation(points);
    EXPECT_FALSE(expected.has_value());
    EXPECT_EQ(actual, expected.error());
}
