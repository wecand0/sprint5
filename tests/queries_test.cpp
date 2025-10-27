#include "geometry.hpp"
#include "queries.hpp"
#include <gtest/gtest.h>

using namespace geometry;
using namespace geometry::queries;

TEST(queries_test, distance_point_to_line) {
    Point2D p{30., 30.};

    {
        Line l{{10., 10.}, {50., 10.}};
        auto actual = 20.;
        auto expected = DistanceToPoint(l, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Line l{{50., 10.}, {100., 10.}};
        auto actual = std::sqrt(800);
        auto expected = DistanceToPoint(l, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Line l{{20., 30.}, {40., 30.}};
        auto actual = 0.;
        auto expected = DistanceToPoint(l, p);
        EXPECT_EQ(actual, expected);
    }
}

TEST(queries_test, distance_point_to_circle) {
    Point2D p{30., 30.};

    {
        Circle c{{30., 30.}, 10.};
        auto actual = 0.;
        auto expected = DistanceToPoint(c, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Circle c{{25., 25.}, 10.};
        auto actual = 0.;
        auto expected = DistanceToPoint(c, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Circle c{{50., 30.}, 20.};
        auto actual = 0.;
        auto expected = DistanceToPoint(c, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Circle c{{100., 30.}, 20.};
        auto actual = 50.;
        auto expected = DistanceToPoint(c, p);
        EXPECT_EQ(actual, expected);
    }
}

TEST(queries_test, distance_point_to_rect) {
    Point2D p{30., 30.};

    {
        Rectangle r{{20., 20.}, 20., 20.};
        auto actual = 0.;
        auto expected = DistanceToPoint(r, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Rectangle r{{25., 25.}, 20., 20.};
        auto actual = 0.;
        auto expected = DistanceToPoint(r, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Rectangle r{{30., 20.}, 20., 20};
        auto actual = 0.;
        auto expected = DistanceToPoint(r, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Rectangle r{{50., 20.}, 20., 20};
        auto actual = 20.;
        auto expected = DistanceToPoint(r, p);
        EXPECT_EQ(actual, expected);
    }
}

TEST(queries_test, distance_point_to_triangle) {
    Point2D p{30., 30.};

    {
        Triangle t{{20., 20.}, {20., 40.}, {40., 20.}};
        auto actual = 0.;
        auto expected = DistanceToPoint(t, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Triangle t{{30., 20.}, {30., 50.}, {50., 20.}};
        auto actual = 0.;
        auto expected = DistanceToPoint(t, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Triangle t{{40., 20.}, {40., 50.}, {60., 20.}};
        auto actual = 10.;
        auto expected = DistanceToPoint(t, p);
        EXPECT_EQ(actual, expected);
    }
}

TEST(queries_test, distance_point_to_polygon) {
    Point2D p{30., 30.};

    {
        Polygon poly{{{20., 20.}, {20., 40.}, {40., 20.}}};
        auto actual = 0.;
        auto expected = DistanceToPoint(poly, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Polygon poly{{{30., 20.}, {30., 50.}, {50., 20.}}};
        auto actual = 0.;
        auto expected = DistanceToPoint(poly, p);
        EXPECT_EQ(actual, expected);
    }

    {
        Polygon poly{{{40., 20.}, {40., 50.}, {60., 20.}}};
        auto actual = 10.;
        auto expected = DistanceToPoint(poly, p);
        EXPECT_EQ(actual, expected);
    }
}

TEST(queries_test, get_bound_box) {
    {
        auto shape = Circle{{30., 30.}, 10.};
        auto actual = BoundingBox{20., 20., 40., 40.};
        auto expected = GetBoundBox(shape);
        EXPECT_EQ(actual, expected);
    }

    {
        auto shape = Triangle{{30., 30.}, {40., 50.}, {50., 30.}};
        auto actual = BoundingBox{30., 30., 50., 50.};
        auto expected = GetBoundBox(shape);
        EXPECT_EQ(actual, expected);
    }
}

TEST(queries_test, get_height) {
    {
        auto shape = Circle{{30., 30.}, 10.};
        auto actual = 20.;
        auto expected = GetHeight(shape);
        EXPECT_EQ(actual, expected);
    }

    {
        auto shape = Triangle{{30., 30.}, {40., 50.}, {50., 30.}};
        auto actual = 20.;
        auto expected = GetHeight(shape);
        EXPECT_EQ(actual, expected);
    }
}

TEST(queries_test, bound_box_overlap) {
    Circle c{{30., 30.}, 10.};

    {
        auto shape = Circle{{40, 30.}, 10.};
        EXPECT_TRUE(BoundingBoxesOverlap(shape, c));
    }

    {
        auto shape = Circle{{50., 30.}, 10.};
        EXPECT_TRUE(BoundingBoxesOverlap(shape, c));
    }

    {
        auto shape = Circle{{60., 30.}, 10.};
        EXPECT_FALSE(BoundingBoxesOverlap(shape, c));
    }
}
