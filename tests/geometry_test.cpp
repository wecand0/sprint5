#include "geometry.hpp"
#include <gtest/gtest.h>

using namespace geometry;

TEST(geometry_test, point) {
    Point2D a{1., 2.};
    Point2D b{10., 20.};

    {
        Point2D actual = a + b;
        Point2D expected = {11, 22};
        EXPECT_EQ(actual, expected);
    }

    {
        Point2D actual = b - a;
        Point2D expected = {9, 18};
        EXPECT_EQ(actual, expected);
    }

    {
        Point2D actual = a * 3.;
        Point2D expected = {3, 6};
        EXPECT_EQ(actual, expected);
    }

    {
        Point2D actual = b / 2.;
        Point2D expected = {5, 10};
        EXPECT_EQ(actual, expected);
    }
}

TEST(geometry_test, line) {
    Line l{{1., 1.}, {3., 3.}};

    {
        auto actual = std::sqrt(8);
        auto expected = l.Length();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = Point2D{2., 2.};
        auto expected = l.Direction();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = BoundingBox{1., 1., 3., 3.};
        auto expected = l.BoundBox();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = 2.;
        auto expected = l.Height();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = Point2D{2., 2.};
        auto expected = l.Center();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = std::array<Point2D, 2>{l.start, l.end};
        auto expected = l.Vertices();
        EXPECT_EQ(actual, expected);
    }
}

TEST(geometry_test, rectangle) {
    Rectangle r{{0., 0.}, 10., 20.};

    {
        auto actual = 200.;
        auto expected = r.Area();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = BoundingBox{0., 0., 10., 20.};
        auto expected = r.BoundBox();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = 20.;
        auto expected = r.Height();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = Point2D{5., 10.};
        auto expected = r.Center();
        EXPECT_EQ(actual, expected);
    }
}

TEST(geometry_test, triangle) {
    Triangle t{{0., 0.}, {2., 0.}, {1., 1.}};

    {
        auto actual = 1.;
        auto expected = t.Area();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = BoundingBox{0., 0., 2., 1.};
        auto expected = t.BoundBox();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = 1.;
        auto expected = t.Height();
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = Point2D{1., 1. / 3.};
        auto expected = t.Center();
        EXPECT_EQ(actual, expected);
    }
}
