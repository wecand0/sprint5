#include "geometry.hpp"
#include "intersections.hpp"
#include <gtest/gtest.h>
#include <optional>

using namespace geometry;
using namespace geometry::intersections;

TEST(intersections, line_vs_line) {
    Line l1{{-20., -20.}, {+20., +20.}};
    Line l2{{-20., +20.}, {+20., -20.}};
    Line l3{{+30., +30.}, {+40., +40.}};

    {
        auto actual = Point2D{0., 0.};
        auto expected = GetIntersectPoint(l1, l2);
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = std::nullopt;
        auto expected = GetIntersectPoint(l1, l1);
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = std::nullopt;
        auto expected = GetIntersectPoint(l1, l3);
        EXPECT_EQ(actual, expected);
    }
}

TEST(intersections, line_vs_circle) {
    Circle c{{100., 100.}, 100.};

    {
        Line l{{100., 0.}, {100., 0.}};
        auto actual = Point2D{100., 0.};
        auto expected = GetIntersectPoint(c, l);
        EXPECT_EQ(actual, expected);
    }

    {
        Line l{{0., 100.}, {200., 100.}};
        auto actual = Point2D{0., 100.};  // Point2D{100., 0.};
        auto expected = GetIntersectPoint(c, l);
        EXPECT_EQ(actual, expected);
    }

    {
        Line l{{-10., 100.}, {10., 100.}};
        auto actual = Point2D{0., 100.};
        auto expected = GetIntersectPoint(c, l);
        EXPECT_EQ(actual, expected);
    }

    {
        Line l{{0., 300.}, {200., 300.}};
        auto actual = std::nullopt;
        auto expected = GetIntersectPoint(c, l);
        EXPECT_EQ(actual, expected);
    }
}

TEST(intersection_test, circle_vs_circle) {
    Circle c1{{100., 100.}, 100.};

    {
        Circle c2{{400., 100.}, 100.};
        auto actual = std::nullopt;
        auto expected = GetIntersectPoint(c1, c2);
        EXPECT_EQ(actual, expected);
    }

    {
        Circle c2{{300., 100.}, 100.};
        auto actual = Point2D{200, 100};
        auto expected = GetIntersectPoint(c1, c2);
        EXPECT_EQ(actual, expected);
    }

    {
        Circle c2{{500., 100.}, 100.};
        auto actual = std::nullopt;
        auto expected = GetIntersectPoint(c1, c2);
        EXPECT_EQ(actual, expected);
    }

    {
        Circle c2{{100., 100.}, 50.};
        auto actual = std::nullopt;
        auto expected = GetIntersectPoint(c1, c2);
        EXPECT_EQ(actual, expected);
    }

    {
        Circle c2{{100., 100.}, 100.};
        auto actual = std::nullopt;
        auto expected = GetIntersectPoint(c1, c2);
        EXPECT_EQ(actual, expected);
    }
}

TEST(intersection_test, unsupported_shapes) {
    Triangle t{};
    Line l{};

    // clang-format off
    EXPECT_THROW(
        try {
            GetIntersectPoint(t, l);
        } catch (const std::logic_error &e) {
            EXPECT_STREQ("search intersection for these shapes is not supported", e.what());
            throw;
        },
        std::logic_error
    );
    // clang-format on
}