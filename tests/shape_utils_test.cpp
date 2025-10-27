#include "shape_utils.hpp"
#include <gtest/gtest.h>
#include <optional>

using namespace geometry;
using namespace geometry::utils;

class shape_utils_test : public ::testing::Test {
protected:
    const Shape c = Circle{{9., 10.}, 5.};
    const Shape t = Triangle{{10., 10.}, {20., 40.}, {30., 10.}};
    const Shape r = Rectangle{{31., 10.}, 10., 31};
};

TEST_F(shape_utils_test, find_collisions) {
    auto shapes = std::vector<Shape>{c, t, r};

    {
        auto actual = std::vector<std::pair<Shape, Shape>>{{c, t}};
        auto expected = FindAllCollisions(shapes);
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = std::vector<std::pair<Shape, Shape>>{};
        auto expected = FindAllCollisions({});
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = std::vector<std::pair<Shape, Shape>>{{t, c}};
        auto expected = FindAllCollisions(shapes);
        EXPECT_NE(actual, expected);
    }

    {
        auto actual = std::vector<std::pair<Shape, Shape>>{{c, t}, {c, r}};
        auto expected = FindAllCollisions(shapes);
        EXPECT_NE(actual, expected);
    }
}

TEST_F(shape_utils_test, find_highest) {
    auto shapes = std::vector<Shape>{c, t, r};

    {
        auto actual = std::optional<size_t>{2};
        auto expected = FindHighestShape(shapes);
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = std::nullopt;
        auto expected = FindHighestShape({});
        EXPECT_EQ(actual, expected);
    }

    {
        auto actual = std::optional<size_t>{1};
        auto expected = FindHighestShape(shapes);
        EXPECT_NE(actual, expected);
    }
}