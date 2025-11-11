#pragma once
#include "geometry.hpp"
#include <optional>
#include <variant>

namespace geometry::intersections {

using TwoPoints2D = std::array<Point2D, 2>;
using Intersection = std::variant<std::monostate, Point2D, TwoPoints2D>;

class IntersectionVisitor {
public:
    Intersection operator()(const Line &l1, const Line &l2) const {
        const auto p1 = l1.start;
        const auto p2 = l1.end;
        const auto p3 = l2.start;
        const auto p4 = l2.end;

        const auto dir_12 = p1 - p2;
        const auto dir_42 = p4 - p2;
        const auto dir_43 = p4 - p3;

        // u * dir_12 + v * dir_43 = dir_42
        const auto det = dir_12.Cross(dir_43);
        if (is_equal_zero(det)) {  // параллельны или на 1 прямой
            return std::monostate{};
        } else {  // пересекаются или нет
            // метод Крамера
            const auto det_u = dir_42.Cross(dir_43);
            const auto det_v = dir_12.Cross(dir_42);
            const auto u = det_u / det;
            const auto v = det_v / det;
            bool is_segments_intersect = ((0.0 <= u) && (u <= 1.0)) && ((0.0 <= v) && (v <= 1.0));
            if (is_segments_intersect) {
                const auto p = p1 * u + p2 * (1 - u);
                return p;
            }
        }
        return std::monostate{};
    }

    Intersection operator()(const Line &line, const Circle &circle) const {
        const auto r = std::abs(circle.radius);
        const auto [p1, p2] = line.Vertices();

        if (line.start == line.end) {
            const auto p = line.start;
            bool is_point_on_circle{p.x * p.x + p.y * p.y == r * r};
            return is_point_on_circle ? Intersection{p} : std::monostate{};
        }

        const auto a = p1.y - p2.y;
        const auto b = p2.x - p1.x;
        const auto c = -(p1.x * p2.y - p2.x * p1.y);

        const auto under_root = r * r * (a * a + b * b) - c * c;
        const auto a2b2 = (a * a + b * b);

        auto is_point_inside_segment = [&p1, &p2](const Point2D &p) -> bool {
            // на одной прямой
            // clang-format off
            return (std::min(p1.x, p2.x) <= p.x) && (p.x <= std::max(p1.x, p2.x)) && 
                   (std::min(p1.y, p2.y) <= p.y) && (p.y <= std::max(p2.y, p2.y));
            // clang-format on
        };

        std::vector<Point2D> points{};

        if (is_equal_zero(under_root)) {
            const Point2D p{a * c / a2b2, b * c / a2b2};
            if (is_point_inside_segment(p)) {
                points.push_back(p);
            }
        } else if (under_root > 0) {
            const Point2D p1{(a * c + b * std::sqrt(under_root)) / a2b2, (b * c - a * std::sqrt(under_root)) / a2b2};
            const Point2D p2{(a * c - b * std::sqrt(under_root)) / a2b2, (b * c + a * std::sqrt(under_root)) / a2b2};
            if (is_point_inside_segment(p1)) {
                points.push_back(p1);
            }
            if (is_point_inside_segment(p2)) {
                points.push_back(p2);
            }
        }

        if (points.empty()) {
            return std::monostate{};
        } else if (points.size() == 1) {
            return points.front();
        } else {
            return TwoPoints2D{points[0], points[1]};
        }
    }

    Intersection operator()(const Circle &c, const Line &l) const { return operator()(l, c); }

    Intersection operator()(const Circle &c1, const Circle &c2) const {
        const auto r1 = std::abs(c1.radius);
        const auto r2 = std::abs(c2.radius);
        const auto d = c1.center_p.DistanceTo(c2.center_p);

        if (d > r1 + r2) {
            return std::monostate{};
        }

        if (is_equal_zero(d)) {
            return std::monostate{};
        }

        const auto a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        const auto h_squared = r1 * r1 - a * a;
        const auto h = (h_squared > 0) ? std::sqrt(h_squared) : 0;

        const auto p_mid = c1.center_p + (c2.center_p - c1.center_p) * (a / d);

        if (is_equal_zero(h)) {
            return p_mid;
        }

        const auto v = (c2.center_p - c1.center_p).Normalize() * h;
        const auto p1 = Point2D{p_mid.x + v.y, p_mid.y - v.x};
        const auto p2 = Point2D{p_mid.x - v.y, p_mid.y + v.x};
        return TwoPoints2D{p1, p2};
    }

    Intersection operator()(auto &&, auto &&) const {
        throw std::logic_error{"search intersection for these shapes is not supported"};
    }
};

inline std::optional<Point2D> GetIntersectPoint(const Shape &shape1, const Shape &shape2) {
    const Intersection result = std::visit(IntersectionVisitor{}, shape1, shape2);
    return std::visit(
        [](auto &&arg) -> std::optional<Point2D> {
            using T = std::decay_t<decltype(arg)>;

            if constexpr (std::is_same_v<T, Point2D>) {
                return arg;
            }
            if constexpr (std::is_same_v<T, TwoPoints2D>) {
                return arg[0];
            }
            return std::nullopt;
        },
        result);
}

}  // namespace geometry::intersections