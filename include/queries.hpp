#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <cassert>
#include <optional>
#include <variant>

namespace geometry::queries {

namespace vs = std::views;

/*
 * Класс для поиска расстояния от точки до фигуры
 *
 * Требуется организовать возможность нахождения расстояния для всех возможных фигур типа-суммы Shape
 */
struct PointToShapeDistanceVisitor {
    Point2D point;

    explicit PointToShapeDistanceVisitor(const Point2D &p) : point(p) {}

    double operator()(const Line &line) const {
        const auto vl = line.Direction();
        const auto vp = point - line.start;
        const auto norm_vl_sq = vl.Dot(vl);

        if (is_equal_zero(norm_vl_sq)) {
            return point.DistanceTo(line.start);
        }

        // формула Хари
        const auto t = std::clamp(vp.Dot(vl) / norm_vl_sq, 0.0, 1.0);
        const auto proj = line.start - point + vl * t;
        const auto l = std::sqrt(proj.Dot(proj));
        return l;
    }

    double operator()(const Circle &circle) const {
        const auto r = std::abs(circle.radius);
        const auto dist = point.DistanceTo(circle.center_p);
        return std::max(0.0, dist - r);
    }

    double operator()(const Rectangle &rect) const {
        const auto bb = rect.BoundBox();

        if (bb.min_x <= point.x && point.x <= bb.max_x && bb.min_y <= point.y && point.y <= bb.max_y) {
            return 0.0;
        }

        const auto distances = rect.Edges() | vs::transform([this](const Line &e) { return operator()(e); });
        return std::ranges::min(distances);
    }

    double operator()(const Triangle &triangle) const {
        const auto pts = triangle.Vertices();
        std::vector<double> vprods(pts.size());
        for (size_t i = 0, j = pts.size() - 1; i < pts.size(); j = i++) {
            const auto v1 = pts[j] - pts[i];
            const auto v2 = point - pts[i];
            vprods[i] = v1.Cross(v2);
        }

        bool is_inside = std::ranges::all_of(vprods, [](double vprod) { return vprod <= 0; }) ||
                         std::ranges::all_of(vprods, [](double vprod) { return vprod >= 0; });

        if (is_inside) {
            return 0.0;
        }

        const auto distances = triangle.Edges() | vs::transform([this](const Line &e) { return operator()(e); });
        return std::ranges::min(distances);
    }

    double operator()(const RegularPolygon &poly) const {
        const auto pts = poly.Vertices();
        return operator()(Polygon{pts});
    }

    double operator()(const Polygon &poly) const {
        const auto &pts = poly.Vertices();
        if (pts.empty()) {
            return std::numeric_limits<double>::infinity();
        }

        if (pts.size() == 1) {
            return point.DistanceTo(pts[0]);
        }

        bool is_inside = false;
        for (size_t i = 0, j = pts.size() - 1; i < pts.size(); j = i++) {
            if (((pts[i].y > point.y) != (pts[j].y > point.y)) &&
                (point.x < (pts[j].x - pts[i].x) * (point.y - pts[i].y) / (pts[j].y - pts[i].y) + pts[i].x)) {
                is_inside = !is_inside;
            }
        }
        if (is_inside) {
            return 0.0;
        }

        auto distances = poly.Edges() | vs::transform([this](const Line &e) { return operator()(e); });
        return std::ranges::min(distances);
    }
};

/*
 * Класс для поиска расстояния между двумя фигурами
 *
 * Требуется организовать возможность нахождения расстояния только для следующих комбинаций фигур:
 *    - Any    & Point
 *    - Line   & Line
 *    - Circle & Circle
 *
 * Важно: вы можете выбрать любой метод нахождения расстояния, даже если он даёт не точный результат
 *
 * Для всех остальных требуется вернуть пустое значение
 */
struct ShapeToShapeDistanceVisitor {

    using Distance = std::optional<double>;

    Distance operator()(const Circle &c1, const Circle &c2) const {
        const auto dist_centers = c1.center_p.DistanceTo(c2.center_p);
        const auto dist_borders = dist_centers - (c1.radius + c2.radius);
        return std::max(0.0, dist_borders);
    }

    Distance operator()(const Line &l1, const Line &l2) const {
        const std::array<double, 4> distances = {
            PointToShapeDistanceVisitor{l1.start}(l2),
            PointToShapeDistanceVisitor{l1.end}(l2),
            PointToShapeDistanceVisitor{l2.start}(l1),
            PointToShapeDistanceVisitor{l2.end}(l1),
        };

        const auto min_it = std::ranges::min_element(distances);
        assert(min_it != distances.end());

        return *min_it;
    }

    Distance operator()(auto &&, auto &&) const { return std::nullopt; }
};

/*
 * Функции-помощники
 */
inline double DistanceToPoint(const Shape &shape, const Point2D &point) {
    return std::visit(PointToShapeDistanceVisitor{point}, shape);
}

inline BoundingBox GetBoundBox(const Shape &shape) {
    return std::visit([](const auto &s) { return s.BoundBox(); }, shape);
}

inline double GetHeight(const Shape &shape) { return GetBoundBox(shape).Height(); }

inline bool BoundingBoxesOverlap(const Shape &shape1, const Shape &shape2) {
    const auto bb1 = GetBoundBox(shape1);
    const auto bb2 = GetBoundBox(shape2);
    return bb1.Overlaps(bb2);
}

inline std::optional<double> DistanceBetweenShapes(const Shape &shape1, const Shape &shape2) {
    return std::visit(ShapeToShapeDistanceVisitor{}, shape1, shape2);
}

}  // namespace geometry::queries