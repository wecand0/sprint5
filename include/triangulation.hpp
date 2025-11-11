#pragma once
#include "convex_hull.hpp"
#include "geometry.hpp"
#include <algorithm>
#include <cassert>
#include <flat_map>
#include <vector>

namespace geometry::triangulation {

struct DelaunayTriangle {
    Point2D a, b, c;

    DelaunayTriangle() {}
    DelaunayTriangle(Point2D a, Point2D b, Point2D c) : a(a), b(b), c(c) {}

    bool ContainsPoint(const Point2D &p) const {
        Point2D center = Circumcenter();
        double radius = Circumradius();
        return center.DistanceTo(p) <= radius + 1e-10;
    }

    Point2D Circumcenter() const {
        double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        if (std::abs(d) < 1e-10) {
            return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3};
        }

        double ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) +
                     (c.x * c.x + c.y * c.y) * (a.y - b.y)) /
                    d;

        double uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) +
                     (c.x * c.x + c.y * c.y) * (b.x - a.x)) /
                    d;

        return {ux, uy};
    }

    double Circumradius() const {
        Point2D center = Circumcenter();
        return center.DistanceTo(a);
    }

    bool SharesEdge(const DelaunayTriangle &other) const {
        std::vector<Point2D> this_points = {a, b, c};
        std::vector<Point2D> other_points = {other.a, other.b, other.c};

        int shared_count = 0;
        for (const Point2D &p1 : this_points) {
            for (const Point2D &p2 : other_points) {
                if (std::abs(p1.x - p2.x) < 1e-10 && std::abs(p1.y - p2.y) < 1e-10) {
                    shared_count++;
                    break;
                }
            }
        }

        return shared_count == 2;
    }

    bool operator==(const DelaunayTriangle &other) const noexcept {
        return std::tie(a, b, c) == std::tie(other.a, other.b, other.c);
    };

    std::vector<Point2D> vertices() const { return {a, b, c}; }
};

struct Edge {
    Point2D p1, p2;

    Edge(Point2D p1, Point2D p2) : p1(p1), p2(p2) {
        if (p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y)) {
            std::swap(this->p1, this->p2);
        }
    }

    bool operator<(const Edge &other) const {
        if (std::abs(p1.x - other.p1.x) > 1e-10)
            return p1.x < other.p1.x;
        if (std::abs(p1.y - other.p1.y) > 1e-10)
            return p1.y < other.p1.y;
        if (std::abs(p2.x - other.p2.x) > 1e-10)
            return p2.x < other.p2.x;
        return p2.y < other.p2.y;
    }

    bool operator==(const Edge &other) const {
        return std::abs(p1.x - other.p1.x) < 1e-10 && std::abs(p1.y - other.p1.y) < 1e-10 &&
               std::abs(p2.x - other.p2.x) < 1e-10 && std::abs(p2.y - other.p2.y) < 1e-10;
    }
};

inline GeometryResult<std::vector<DelaunayTriangle>> DelaunayTriangulation(std::span<const Point2D> points) {

    if (points.size() < 3) {
        return std::unexpected(GeometryError::InsufficientPoints);
    }

    std::vector<DelaunayTriangle> triangulation;

    // большой треугольник, содержащий все точки
    DelaunayTriangle super_triangle{};
    {
        auto hull = convex_hull::GrahamScan(points);
        assert(hull.has_value());

        const auto poly_hull = Polygon(std::move(*hull));
        const auto bb_hull = poly_hull.BoundBox();
        const auto bb_center = bb_hull.Center();

        // радиус окружности вписанной в правильный треугольник = (a * sqrt(3)) / 6
        const auto r_in = bb_hull.Center().DistanceTo({bb_hull.min_x, bb_hull.min_y});

        // радиус окружности описанной около правильного треугольника = (a * sqrt(3)) / 3
        const auto r_out = 2. * r_in;

        // треугольник в описанной около него окружности
        const auto rp_three_sides = RegularPolygon(bb_center, r_out, 3);
        const auto vts = rp_three_sides.Vertices();
        triangulation.emplace_back(vts[0], vts[1], vts[2]);
        std::tie(super_triangle.a, super_triangle.b, super_triangle.c) = std::tie(vts[0], vts[1], vts[2]);
    }

    for (const Point2D &point : points) {

        auto bad_triangles =
            triangulation |
            std::views::filter([&point](const auto &triangle) { return triangle.ContainsPoint(point); }) |
            std::ranges::to<std::vector>();
        if (bad_triangles.empty()) {
            continue;
        }

        // поиск общих ребер
        std::flat_map<Edge, int> counting_edges;
        for (const auto &triangle : bad_triangles) {
            auto vts = triangle.vertices();
            ++counting_edges[Edge(vts[0], vts[1])];
            ++counting_edges[Edge(vts[1], vts[2])];
            ++counting_edges[Edge(vts[2], vts[0])];
        }

        // удаление из триангуляции всех треугольников с точками внутри
        std::erase_if(triangulation, [&bad_triangles](const DelaunayTriangle &triangle) {
            return std::ranges::any_of(bad_triangles, [&](const DelaunayTriangle &bad_triangle) {
                return std::tie(triangle.a, triangle.b, triangle.c) ==
                       std::tie(bad_triangle.a, bad_triangle.b, bad_triangle.c);
            });
        });

        // ребра-границы "дырки" -- одно общее ребро
        auto hole_edges =
            counting_edges | std::views::filter([](const auto &pair) { return pair.second == 1; }) | std::views::keys;

        for (const auto &edge : hole_edges) {
            triangulation.emplace_back(edge.p1, edge.p2, point);
        }
    }

    std::erase_if(triangulation, [&super_triangle](const DelaunayTriangle &tri) {
        auto has_super_vertex = [&super_triangle](const Point2D &v) {
            return v == super_triangle.a || v == super_triangle.b || v == super_triangle.c;
        };
        return has_super_vertex(tri.a) || has_super_vertex(tri.b) || has_super_vertex(tri.c);
    });

    return triangulation;
}
}  // namespace geometry::triangulation

template <>
struct std::formatter<geometry::triangulation::DelaunayTriangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::triangulation::DelaunayTriangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "DelaunayTriangle({}, {}, {})", t.a, t.b, t.c);
    }
};
