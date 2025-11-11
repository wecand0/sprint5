#include "convex_hull.hpp"
#include "geometry.hpp"
#include "intersections.hpp"
#include "queries.hpp"
#include "shape_utils.hpp"
#include "triangulation.hpp"
#include "visualization.hpp"

#include <algorithm>
#include <print>
#include <ranges>

using namespace geometry;

namespace rng = std::ranges;
namespace views = std::ranges::views;

void PrintAllIntersections(const Shape &shape, std::span<const Shape> others) {
    std::println("\n=== Intersections ===");

    rng::for_each(others | views::filter([&shape](const auto &other) { return &other != &shape; }),
                  [&](const Shape &other) {
                      try {
                          if (auto pt = intersections::GetIntersectPoint(shape, other)) {
                              std::println("  - {} vs {}: FOUND at {}", shape, other, *pt);
                          }
                      } catch (const std::logic_error &) {
                      }
                  });
}

void PrintDistancesFromPointToShapes(Point2D p, std::span<const Shape> shapes) {
    std::println("\n=== Distance from Point Test ===");
    std::println("  testing point: {} ", p);

    rng::for_each(shapes | views::take(5), [&p](const Shape &shape) {
        std::println("    - dist from {} to {}: {:.2f}", p, shape, queries::DistanceToPoint(shape, p));
    });
}

void PerformShapeAnalysis(std::span<const Shape> shapes) {
    std::println("\n=== Shape Analysis ===");

    std::println("  bounding box collisions:");
    auto colliding_pairs = views::cartesian_product(shapes, shapes) | views::filter([](const auto &pair) {
                               auto &[s1, s2] = pair;
                               return &s1 < &s2 && queries::BoundingBoxesOverlap(s1, s2);
                           });
    rng::for_each(colliding_pairs, [](const auto &pair) {
        auto &[s1, s2] = pair;
        std::println("    - {} and {}", s1, s2);
    });

    if (auto it = rng::max_element(shapes, {}, &queries::GetHeight); it != shapes.end()) {
        std::println("  highest: {} (h={:.2f})", *it, queries::GetHeight(*it));
    }

    auto supported_dist = views::cartesian_product(shapes, shapes) | views::filter([](const auto &pair) {
                              auto &[s1, s2] = pair;
                              return &s1 < &s2;
                          }) |
                          views::transform([](const auto &pair) {
                              auto &[s1, s2] = pair;
                              return std::tuple{s1, s2, queries::DistanceBetweenShapes(s1, s2)};
                          }) |
                          views::filter([](const auto &tuple) {
                              auto &dist = std::get<2>(tuple);
                              return dist.has_value();
                          }) |
                          views::take(1);

    if (!supported_dist.empty()) {
        std::println("  sample distance:");
        rng::for_each(supported_dist, [](const auto &tuple) {
            auto &[s1, s2, dist] = tuple;
            std::println("    - {} vs {}: dist={:.2f}", s1, s2, *dist);
        });
    }
}

void PerformExtraShapeAnalysis(std::span<const Shape> shapes) {
    std::println("\n=== Shape Extra Analysis ===");

    auto high_shapes = shapes |
                       views::filter([](const auto &shape) { return queries::GetBoundBox(shape).min_y > 50.0; }) |
                       views::take(3);

    if (!high_shapes.empty()) {
        std::println("  shapes above y=50.0:");
        rng::for_each(high_shapes, [](const auto &shape) { std::println("    - {}", shape); });
    }

    if (!shapes.empty()) {
        auto [min_it, max_it] = rng::minmax_element(shapes, {}, &queries::GetHeight);
        std::println("  min/max height:");
        std::println("    - min: {} (h={:.2f})", *min_it, queries::GetHeight(*min_it));
        std::println("    - max: {} (h={:.2f})", *max_it, queries::GetHeight(*max_it));
    }
}

int main() {
    utils::ShapeGenerator generator(-50.0, 50.0, 5.0, 25.0);
    std::vector<Shape> shapes = generator.GenerateShapes(15);

    std::println("Generated {} random shapes", shapes.size());

    // Выведите индекс каждой фигуры и её высоту
    rng::for_each(views::enumerate(shapes), [](const auto &indexed_shape) {
        auto [index, shape] = indexed_shape;
        std::println(" [{:2}] h={:5.2f}, {}", index, queries::GetHeight(shape), shape);
    });

    //
    // Вызываем разработанные функции
    //
    assert(!shapes.empty());
    PrintAllIntersections(shapes[0], shapes);

    PrintDistancesFromPointToShapes(Point2D{10.0, 10.0}, shapes);

    PerformShapeAnalysis(shapes);

    PerformExtraShapeAnalysis(shapes);

    //
    // Рисуем все фигуры
    //
    // Важно: после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 2ой график
    //
    geometry::visualization::Draw(shapes);

    //
    // Формируем список из вершин всех фигур
    //

    std::vector<Point2D> points;
    {

        auto all_vertices_view = shapes | views::transform([](const Shape &shape) {
                                     return std::visit(
                                         [](const auto &shape) {
                                             const auto vts = shape.Vertices();
                                             std::vector<Point2D> pts(std::begin(vts), std::end(vts));
                                             return pts;
                                         },
                                         shape);
                                 }) |
                                 views::join;
        rng::copy(all_vertices_view, std::back_inserter(points));
    }

    //
    // Находим список точек, для построения выпуклой оболочки - convex hull - алгоритмом Грэхема
    // Создаём из них объект класса `Polygon` и добавляем его в список shapes
    // Рисуем все фигуры
    //

    if (auto hull_result = geometry::convex_hull::GrahamScan(points)) {
        shapes.push_back(geometry::Polygon(*hull_result));
        geometry::visualization::Draw(shapes);
    } else {
        std::println("Error {}", static_cast<int>(hull_result.error()));
    }

    //
    // после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 3ий график
    //

    {
        std::vector<Point2D> points = {{0, 0}, {10, 0}, {5, 8}, {15, 5}, {2, 12}};

        if (auto triangulation_result = geometry::triangulation::DelaunayTriangulation(points)) {
            geometry::visualization::Draw(triangulation_result.value());
        } else {
            std::println("Error {}", static_cast<int>(triangulation_result.error()));
        }
    }

    return 0;
}