#include "convex_hull.hpp"
#include "geometry.hpp"
#include <algorithm>
#include <cassert>
#include <iterator>

namespace geometry::convex_hull {

double CrossProduct(Point2D p1, Point2D middle, Point2D p2) {
    auto new_p1 = p1 - middle;
    auto new_p2 = p2 - middle;
    return new_p1.Cross(new_p2);
}

GeometryResult<std::vector<Point2D>> GrahamScan(std::span<const Point2D> points) {
    if (points.size() < 3) {
        return std::unexpected{GeometryError::InsufficientPoints};
    }

    std::vector<Point2D> copy_points(points.begin(), points.end());

    // поиск опорной точки
    Point2D p0{};
    {
        auto it = std::ranges::min_element(copy_points, {}, [](const Point2D &p) { return std::tie(p.y, p.x); });
        assert(it != copy_points.end());
        std::iter_swap(copy_points.begin(), it);
        p0 = copy_points[0];
    }

    // сортировка по полярному углу
    {
        auto range_for_sort = std::ranges::subrange(std::next(copy_points.begin(), 1), std::end(copy_points));
        std::ranges::sort(range_for_sort, [&p0](const Point2D &lhs, const Point2D &rhs) {
            auto cross_prod = CrossProduct(lhs, p0, rhs);
            return is_equal_zero(cross_prod) ? (p0.DistanceTo(lhs) < p0.DistanceTo(rhs)) : (cross_prod > 0.);
        });
    }

    StackForGrahamScan hull{copy_points.size()};

    for (auto p : copy_points) {
        // удаляется последняя точка со стека, пока она образует невыпуклость
        while (hull.Size() >= 2) {
            auto cross_prod = CrossProduct(p, hull.Top(), hull.NextToTop());
            if (cross_prod <= 0.) {
                hull.Pop();
            } else {
                break;
            }
        }

        hull.Push(p);
    }

    return std::move(hull).Extract();
}

}  // namespace geometry::convex_hull