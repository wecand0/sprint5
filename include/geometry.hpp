#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <expected>
#include <format>
#include <numbers>
#include <print>
#include <ranges>
#include <stdexcept>
#include <variant>
#include <vector>

namespace geometry {

inline bool are_equals(double a, double b, double epsilon = 1e-9) { return std::abs(a - b) <= epsilon; }

inline bool is_equal_zero(double a, double epsilon = 1e-9) { return std::abs(a) <= epsilon; }

/*
 * Добавьте к методам класса Point2D и Lines2DDyn все необходимые аттрибуты и спецификаторы
 * Важно: Возвращаемый тип и принимаемые аргументы менять не нужно
 */
struct Point2D {
    double x, y;

    constexpr Point2D() : x(0), y(0) {}
    constexpr Point2D(double x, double y) : x(x), y(y) {}

    // Comparison
    bool operator<(const Point2D &other) const noexcept { return std::tie(x, y) < std::tie(other.x, other.y); }
    bool operator==(const Point2D &other) const noexcept { return are_equals(x, other.x) && are_equals(y, other.y); }

    // Binary math operators
    Point2D operator+(const Point2D &other) const noexcept { return {x + other.x, y + other.y}; }
    Point2D operator-(const Point2D &other) const noexcept { return {x - other.x, y - other.y}; }
    Point2D operator*(double value) const noexcept { return {x * value, y * value}; }
    Point2D operator/(double value) const noexcept { return {x / value, y / value}; }

    // Binary geometry operations
    double Dot(const Point2D &other) const noexcept { return x * other.x + y * other.y; }
    double Cross(const Point2D &other) const noexcept { return x * other.y - y * other.x; }
    double Length() const noexcept { return std::sqrt(x * x + y * y); }
    double DistanceTo(const Point2D &other) const noexcept { return (*this - other).Length(); }

    Point2D Normalize() {
        const double len = Length();
        return len > 0 ? Point2D{x / len, y / len} : Point2D{0, 0};
    }
};

template <std::ranges::random_access_range R>
    requires std::is_same_v<std::ranges::range_value_t<R>, Point2D>
void sort_points_clockwise(R &&r) {
    Point2D mid = std::ranges::fold_left(r, Point2D{0, 0}, std::plus<Point2D>{}) / r.size();
    std::ranges::sort(r, [&mid](const Point2D &lhs, const Point2D &rhs) {
        return std::atan2(lhs.y - mid.y, lhs.x - mid.x) < std::atan2(rhs.y - mid.y, rhs.x - mid.x);
    });
}

template <size_t N>
struct Lines2D {
    std::array<double, N> x;
    std::array<double, N> y;
};

struct Lines2DDyn {
    std::vector<double> x;
    std::vector<double> y;

    void Reserve(size_t n) {
        x.reserve(n);
        y.reserve(n);
    }
    void PushBack(Point2D p) {
        x.push_back(p.x);
        y.push_back(p.y);
    }
    void PushBack(double px, double py) {
        x.push_back(px);
        y.push_back(py);
    }
    Point2D Front() { return {x.front(), y.front()}; }
};

struct BoundingBox {
    double min_x, min_y, max_x, max_y;

    bool Overlaps(const BoundingBox &other) const noexcept {
        bool no_overlap_along_x{max_x < other.min_x || other.max_x < min_x};
        bool no_overlap_along_y{max_y < other.min_y || other.max_y < min_y};
        return !no_overlap_along_x && !no_overlap_along_y;
    }

    double Width() const noexcept { return std::abs(max_x - min_x); }
    double Height() const noexcept { return std::abs(max_y - min_y); }

    Point2D Center() const noexcept { return Point2D{min_x + Width() / 2.0, min_y + Height() / 2.0}; }

    bool operator==(const BoundingBox &other) const noexcept {
        return are_equals(min_x, other.min_x) && are_equals(min_y, other.min_y) && are_equals(max_x, other.max_x) &&
               are_equals(max_y, other.max_y);
    }
};

struct Line {
    Point2D start, end;

    double Length() const noexcept { return start.DistanceTo(end); }
    Point2D Direction() const noexcept { return end - start; }
    BoundingBox BoundBox() const noexcept {
        auto [min_x, max_x] = std::minmax(start.x, end.x);
        auto [min_y, max_y] = std::minmax(start.y, end.y);

        return BoundingBox{.min_x = min_x, .min_y = min_y, .max_x = max_x, .max_y = max_y};
    }

    double Height() const noexcept { return BoundBox().Height(); }
    Point2D Center() const noexcept { return (start + end) / 2.0; }

    std::array<Point2D, 2> Vertices() const noexcept {
        std::array<Point2D, 2> res{start, end};
        sort_points_clockwise(res);
        return res;
    }

    Lines2D<2> Lines() const noexcept { return {{start.x, end.x}, {start.y, end.y}}; }

    bool operator==(const Line &other) const noexcept {
        return std::tie(start, end) == std::tie(other.start, other.end);
    };
};

struct Triangle {
    Point2D a, b, c;

    double Area() const noexcept {
        const Point2D diff_ba = b - a;
        const Point2D diff_ca = c - a;
        return 0.5 * std::abs(diff_ba.Cross(diff_ca));
    }

    double Height() const noexcept { return BoundBox().Height(); }
    Point2D Center() const noexcept { return (a + b + c) / 3.0; }

    BoundingBox BoundBox() const noexcept {
        auto [min_x, max_x] = std::minmax({a.x, b.x, c.x});
        auto [min_y, max_y] = std::minmax({a.y, b.y, c.y});

        return BoundingBox{.min_x = min_x, .min_y = min_y, .max_x = max_x, .max_y = max_y};
    }

    std::array<Point2D, 3> Vertices() const noexcept {
        std::array<Point2D, 3> res{a, b, c};
        sort_points_clockwise(res);
        return res;
    }

    Lines2D<4> Lines() const noexcept { return {{a.x, b.x, c.x, a.x}, {a.y, b.y, c.y, a.y}}; }

    std::array<Line, 3> Edges() const noexcept {
        const auto pts = Vertices();
        return {{{pts[0], pts[1]}, {pts[1], pts[2]}, {pts[2], pts[0]}}};
    }

    bool operator==(const Triangle &other) const noexcept {
        return std::tie(a, b, c) == std::tie(other.a, other.b, other.c);
    };
};

struct Rectangle {
    Point2D bottom_left;
    double width, height;

    double Area() const noexcept { return std::abs(width * height); }
    double Height() const noexcept { return std::abs(height); }
    Point2D Center() const noexcept { return {bottom_left.x + width / 2, bottom_left.y + height / 2}; }

    BoundingBox BoundBox() const noexcept {
        Point2D top_right{bottom_left.x + width, bottom_left.y + height};
        const auto [min_x, max_x] = std::minmax(bottom_left.x, top_right.x);
        const auto [min_y, max_y] = std::minmax(bottom_left.y, top_right.y);

        return BoundingBox{.min_x = min_x, .min_y = min_y, .max_x = max_x, .max_y = max_y};
    }

    std::array<Point2D, 4> Vertices() const noexcept {
        // clang-format off
        std::array<Point2D, 4> res{
            Point2D{bottom_left.x,         bottom_left.y},
            Point2D{bottom_left.x + width, bottom_left.y},
            Point2D{bottom_left.x,         bottom_left.y + height},
            Point2D{bottom_left.x + width, bottom_left.y + height},
        };
        // clang-format on
        sort_points_clockwise(res);
        return res;
    }

    Lines2D<5> Lines() const noexcept {
        const auto pts = Vertices();
        return {{pts[0].x, pts[1].x, pts[2].x, pts[3].x, pts[0].x}, {pts[0].y, pts[1].y, pts[2].y, pts[3].y, pts[0].y}};
    }

    std::array<Line, 4> Edges() const noexcept {
        const auto pts = Vertices();
        return {{{pts[0], pts[1]}, {pts[1], pts[2]}, {pts[2], pts[3]}, {pts[3], pts[0]}}};
    }

    bool operator==(const Rectangle &other) const noexcept {
        return std::tie(bottom_left, width, height) == std::tie(other.bottom_left, other.width, other.height);
    };
};

struct RegularPolygon {
    Point2D center_p;
    double radius;
    int sides;

    constexpr RegularPolygon(Point2D center, double radius, int sides)
        : center_p(center), radius(radius), sides(sides) {
        if (sides < 0) {
            throw std::invalid_argument{"sides must be positive"};
        }
    }

    double Height() const noexcept { return BoundBox().Height(); }
    Point2D Center() const noexcept { return center_p; }

    BoundingBox BoundBox() const noexcept {
        const auto pts = Vertices();
        if (pts.empty()) {
            return {0.0, 0.0, 0.0, 0.0};
        }

        const auto [min_x, max_x] = std::ranges::minmax(pts, {}, &Point2D::x);
        const auto [min_y, max_y] = std::ranges::minmax(pts, {}, &Point2D::y);

        return {min_x.x, min_y.y, max_x.x, max_y.y};
    }

    std::vector<Point2D> Vertices() const {
        std::vector<Point2D> points;
        points.reserve(sides);

        for (int i = 0; i < sides; ++i) {
            const double angle = 2 * std::numbers::pi * i / sides;
            points.emplace_back(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        return points;
    }

    Lines2DDyn Lines() const {
        if (sides == 0) {
            return {};
        }

        Lines2DDyn res;
        const auto pts = Vertices();
        res.Reserve(pts.size() + 1);
        for (const auto &pt : pts) {
            res.PushBack(pt);
        }
        res.PushBack(pts.front());
        return res;
    }

    bool operator==(const RegularPolygon &other) const noexcept {
        return std::tie(center_p, radius, sides) == std::tie(other.center_p, other.radius, other.sides);
    };
};

struct Circle {
    Point2D center_p;
    double radius;

    constexpr Circle(Point2D center, double radius) : center_p(center), radius(radius) {}

    double Height() const noexcept { return 2 * std::abs(radius); }
    Point2D Center() const noexcept { return center_p; }

    BoundingBox BoundBox() const noexcept {
        const double r = std::abs(radius);
        return {center_p.x - r, center_p.y - r, center_p.x + r, center_p.y + r};
    }

    std::vector<Point2D> Vertices(size_t N = 30) const {
        if (N == 0) {
            return {};
        }

        std::vector<Point2D> points;
        points.reserve(N);

        const double r = std::abs(radius);
        for (size_t i = 0; i != N; ++i) {
            const double angle = 2.0 * std::numbers::pi * i / N;
            points.emplace_back(center_p.x + r * std::cos(angle), center_p.y + r * std::sin(angle));
        }
        return points;
    }

    Lines2DDyn Lines(size_t N = 100) const {
        const auto pts = Vertices(N);
        if (N == 0 || pts.empty()) {
            return {};
        }

        Lines2DDyn res;
        res.Reserve(pts.size() + 1);
        for (const auto &pt : pts) {
            res.PushBack(pt);
        }
        res.PushBack(pts.front());
        return res;
    }

    bool operator==(const Circle &other) const noexcept {
        return std::tie(center_p, radius) == std::tie(other.center_p, other.radius);
    };
};

class Polygon {
public:
    explicit Polygon(std::vector<Point2D> points) : points_{std::move(points)}, bounding_box_{0.0, 0.0, 0.0, 0.0} {
        if (!points_.empty()) {
            const auto [min_x, max_x] = std::ranges::minmax(points_, {}, &Point2D::x);
            const auto [min_y, max_y] = std::ranges::minmax(points_, {}, &Point2D::y);

            bounding_box_ = {min_x.x, min_y.y, max_x.x, max_y.y};
        }
    }
    double Height() const noexcept { return bounding_box_.Height(); }
    Point2D Center() const noexcept { return bounding_box_.Center(); }
    BoundingBox BoundBox() const noexcept { return bounding_box_; }
    std::vector<Point2D> Vertices(size_t N = 30) const noexcept {
        size_t size = std::min(N, points_.size());

        std::vector<Point2D> res;
        res.reserve(size);
        std::copy_n(points_.begin(), size, std::back_inserter(res));
        return res;
    }
    Lines2DDyn Lines(size_t N = 100) const {
        size_t size = std::min(N, points_.size());

        Lines2DDyn res;
        res.Reserve(size + 1);
        for (size_t i = 0; i != size; ++i) {
            res.PushBack(points_[i]);
        }
        res.PushBack(points_.front());
        return res;
    }

    std::vector<Line> Edges() const {
        if (points_.size() < 2) {
            return {};
        }

        return std::views::iota(0u, points_.size()) |
               std::views::transform([this](size_t i) { return Line{points_[i], points_[(i + 1) % points_.size()]}; }) |
               std::ranges::to<std::vector>();
    }

    bool operator==(const Polygon &other) const noexcept {
        return std::tie(points_, bounding_box_) == std::tie(other.points_, other.bounding_box_);
    };

private:
    std::vector<Point2D> points_;
    BoundingBox bounding_box_;
};

using Shape = std::variant<Line, Triangle, Rectangle, RegularPolygon, Circle, Polygon>;

enum class GeometryError { Unsupported, NoIntersection, InvalidInput, DegenrateCase, InsufficientPoints };

template <typename T>
using GeometryResult = std::expected<T, GeometryError>;

}  // namespace geometry

template <>
struct std::formatter<geometry::Point2D> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Point2D &p, FormatContext &ctx) const {
        return format_to(ctx.out(), "({:.2f}, {:.2f})", p.x, p.y);
    }
};
template <>
struct std::formatter<std::vector<geometry::Point2D>> {
    bool use_new_line = false;

    constexpr auto parse(std::format_parse_context &ctx) {
        auto it = ctx.begin();

        auto end = ctx.begin();
        while (end != ctx.end() && *end != '}') {
            ++end;
        }

        std::string_view specifier(it, end);
        if (specifier.empty()) {
            use_new_line = false;
        } else if (specifier == "new_line"sv) {
            use_new_line = true;
        } else {
            throw std::format_error("invalid format specifier");
        }

        return it;
    }

    template <typename FormatContext>
    auto format(const std::vector<geometry::Point2D> &v, FormatContext &ctx) const {
        const auto sep = use_new_line ? "\n\t"sv : " "sv;
        for (const auto &p : v) {
            std::format_to(ctx.out(), "{}{}", sep, p);
        }
        return ctx.out();
    }
};

template <>
struct std::formatter<geometry::Line> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Line &l, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Line({}, {})", l.start, l.end);
    }
};

template <>
struct std::formatter<geometry::Circle> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Circle &c, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Circle(center={}, r={:.2f})", c.center_p, c.radius);
    }
};

template <>
struct std::formatter<geometry::Rectangle> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Rectangle &r, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Rectangle(bottom_left={}, w={:.2f}, h={:.2f})", r.bottom_left, r.width,
                              r.height);
    }
};

template <>
struct std::formatter<geometry::RegularPolygon> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::RegularPolygon &p, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "RegularPolygon(center={}, r={:.2f}, sides={})", p.center_p, p.radius,
                              p.sides);
    }
};
template <>
struct std::formatter<geometry::Triangle> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Triangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Triangle({}, {}, {})", t.a, t.b, t.c);
    }
};
template <>
struct std::formatter<geometry::Polygon> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Polygon &poly, FormatContext &ctx) const {
        auto out = ctx.out();
        out = std::format_to(out, "Polygon[{} points]: [", poly.Vertices().size());

        for (const auto &p : poly.Vertices()) {
            out = std::format_to(out, "{} ", p);
        }

        return std::format_to(out, "]");
    }
};

template <>
struct std::formatter<geometry::Shape> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Shape &shape, FormatContext &ctx) const {
        return std::visit([&ctx](const auto &s) { return std::format_to(ctx.out(), "{}", s); }, shape);
    }
};