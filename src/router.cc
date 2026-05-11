#include "router.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace router {
namespace {

static bool g_debug_enabled = false;
static bool g_debug_file_enabled = false;

static constexpr double EPS = 1e-9;
static constexpr double INF = 1e100;

enum class Dir {
    None = 0,
    Up,
    Down,
    Left,
    Right
};

enum class Shape {
    I,
    L,
    Z,
    Maze,
    Failed
};

enum class Policy {
    GlobalPatternThenMaze,
    ExternalAccessPatternThenMaze,
    LocalClusterPatternOnly,
    Unknown
};

struct ScaledPoint {
    long long x = 0;
    long long y = 0;

    bool operator==(const ScaledPoint& other) const {
        return x == other.x && y == other.y;
    }
};

struct ScaledPointHash {
    std::size_t operator()(const ScaledPoint& p) const {
        const std::uint64_t ux = static_cast<std::uint64_t>(p.x);
        const std::uint64_t uy = static_cast<std::uint64_t>(p.y);
        return static_cast<std::size_t>((ux << 32U) ^ uy);
    }
};

struct ScaledState {
    ScaledPoint point;
    Dir prev_dir = Dir::None;

    bool operator==(const ScaledState& other) const {
        return prev_dir == other.prev_dir && point == other.point;
    }
};

struct ScaledStateHash {
    std::size_t operator()(const ScaledState& s) const {
        const std::size_t p = ScaledPointHash{}(s.point);
        return (p * 1315423911ULL) ^ static_cast<std::size_t>(s.prev_dir);
    }
};

struct ScaledBBox {
    long long lx = 0;
    long long ly = 0;
    long long ux = 0;
    long long uy = 0;
    bool valid = false;
};

struct NodePorts {
    std::array<bool, 4> used{{false, false, false, false}};
};

struct EdgeInfo {
    int topo_edge_index = -1;
    int parent = -1;
    int child = -1;
    bool parent_is_source = false;
    common::NodeKind parent_kind = common::NodeKind::Global;
    common::NodeKind child_kind = common::NodeKind::Global;
    ScaledPoint start;
    ScaledPoint goal;
    std::string parent_class;
    std::string child_class;
    Policy policy = Policy::Unknown;
    int policy_group = 0;
    int cluster_top = -1;
    int cluster_depth = 0;
    int source_depth = 0;
    common::BBox cluster_bbox;
    bool has_cluster_bbox = false;
    long long manhattan_distance = 0;
};

struct Candidate {
    std::vector<ScaledPoint> points;
    Shape shape = Shape::Failed;
    double score = INF;
    double wirelength = 0.0;
    int bends = 0;
    Dir parent_exit_dir = Dir::None;
    Dir child_entry_dir = Dir::None;
    bool parent_port_available = false;
    bool child_port_available = false;
    bool used_preferred_parent = false;
    bool used_preferred_child = false;
    double parent_exit_factor = 1.0;
    double child_entry_factor = 1.0;
    int expanded_nodes = 0;
    bool maze_used = false;
    std::string maze_failed_reason;
};

struct PointKey {
    long long x = 0;
    long long y = 0;

    bool operator==(const PointKey& other) const {
        return x == other.x && y == other.y;
    }
};

struct PointKeyHash {
    std::size_t operator()(const PointKey& p) const {
        const std::uint64_t ux = static_cast<std::uint64_t>(p.x);
        const std::uint64_t uy = static_cast<std::uint64_t>(p.y);
        return static_cast<std::size_t>((ux << 32U) ^ uy);
    }
};

struct RouteSegment {
    ScaledPoint a;
    ScaledPoint b;
    Policy policy = Policy::Unknown;
    int cluster_top = -1;
};

struct AStarNode {
    ScaledState state;
    long long g = 0;
    long long f = 0;
};

struct AStarCompare {
    bool operator()(const AStarNode& a, const AStarNode& b) const {
        if (a.f != b.f) {
            return a.f > b.f;
        }
        return a.g > b.g;
    }
};

static PointKey to_key(const ScaledPoint& p) {
    return PointKey{p.x, p.y};
}

static bool nearly_scaled(double v, int scale) {
    return std::abs(v * scale - std::round(v * scale)) <= 10.0 * EPS;
}

static int choose_scale(const common::Problem& problem,
                        const common::TopoTree& tree,
                        const common::LocerResult& loc_result) {
    const std::array<int, 4> scales{{1, 2, 4, 8}};
    auto fits = [&](int scale) {
        if (!nearly_scaled(problem.source.loc.x, scale) ||
            !nearly_scaled(problem.source.loc.y, scale)) {
            return false;
        }
        for (const common::Sink& sink : problem.sinks) {
            if (!nearly_scaled(sink.loc.x, scale) ||
                !nearly_scaled(sink.loc.y, scale)) {
                return false;
            }
        }
        if (loc_result.node_results.size() != tree.nodes.size()) {
            return false;
        }
        for (std::size_t i = 0; i < loc_result.node_results.size(); ++i) {
            const common::LocerNodeResult& node = loc_result.node_results[i];
            if (!node.valid) {
                return false;
            }
            if (!nearly_scaled(node.loc.x, scale) ||
                !nearly_scaled(node.loc.y, scale)) {
                return false;
            }
        }
        return true;
    };

    for (int scale : scales) {
        if (fits(scale)) {
            return scale;
        }
    }
    return 8;
}

static ScaledPoint scale_point(const common::SegmentPoint& p, int scale) {
    return ScaledPoint{
        static_cast<long long>(std::llround(p.x * scale)),
        static_cast<long long>(std::llround(p.y * scale)),
    };
}

static ScaledPoint scale_point(const common::Point& p, int scale) {
    return ScaledPoint{
        static_cast<long long>(p.x) * scale,
        static_cast<long long>(p.y) * scale,
    };
}

static common::SegmentPoint unscale_point(const ScaledPoint& p, int scale) {
    return common::SegmentPoint{
        static_cast<double>(p.x) / scale,
        static_cast<double>(p.y) / scale,
    };
}

static long long manhattan_scaled(const ScaledPoint& a, const ScaledPoint& b) {
    return std::llabs(a.x - b.x) + std::llabs(a.y - b.y);
}

static bool in_die(const ScaledPoint& p, int die_width, int die_height, int scale) {
    return p.x >= 0 && p.x <= static_cast<long long>(die_width) * scale &&
           p.y >= 0 && p.y <= static_cast<long long>(die_height) * scale;
}

static bool same_point(const ScaledPoint& a, const ScaledPoint& b) {
    return a.x == b.x && a.y == b.y;
}

static std::string node_class_to_string(common::NodeKind kind, bool is_source = false) {
    if (is_source) {
        return "SOURCE";
    }
    switch (kind) {
        case common::NodeKind::Sink:
            return "SINK";
        case common::NodeKind::ClusterInternal:
            return "CLUSTER_INTERNAL";
        case common::NodeKind::ClusterAccess:
            return "CLUSTER_ACCESS";
        case common::NodeKind::ClusterBridge:
            return "CLUSTER_BRIDGE";
        case common::NodeKind::ClusterTop:
            return "CLUSTER_TOP";
        case common::NodeKind::Global:
            return "GLOBAL";
    }
    return "UNKNOWN";
}

static std::string policy_to_string(Policy policy) {
    switch (policy) {
        case Policy::GlobalPatternThenMaze:
            return "GlobalPatternThenMaze";
        case Policy::ExternalAccessPatternThenMaze:
            return "ExternalAccessPatternThenMaze";
        case Policy::LocalClusterPatternOnly:
            return "LocalClusterPatternOnly";
        case Policy::Unknown:
            return "Unknown";
    }
    return "Unknown";
}

static std::string shape_to_string(Shape shape) {
    switch (shape) {
        case Shape::I:
            return "I";
        case Shape::L:
            return "L";
        case Shape::Z:
            return "Z";
        case Shape::Maze:
            return "MAZE";
        case Shape::Failed:
            return "FAILED";
    }
    return "FAILED";
}

static std::string dir_to_string(Dir dir) {
    switch (dir) {
        case Dir::Up:
            return "UP";
        case Dir::Down:
            return "DOWN";
        case Dir::Left:
            return "LEFT";
        case Dir::Right:
            return "RIGHT";
        case Dir::None:
            return "NONE";
    }
    return "NONE";
}

static int dir_index(Dir dir) {
    switch (dir) {
        case Dir::Up:
            return 0;
        case Dir::Down:
            return 1;
        case Dir::Left:
            return 2;
        case Dir::Right:
            return 3;
        case Dir::None:
            return -1;
    }
    return -1;
}

static Dir opposite(Dir dir) {
    switch (dir) {
        case Dir::Up:
            return Dir::Down;
        case Dir::Down:
            return Dir::Up;
        case Dir::Left:
            return Dir::Right;
        case Dir::Right:
            return Dir::Left;
        case Dir::None:
            return Dir::None;
    }
    return Dir::None;
}

static ScaledPoint dir_vector(Dir dir) {
    switch (dir) {
        case Dir::Up:
            return ScaledPoint{0, 1};
        case Dir::Down:
            return ScaledPoint{0, -1};
        case Dir::Left:
            return ScaledPoint{-1, 0};
        case Dir::Right:
            return ScaledPoint{1, 0};
        case Dir::None:
            return ScaledPoint{0, 0};
    }
    return ScaledPoint{0, 0};
}

static std::array<Dir, 4> all_dirs() {
    return {Dir::Up, Dir::Down, Dir::Left, Dir::Right};
}

static Dir step_dir(const ScaledPoint& a, const ScaledPoint& b);

static bool candidate_matches_port_pair(const std::vector<ScaledPoint>& points,
                                        Dir parent_exit_dir,
                                        Dir child_entry_dir) {
    if (points.size() < 2U) {
        return false;
    }
    const Dir first = step_dir(points.front(), points[1U]);
    const Dir last = step_dir(points[points.size() - 2U], points.back());
    return first == parent_exit_dir && opposite(last) == child_entry_dir;
}

static Dir step_dir(const ScaledPoint& a, const ScaledPoint& b) {
    if (a.x == b.x) {
        if (a.y < b.y) return Dir::Up;
        if (a.y > b.y) return Dir::Down;
    } else if (a.y == b.y) {
        if (a.x < b.x) return Dir::Right;
        if (a.x > b.x) return Dir::Left;
    }
    return Dir::None;
}

static bool is_manhattan_segment(const ScaledPoint& a, const ScaledPoint& b) {
    return a.x == b.x || a.y == b.y;
}

static Dir dominant_dir_between(const ScaledPoint& from,
                                const ScaledPoint& to);

static double preferred_dir_factor(const EdgeInfo& edge, Dir dir, bool is_child_entry) {
    if (dir == Dir::None) {
        return 1.0;
    }
    Dir dom = dominant_dir_between(edge.start, edge.goal);
    if (dom == Dir::None) {
        return 0.0;
    }
    if (is_child_entry) {
        dom = opposite(dom);
    }
    if (dir == dom) {
        return 0.0;
    }
    if (dir == opposite(dom)) {
        return 1.0;
    }
    return 0.5;
}

static Dir dominant_dir_between(const ScaledPoint& from,
                                const ScaledPoint& to) {
    const long long dx = to.x - from.x;
    const long long dy = to.y - from.y;
    if (std::llabs(dx) >= std::llabs(dy)) {
        if (dx > 0) return Dir::Right;
        if (dx < 0) return Dir::Left;
    } else {
        if (dy > 0) return Dir::Up;
        if (dy < 0) return Dir::Down;
    }
    return Dir::None;
}

static ScaledBBox scaled_bbox(const common::BBox& bbox, int scale) {
    return ScaledBBox{
        static_cast<long long>(bbox.lx) * scale,
        static_cast<long long>(bbox.ly) * scale,
        static_cast<long long>(bbox.ux) * scale,
        static_cast<long long>(bbox.uy) * scale,
        true,
    };
}

static int find_cluster_top_ancestor(const std::vector<common::TopoNode>& nodes,
                                    int node_id) {
    int cur = node_id;
    while (cur >= 0 && static_cast<std::size_t>(cur) < nodes.size()) {
        if (nodes[static_cast<std::size_t>(cur)].kind == common::NodeKind::ClusterTop) {
            return cur;
        }
        cur = nodes[static_cast<std::size_t>(cur)].parent;
    }
    return -1;
}

static int node_depth_to_source(const std::vector<common::TopoNode>& nodes,
                                int node_id,
                                std::vector<int>& cache) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= nodes.size()) {
        return 0;
    }
    int& memo = cache[static_cast<std::size_t>(node_id)];
    if (memo > 0) {
        return memo;
    }
    const int parent = nodes[static_cast<std::size_t>(node_id)].parent;
    if (parent < 0) {
        memo = 1;
        return memo;
    }
    memo = node_depth_to_source(nodes, parent, cache) + 1;
    return memo;
}

static int node_depth_to_cluster_top(const std::vector<common::TopoNode>& nodes,
                                     int node_id,
                                     int cluster_top) {
    if (node_id < 0 || cluster_top < 0) {
        return 0;
    }
    int depth = 0;
    int cur = node_id;
    while (cur >= 0 && static_cast<std::size_t>(cur) < nodes.size()) {
        if (cur == cluster_top) {
            return depth;
        }
        cur = nodes[static_cast<std::size_t>(cur)].parent;
        ++depth;
    }
    return 0;
}

static bool point_strictly_inside_bbox(const ScaledPoint& p,
                                      const ScaledBBox& bbox) {
    return bbox.valid &&
           p.x > bbox.lx && p.x < bbox.ux &&
           p.y > bbox.ly && p.y < bbox.uy;
}

static ScaledBBox expand_bbox_to_point(ScaledBBox bbox,
                                       const ScaledPoint& p,
                                       long long margin) {
    if (!bbox.valid) {
        bbox.lx = bbox.ux = p.x;
        bbox.ly = bbox.uy = p.y;
        bbox.valid = true;
    }
    bbox.lx = std::min(bbox.lx, p.x - margin);
    bbox.ly = std::min(bbox.ly, p.y - margin);
    bbox.ux = std::max(bbox.ux, p.x + margin);
    bbox.uy = std::max(bbox.uy, p.y + margin);
    return bbox;
}

static ScaledBBox clamp_bbox_to_die(ScaledBBox bbox,
                                    int die_width,
                                    int die_height,
                                    int scale) {
    if (!bbox.valid) {
        return bbox;
    }
    bbox.lx = std::max(0LL, bbox.lx);
    bbox.ly = std::max(0LL, bbox.ly);
    bbox.ux = std::min(static_cast<long long>(die_width) * scale, bbox.ux);
    bbox.uy = std::min(static_cast<long long>(die_height) * scale, bbox.uy);
    return bbox;
}

static long long segment_length_scaled(const ScaledPoint& a, const ScaledPoint& b) {
    return manhattan_scaled(a, b);
}

static void canonicalize_polyline(std::vector<ScaledPoint>& points) {
    std::vector<ScaledPoint> cleaned;
    cleaned.reserve(points.size());
    for (const ScaledPoint& p : points) {
        if (!cleaned.empty() && same_point(cleaned.back(), p)) {
            continue;
        }
        cleaned.push_back(p);
    }
    bool changed = true;
    while (changed && cleaned.size() >= 3U) {
        changed = false;
        std::vector<ScaledPoint> next;
        next.reserve(cleaned.size());
        next.push_back(cleaned.front());
        for (std::size_t i = 1; i + 1 < cleaned.size(); ++i) {
            const ScaledPoint& a = next.back();
            const ScaledPoint& b = cleaned[i];
            const ScaledPoint& c = cleaned[i + 1U];
            if ((a.x == b.x && b.x == c.x) || (a.y == b.y && b.y == c.y)) {
                changed = true;
                continue;
            }
            next.push_back(b);
        }
        next.push_back(cleaned.back());
        cleaned.swap(next);
    }
    points.swap(cleaned);
}

static int bends_of_polyline(const std::vector<ScaledPoint>& points) {
    if (points.size() < 3U) {
        return 0;
    }
    int bends = 0;
    for (std::size_t i = 2; i < points.size(); ++i) {
        const Dir d1 = step_dir(points[i - 2U], points[i - 1U]);
        const Dir d2 = step_dir(points[i - 1U], points[i]);
        if (d1 != Dir::None && d2 != Dir::None && d1 != d2) {
            ++bends;
        }
    }
    return bends;
}

static double wirelength_of_polyline(const std::vector<ScaledPoint>& points, int scale) {
    long long total = 0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        total += segment_length_scaled(points[i - 1U], points[i]);
    }
    return static_cast<double>(total) / scale;
}

static std::string polyline_key(const std::vector<ScaledPoint>& points) {
    std::ostringstream oss;
    for (const ScaledPoint& p : points) {
        oss << p.x << "," << p.y << ";";
    }
    return oss.str();
}

static bool port_free(const NodePorts& ports, Dir dir) {
    const int idx = dir_index(dir);
    return idx >= 0 && !ports.used[static_cast<std::size_t>(idx)];
}

static void occupy_port(NodePorts& ports, Dir dir) {
    const int idx = dir_index(dir);
    if (idx >= 0) {
        ports.used[static_cast<std::size_t>(idx)] = true;
    }
}

static std::vector<Dir> available_dirs(const NodePorts& ports) {
    std::vector<Dir> dirs;
    for (Dir dir : all_dirs()) {
        if (port_free(ports, dir)) {
            dirs.push_back(dir);
        }
    }
    return dirs;
}

static bool point_forbidden_for_edge(
    const ScaledPoint& p,
    const EdgeInfo& edge,
    const std::vector<ScaledPoint>& node_points,
    const ScaledPoint& source_point,
    const std::unordered_set<PointKey, PointKeyHash>& committed_points) {
    if (!same_point(p, edge.start) && !same_point(p, edge.goal)) {
        if (committed_points.find(to_key(p)) != committed_points.end()) {
            return true;
        }
    }

    if (!same_point(p, edge.start) && !same_point(p, edge.goal)) {
        if (same_point(p, source_point) && !edge.parent_is_source && !same_point(source_point, edge.start) &&
            !same_point(source_point, edge.goal)) {
            return true;
        }
        for (std::size_t i = 0; i < node_points.size(); ++i) {
            if ((static_cast<int>(i) == edge.parent && same_point(p, edge.start)) ||
                (static_cast<int>(i) == edge.child && same_point(p, edge.goal))) {
                continue;
            }
            if (same_point(p, node_points[i])) {
                return true;
            }
        }
    }
    return false;
}

static bool point_inside_forbidden_bbox(const ScaledPoint& p,
                                        const EdgeInfo& edge,
                                        const std::vector<ScaledBBox>& node_bboxes,
                                        const std::vector<common::TopoNode>& topo_nodes) {
    if (same_point(p, edge.start) || same_point(p, edge.goal)) {
        return false;
    }
    for (std::size_t i = 0; i < node_bboxes.size(); ++i) {
        if (static_cast<int>(i) == edge.parent || static_cast<int>(i) == edge.child) {
            continue;
        }
        if (static_cast<int>(i) == edge.cluster_top) {
            continue;
        }
        if (i >= topo_nodes.size() ||
            topo_nodes[i].kind != common::NodeKind::ClusterTop) {
            continue;
        }
        if (point_strictly_inside_bbox(p, node_bboxes[i])) {
            return true;
        }
    }
    return false;
}

static bool point_on_segment(const ScaledPoint& p,
                             const RouteSegment& s) {
    if (!is_manhattan_segment(s.a, s.b)) {
        return false;
    }
    if (s.a.x == s.b.x) {
        return p.x == s.a.x &&
               p.y >= std::min(s.a.y, s.b.y) &&
               p.y <= std::max(s.a.y, s.b.y);
    }
    return p.y == s.a.y &&
           p.x >= std::min(s.a.x, s.b.x) &&
           p.x <= std::max(s.a.x, s.b.x);
}

static bool segment_endpoint(const ScaledPoint& p,
                             const RouteSegment& s) {
    return same_point(p, s.a) || same_point(p, s.b);
}

static bool segment_intersection_point(const RouteSegment& a,
                                       const RouteSegment& b,
                                       ScaledPoint& out) {
    const bool a_h = a.a.y == a.b.y;
    const bool a_v = a.a.x == a.b.x;
    const bool b_h = b.a.y == b.b.y;
    const bool b_v = b.a.x == b.b.x;
    if (!((a_h || a_v) && (b_h || b_v))) {
        return false;
    }
    if (a_h && b_v) {
        const ScaledPoint p{b.a.x, a.a.y};
        if (point_on_segment(p, a) && point_on_segment(p, b)) {
            out = p;
            return true;
        }
        return false;
    }
    if (a_v && b_h) {
        const ScaledPoint p{a.a.x, b.a.y};
        if (point_on_segment(p, a) && point_on_segment(p, b)) {
            out = p;
            return true;
        }
        return false;
    }

    if (a_h && b_h && a.a.y == b.a.y) {
        const long long lo = std::max(std::min(a.a.x, a.b.x),
                                      std::min(b.a.x, b.b.x));
        const long long hi = std::min(std::max(a.a.x, a.b.x),
                                      std::max(b.a.x, b.b.x));
        if (lo > hi) {
            return false;
        }
        out = ScaledPoint{lo, a.a.y};
        return true;
    }
    if (a_v && b_v && a.a.x == b.a.x) {
        const long long lo = std::max(std::min(a.a.y, a.b.y),
                                      std::min(b.a.y, b.b.y));
        const long long hi = std::min(std::max(a.a.y, a.b.y),
                                      std::max(b.a.y, b.b.y));
        if (lo > hi) {
            return false;
        }
        out = ScaledPoint{a.a.x, lo};
        return true;
    }
    return false;
}

static bool segments_overlap_with_length(const RouteSegment& a,
                                         const RouteSegment& b) {
    if (a.a.y == a.b.y && b.a.y == b.b.y && a.a.y == b.a.y) {
        const long long lo = std::max(std::min(a.a.x, a.b.x),
                                      std::min(b.a.x, b.b.x));
        const long long hi = std::min(std::max(a.a.x, a.b.x),
                                      std::max(b.a.x, b.b.x));
        return lo < hi;
    }
    if (a.a.x == a.b.x && b.a.x == b.b.x && a.a.x == b.a.x) {
        const long long lo = std::max(std::min(a.a.y, a.b.y),
                                      std::min(b.a.y, b.b.y));
        const long long hi = std::min(std::max(a.a.y, a.b.y),
                                      std::max(b.a.y, b.b.y));
        return lo < hi;
    }
    return false;
}

static bool allow_same_cluster_global_touch(const EdgeInfo& edge,
                                            const RouteSegment& committed) {
    if (edge.policy != Policy::GlobalPatternThenMaze ||
        committed.policy == Policy::GlobalPatternThenMaze) {
        return false;
    }
    if (edge.child_kind != common::NodeKind::ClusterTop) {
        return false;
    }
    return edge.child >= 0 && committed.cluster_top == edge.child;
}

static bool point_on_allowed_same_cluster_route(
    const ScaledPoint& p,
    const EdgeInfo& edge,
    const std::vector<RouteSegment>& committed_segments) {
    for (const RouteSegment& committed : committed_segments) {
        if (allow_same_cluster_global_touch(edge, committed) &&
            point_on_segment(p, committed)) {
            return true;
        }
    }
    return false;
}

static std::vector<RouteSegment> polyline_segments(
    const std::vector<ScaledPoint>& points,
    Policy policy = Policy::Unknown,
    int cluster_top = -1) {
    std::vector<RouteSegment> segments;
    if (points.size() < 2U) {
        return segments;
    }
    segments.reserve(points.size() - 1U);
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (!same_point(points[i - 1U], points[i])) {
            segments.push_back(RouteSegment{points[i - 1U], points[i], policy, cluster_top});
        }
    }
    return segments;
}

static std::vector<ScaledPoint> expand_segment_points(const ScaledPoint& a,
                                                     const ScaledPoint& b) {
    std::vector<ScaledPoint> points;
    if (!is_manhattan_segment(a, b)) {
        return points;
    }
    points.push_back(a);
    if (a.x == b.x) {
        const long long step = (b.y > a.y) ? 1 : -1;
        for (long long y = a.y + step; y != b.y; y += step) {
            points.push_back(ScaledPoint{a.x, y});
        }
    } else {
        const long long step = (b.x > a.x) ? 1 : -1;
        for (long long x = a.x + step; x != b.x; x += step) {
            points.push_back(ScaledPoint{x, a.y});
        }
    }
    points.push_back(b);
    return points;
}

static void add_route_to_blocked(
    const std::vector<ScaledPoint>& polyline,
    const EdgeInfo& edge,
    std::unordered_set<PointKey, PointKeyHash>& committed_points,
    std::vector<RouteSegment>& committed_segments) {
    if (polyline.size() < 2U) {
        return;
    }
    const std::vector<RouteSegment> segments =
        polyline_segments(polyline, edge.policy, edge.cluster_top);
    committed_segments.insert(committed_segments.end(), segments.begin(), segments.end());
    for (std::size_t i = 1; i < polyline.size(); ++i) {
        const std::vector<ScaledPoint> segment =
            expand_segment_points(polyline[i - 1U], polyline[i]);
        for (std::size_t j = 1; j + 1 < segment.size(); ++j) {
            committed_points.insert(to_key(segment[j]));
        }
    }
    for (std::size_t i = 1; i + 1 < polyline.size(); ++i) {
        committed_points.insert(to_key(polyline[i]));
    }
}

static std::vector<ScaledPoint> canonical_candidate(std::vector<ScaledPoint> points) {
    canonicalize_polyline(points);
    return points;
}

static std::vector<long long> collect_track_values(
    const EdgeInfo& edge,
    const std::vector<ScaledPoint>& node_points,
    const std::vector<ScaledBBox>& node_bboxes,
    bool x_axis,
    long long die_limit) {
    std::set<long long> values;
    const long long start = x_axis ? edge.start.x : edge.start.y;
    const long long goal = x_axis ? edge.goal.x : edge.goal.y;
    values.insert(start);
    values.insert(goal);
    values.insert((start + goal) / 2);
    values.insert((start + goal + 1) / 2);
    values.insert((start + goal - 1) / 2);
    values.insert(start - 1);
    values.insert(start + 1);
    values.insert(goal - 1);
    values.insert(goal + 1);
    for (long long delta : {2LL, 3LL, 4LL, 6LL, 8LL}) {
        values.insert(start - delta);
        values.insert(start + delta);
        values.insert(goal - delta);
        values.insert(goal + delta);
    }
    if (edge.parent_is_source || edge.parent_kind == common::NodeKind::Global ||
        edge.child_kind == common::NodeKind::Global) {
        values.insert(start - 2);
        values.insert(start + 2);
        values.insert(goal - 2);
        values.insert(goal + 2);
    }
    if (edge.cluster_top >= 0 &&
        static_cast<std::size_t>(edge.cluster_top) < node_bboxes.size()) {
        const ScaledBBox& bbox = node_bboxes[static_cast<std::size_t>(edge.cluster_top)];
        const long long lo = x_axis ? bbox.lx : bbox.ly;
        const long long hi = x_axis ? bbox.ux : bbox.uy;
        values.insert(lo - 1);
        values.insert(lo + 1);
        values.insert(hi - 1);
        values.insert(hi + 1);
    }
    for (const ScaledPoint& p : node_points) {
        const long long v = x_axis ? p.x : p.y;
        values.insert(v);
        for (long long delta : {1LL, 2LL, 3LL, 4LL}) {
            values.insert(v - delta);
            values.insert(v + delta);
        }
    }
    for (const ScaledBBox& bbox : node_bboxes) {
        if (!bbox.valid) {
            continue;
        }
        const long long lo = x_axis ? bbox.lx : bbox.ly;
        const long long hi = x_axis ? bbox.ux : bbox.uy;
        for (long long delta : {1LL, 2LL, 4LL}) {
            values.insert(lo - delta);
            values.insert(lo + delta);
            values.insert(hi - delta);
            values.insert(hi + delta);
        }
        values.insert((lo + hi) / 2);
    }
    std::vector<long long> out;
    out.reserve(values.size());
    for (long long v : values) {
        if (v >= 0 && v <= die_limit) {
            out.push_back(v);
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

static bool append_pattern_candidate(std::vector<std::vector<ScaledPoint>>& candidates,
                                     std::set<std::string>& seen,
                                     std::vector<ScaledPoint> pts,
                                     Dir parent_exit_dir,
                                     Dir child_entry_dir) {
    pts = canonical_candidate(std::move(pts));
    if (pts.size() < 2U) {
        return false;
    }
    if (!candidate_matches_port_pair(pts, parent_exit_dir, child_entry_dir)) {
        return false;
    }
    const std::string key = polyline_key(pts);
    if (seen.insert(key).second) {
        candidates.push_back(std::move(pts));
        return true;
    }
    return false;
}

static std::vector<std::vector<ScaledPoint>> generate_pattern_candidates(
    const EdgeInfo& edge,
    const std::vector<ScaledPoint>& node_points,
    const std::vector<ScaledBBox>& node_bboxes,
    const std::vector<Dir>& parent_exit_dirs,
    const std::vector<Dir>& child_entry_dirs,
    int die_width,
    int die_height,
    int scale) {
    std::vector<std::vector<ScaledPoint>> candidates;
    std::set<std::string> seen;

    if (same_point(edge.start, edge.goal)) {
        return candidates;
    }

    const auto add_candidate = [&](std::vector<ScaledPoint> pts,
                                   Dir parent_exit_dir,
                                   Dir child_entry_dir) {
        append_pattern_candidate(candidates, seen, std::move(pts),
                                 parent_exit_dir, child_entry_dir);
    };

    const long long die_x = static_cast<long long>(die_width) * scale;
    const long long die_y = static_cast<long long>(die_height) * scale;
    const std::vector<long long> tracks_x =
        collect_track_values(edge, node_points, node_bboxes, true, die_x);
    const std::vector<long long> tracks_y =
        collect_track_values(edge, node_points, node_bboxes, false, die_y);

    for (Dir parent_exit_dir : parent_exit_dirs) {
        const ScaledPoint de = dir_vector(parent_exit_dir);
        const ScaledPoint parent_bend{edge.start.x + de.x, edge.start.y + de.y};
        if (!in_die(parent_bend, die_width, die_height, scale)) {
            continue;
        }
        for (Dir child_entry_dir : child_entry_dirs) {
            const ScaledPoint ce = dir_vector(child_entry_dir);
            const ScaledPoint child_bend{edge.goal.x + ce.x, edge.goal.y + ce.y};
            if (!in_die(child_bend, die_width, die_height, scale)) {
                continue;
            }

            if ((edge.start.x == edge.goal.x || edge.start.y == edge.goal.y) &&
                step_dir(edge.start, edge.goal) == parent_exit_dir &&
                opposite(step_dir(edge.start, edge.goal)) == child_entry_dir) {
                add_candidate({edge.start, edge.goal}, parent_exit_dir, child_entry_dir);
            }

            add_candidate({edge.start,
                           parent_bend,
                           ScaledPoint{child_bend.x, parent_bend.y},
                           edge.goal},
                          parent_exit_dir,
                          child_entry_dir);
            add_candidate({edge.start,
                           parent_bend,
                           ScaledPoint{parent_bend.x, child_bend.y},
                           edge.goal},
                          parent_exit_dir,
                          child_entry_dir);

            for (long long xm : tracks_x) {
                if (xm < 0 || xm > die_x) {
                    continue;
                }
                add_candidate({edge.start,
                               parent_bend,
                               ScaledPoint{xm, parent_bend.y},
                               ScaledPoint{xm, child_bend.y},
                               child_bend,
                               edge.goal},
                              parent_exit_dir,
                              child_entry_dir);
            }
            for (long long ym : tracks_y) {
                if (ym < 0 || ym > die_y) {
                    continue;
                }
                add_candidate({edge.start,
                               parent_bend,
                               ScaledPoint{parent_bend.x, ym},
                               ScaledPoint{child_bend.x, ym},
                               child_bend,
                               edge.goal},
                              parent_exit_dir,
                              child_entry_dir);
            }
        }
    }

    return candidates;
}

static Policy classify_policy(const EdgeInfo& edge) {
    if (edge.parent_is_source ||
        edge.parent_kind == common::NodeKind::Global ||
        edge.child_kind == common::NodeKind::Global) {
        return Policy::GlobalPatternThenMaze;
    }
    const auto is_access_bridge_top = [](common::NodeKind kind) {
        return kind == common::NodeKind::ClusterAccess ||
               kind == common::NodeKind::ClusterBridge ||
               kind == common::NodeKind::ClusterTop;
    };
    const bool touches_top_or_bridge =
        edge.parent_kind == common::NodeKind::ClusterTop ||
        edge.parent_kind == common::NodeKind::ClusterBridge ||
        edge.child_kind == common::NodeKind::ClusterTop ||
        edge.child_kind == common::NodeKind::ClusterBridge;
    if (is_access_bridge_top(edge.parent_kind) &&
        is_access_bridge_top(edge.child_kind) &&
        touches_top_or_bridge) {
        return Policy::ExternalAccessPatternThenMaze;
    }
    return Policy::LocalClusterPatternOnly;
}

static int policy_group(Policy policy) {
    switch (policy) {
        case Policy::GlobalPatternThenMaze:
            return 0;
        case Policy::ExternalAccessPatternThenMaze:
            return 1;
        case Policy::LocalClusterPatternOnly:
            return 2;
        case Policy::Unknown:
            return 3;
    }
    return 3;
}

static bool policy_allows_maze(Policy policy) {
    return policy == Policy::GlobalPatternThenMaze ||
           policy == Policy::ExternalAccessPatternThenMaze ||
           policy == Policy::LocalClusterPatternOnly;
}

static long long heuristic_cost(const ScaledPoint& a, const ScaledPoint& b) {
    return manhattan_scaled(a, b);
}

static Candidate run_maze_search(const EdgeInfo& edge,
                                 const std::vector<ScaledPoint>& node_points,
                                 const std::vector<ScaledBBox>& node_bboxes,
                                 const ScaledPoint& source_point,
                                 const std::unordered_set<PointKey, PointKeyHash>& committed_points,
                                 const std::vector<RouteSegment>& committed_segments,
                                 const std::vector<common::TopoNode>& topo_nodes,
                                 const common::Problem& problem,
                                 int scale,
                                 const ScaledBBox* allowed_bbox,
                                 bool restrict_to_allowed_bbox,
                                 int bend_weight,
                                 std::string& failure_reason) {
    Candidate cand;
    cand.maze_used = true;
    if (same_point(edge.start, edge.goal)) {
        failure_reason = "DEGENERATE";
        cand.maze_failed_reason = failure_reason;
        return cand;
    }

    auto blocked = [&](const ScaledPoint& p) {
        if (point_forbidden_for_edge(p, edge, node_points, source_point,
                                     committed_points)) {
            return true;
        }
        if (point_inside_forbidden_bbox(p, edge, node_bboxes, topo_nodes)) {
            return true;
        }
        const RouteSegment step_segment{edge.start, p};
        (void)step_segment;
        for (const RouteSegment& committed : committed_segments) {
            if (same_point(p, edge.start) || same_point(p, edge.goal)) {
                continue;
            }
            if (allow_same_cluster_global_touch(edge, committed) &&
                point_on_segment(p, committed)) {
                continue;
            }
            if (point_on_segment(p, committed)) {
                return true;
            }
        }
        return false;
    };

    static const std::array<std::pair<Dir, ScaledPoint>, 4> moves_list{{
        {Dir::Up, ScaledPoint{0, 1}},
        {Dir::Down, ScaledPoint{0, -1}},
        {Dir::Left, ScaledPoint{-1, 0}},
        {Dir::Right, ScaledPoint{1, 0}},
    }};

    std::priority_queue<AStarNode, std::vector<AStarNode>, AStarCompare> pq_fwd;
    std::priority_queue<AStarNode, std::vector<AStarNode>, AStarCompare> pq_bwd;
    std::unordered_map<ScaledState, long long, ScaledStateHash> best_g_fwd;
    std::unordered_map<ScaledState, long long, ScaledStateHash> best_g_bwd;
    std::unordered_map<ScaledState, ScaledState, ScaledStateHash> parent_fwd;
    std::unordered_map<ScaledState, ScaledState, ScaledStateHash> parent_bwd;
    std::unordered_map<PointKey, long long, PointKeyHash> point_best_fwd;
    std::unordered_map<PointKey, long long, PointKeyHash> point_best_bwd;
    std::unordered_map<PointKey, ScaledState, PointKeyHash> best_state_fwd;
    std::unordered_map<PointKey, ScaledState, PointKeyHash> best_state_bwd;

    int expanded_fwd = 0;
    int expanded_bwd = 0;
    bool found_meeting = false;
    ScaledState meeting_fwd;
    ScaledState meeting_bwd;
    long long meeting_cost = 0;

    auto expand_step = [&](const AStarNode& cur,
                           const ScaledPoint& goal,
                           std::priority_queue<AStarNode, std::vector<AStarNode>, AStarCompare>& pq,
                           std::unordered_map<ScaledState, long long, ScaledStateHash>& best_g,
                           std::unordered_map<ScaledState, ScaledState, ScaledStateHash>& parent_map,
                           bool is_forward,
                           int& expanded,
                           bool& found_meeting,
                           ScaledState& meeting_fwd,
                           ScaledState& meeting_bwd,
                           long long& meeting_cost) {
        const auto it_best = best_g.find(cur.state);
        if (it_best == best_g.end() || cur.g != it_best->second) {
            return;
        }

        if (same_point(cur.state.point, goal)) {
            found_meeting = true;
            meeting_fwd = (is_forward ? cur.state : ScaledState{goal, Dir::None});
            meeting_bwd = (is_forward ? ScaledState{goal, Dir::None} : cur.state);
            meeting_cost = cur.g;
            return;
        }

        for (const auto& move : moves_list) {
            const Dir dir = move.first;
            const ScaledPoint next{
                cur.state.point.x + move.second.x,
                cur.state.point.y + move.second.y,
            };
            if (!in_die(next, problem.die_width, problem.die_height, scale) &&
                !same_point(next, goal)) {
                continue;
            }
            if (restrict_to_allowed_bbox && allowed_bbox != nullptr &&
                !same_point(next, goal) &&
                (next.x < allowed_bbox->lx || next.x > allowed_bbox->ux ||
                 next.y < allowed_bbox->ly || next.y > allowed_bbox->uy)) {
                continue;
            }
            if (!same_point(next, goal) && blocked(next)) {
                continue;
            }
            long long step_cost = 1;
            if (cur.state.prev_dir != Dir::None && cur.state.prev_dir != dir) {
                step_cost += bend_weight;
            }
            if (cur.state.prev_dir == Dir::None) {
                const double factor =
                    is_forward ? preferred_dir_factor(edge, dir, false)
                               : preferred_dir_factor(edge, dir, true);
                step_cost += static_cast<long long>(factor * static_cast<double>(bend_weight));
            }
            if (edge.policy != Policy::LocalClusterPatternOnly &&
                !same_point(next, goal) &&
                point_inside_forbidden_bbox(next, edge, node_bboxes, topo_nodes)) {
                step_cost += std::max(1, bend_weight * 2);
            }
            const ScaledState next_state{next, dir};
            const long long tentative_g = cur.g + step_cost;
            const auto it = best_g.find(next_state);
            if (it != best_g.end() && tentative_g >= it->second) {
                continue;
            }
            best_g[next_state] = tentative_g;
            parent_map[next_state] = cur.state;
            const long long f = tentative_g + heuristic_cost(next, goal);
            pq.push(AStarNode{next_state, tentative_g, f});

            const PointKey pk = to_key(next);
            const auto other_point_it =
                is_forward ? point_best_bwd.find(pk) : point_best_fwd.find(pk);
            if (other_point_it != (is_forward ? point_best_bwd.end() : point_best_fwd.end())) {
                found_meeting = true;
                meeting_fwd = (is_forward ? next_state : best_state_fwd[pk]);
                meeting_bwd = (is_forward ? best_state_bwd[pk] : next_state);
                meeting_cost = tentative_g + other_point_it->second;
                return;
            }
            auto& point_map = is_forward ? point_best_fwd : point_best_bwd;
            auto& state_map = is_forward ? best_state_fwd : best_state_bwd;
            const auto pt_it = point_map.find(pk);
            if (pt_it == point_map.end() || tentative_g < pt_it->second) {
                point_map[pk] = tentative_g;
                state_map[pk] = next_state;
            }
            (void)expanded;
        }
    };

    const ScaledState start_state{edge.start, Dir::None};
    const ScaledState goal_state{edge.goal, Dir::None};
    pq_fwd.push(AStarNode{start_state, 0, heuristic_cost(edge.start, edge.goal)});
    pq_bwd.push(AStarNode{goal_state, 0, heuristic_cost(edge.goal, edge.start)});
    best_g_fwd[start_state] = 0;
    best_g_bwd[goal_state] = 0;
    point_best_fwd[to_key(edge.start)] = 0;
    point_best_bwd[to_key(edge.goal)] = 0;

    while (!pq_fwd.empty() || !pq_bwd.empty()) {
        if (!pq_fwd.empty()) {
            const AStarNode cur = pq_fwd.top();
            pq_fwd.pop();
            ++expanded_fwd;
            expand_step(cur, edge.goal, pq_fwd, best_g_fwd, parent_fwd,
                        true, expanded_fwd, found_meeting,
                        meeting_fwd, meeting_bwd, meeting_cost);
            if (found_meeting) break;
        }
        if (!pq_bwd.empty()) {
            const AStarNode cur = pq_bwd.top();
            pq_bwd.pop();
            ++expanded_bwd;
            expand_step(cur, edge.start, pq_bwd, best_g_bwd, parent_bwd,
                        false, expanded_bwd, found_meeting,
                        meeting_fwd, meeting_bwd, meeting_cost);
            if (found_meeting) break;
        }
    }

    cand.expanded_nodes = expanded_fwd + expanded_bwd;
    if (!found_meeting) {
        failure_reason = "NO_LEGAL_MAZE";
        cand.maze_failed_reason = failure_reason;
        return cand;
    }

    std::vector<ScaledPoint> path;
    {
        ScaledState walk = meeting_fwd;
        std::vector<ScaledState> fwd_chain;
        fwd_chain.push_back(walk);
        while (!(walk == start_state)) {
            const auto it = parent_fwd.find(walk);
            if (it == parent_fwd.end()) break;
            walk = it->second;
            fwd_chain.push_back(walk);
        }
        std::reverse(fwd_chain.begin(), fwd_chain.end());
        for (const ScaledState& s : fwd_chain) {
            path.push_back(s.point);
        }
    }
    if (!same_point(meeting_fwd.point, meeting_bwd.point)) {
        path.push_back(meeting_bwd.point);
    }
    {
        ScaledState walk = meeting_bwd;
        std::vector<ScaledState> bwd_chain;
        bwd_chain.push_back(walk);
        while (!(walk == goal_state)) {
            const auto it = parent_bwd.find(walk);
            if (it == parent_bwd.end()) break;
            walk = it->second;
            bwd_chain.push_back(walk);
        }
        for (std::size_t i = 1; i < bwd_chain.size(); ++i) {
            path.push_back(bwd_chain[i].point);
        }
    }
    canonicalize_polyline(path);
    cand.points = std::move(path);
    cand.shape = Shape::Maze;
    cand.wirelength = wirelength_of_polyline(cand.points, scale);
    cand.bends = bends_of_polyline(cand.points);
    cand.score = cand.wirelength + static_cast<double>(bend_weight) * cand.bends;
    cand.maze_failed_reason.clear();
    return cand;
}

static double preferred_penalty_weight(Policy policy) {
    switch (policy) {
        case Policy::GlobalPatternThenMaze:
            return 8.0;
        case Policy::ExternalAccessPatternThenMaze:
            return 20.0;
        case Policy::LocalClusterPatternOnly:
            return 4.0;
        case Policy::Unknown:
            return 10.0;
    }
    return 10.0;
}

static double bend_penalty_weight(Policy policy) {
    switch (policy) {
        case Policy::GlobalPatternThenMaze:
            return 2.0;
        case Policy::ExternalAccessPatternThenMaze:
            return 1.0;
        case Policy::LocalClusterPatternOnly:
            return 0.5;
        case Policy::Unknown:
            return 1.0;
    }
    return 1.0;
}

static Shape shape_from_polyline(const std::vector<ScaledPoint>& points) {
    const int bends = bends_of_polyline(points);
    if (points.size() < 2U) {
        return Shape::Failed;
    }
    if (bends == 0) {
        return Shape::I;
    }
    if (bends == 1) {
        return Shape::L;
    }
    if (bends >= 2) {
        return Shape::Z;
    }
    return Shape::Z;
}

static bool evaluate_candidate(
    const EdgeInfo& edge,
    Candidate& cand,
    const std::vector<ScaledPoint>& node_points,
    const std::vector<ScaledBBox>& node_bboxes,
    const ScaledPoint& source_point,
    const std::unordered_set<PointKey, PointKeyHash>& committed_points,
    const std::vector<RouteSegment>& committed_segments,
    const std::vector<common::TopoNode>& topo_nodes,
    const NodePorts& parent_ports,
    const NodePorts& child_ports,
    int die_width,
    int die_height,
    int scale,
    std::map<std::string, int>& reject_stats,
    std::string& failure_reason) {
    if (cand.points.size() < 2U) {
        ++reject_stats["DEGENERATE"];
        failure_reason = "DEGENERATE";
        return false;
    }
    canonicalize_polyline(cand.points);
    if (cand.points.size() < 2U) {
        ++reject_stats["DEGENERATE"];
        failure_reason = "DEGENERATE";
        return false;
    }
    if (!same_point(cand.points.front(), edge.start) ||
        !same_point(cand.points.back(), edge.goal)) {
        ++reject_stats["IMPOSSIBLE_ENDPOINT"];
        failure_reason = "IMPOSSIBLE_ENDPOINT";
        return false;
    }

    for (const ScaledPoint& p : cand.points) {
        if (!in_die(p, die_width, die_height, scale)) {
            ++reject_stats["OUT_OF_BOUNDARY"];
            failure_reason = "OUT_OF_BOUNDARY";
            return false;
        }
    }
    for (std::size_t i = 1; i + 1 < cand.points.size(); ++i) {
        const ScaledPoint& p = cand.points[i];
        if (point_forbidden_for_edge(p, edge, node_points,
                                     source_point, committed_points)) {
            if (committed_points.find(to_key(p)) != committed_points.end() &&
                point_on_allowed_same_cluster_route(p, edge, committed_segments)) {
                continue;
            }
            if (committed_points.find(to_key(p)) != committed_points.end()) {
                ++reject_stats["CROSSING_COMMITTED_ROUTE"];
                failure_reason = "CROSSING_COMMITTED_ROUTE";
                return false;
            }
            ++reject_stats["HIT_NODE"];
            failure_reason = "HIT_NODE";
            return false;
        }
        if (point_inside_forbidden_bbox(p, edge, node_bboxes, topo_nodes)) {
            ++reject_stats["HIT_BBOX"];
            failure_reason = "HIT_BBOX";
            return false;
        }
    }

    for (std::size_t i = 1; i < cand.points.size(); ++i) {
        if (!is_manhattan_segment(cand.points[i - 1U], cand.points[i])) {
            ++reject_stats["NON_MANHATTAN"];
            failure_reason = "NON_MANHATTAN";
            return false;
        }
    }

    std::unordered_set<PointKey, PointKeyHash> seen;
    seen.insert(to_key(cand.points.front()));
    for (std::size_t i = 1; i < cand.points.size(); ++i) {
        const ScaledPoint& a = cand.points[i - 1U];
        const ScaledPoint& b = cand.points[i];
        const long long dx = b.x - a.x;
        const long long dy = b.y - a.y;
        const long long steps = std::llabs(dx) + std::llabs(dy);
        for (long long s = 1; s < steps; ++s) {
            const ScaledPoint p{
                a.x + (dx == 0 ? 0 : (dx > 0 ? s : -s)),
                a.y + (dy == 0 ? 0 : (dy > 0 ? s : -s)),
            };
            if (!in_die(p, die_width, die_height, scale)) {
                ++reject_stats["OUT_OF_BOUNDARY"];
                failure_reason = "OUT_OF_BOUNDARY";
                return false;
            }
            if (point_forbidden_for_edge(p, edge, node_points,
                                         source_point, committed_points)) {
                if (committed_points.find(to_key(p)) != committed_points.end() &&
                    point_on_allowed_same_cluster_route(p, edge, committed_segments)) {
                    continue;
                }
                if (committed_points.find(to_key(p)) != committed_points.end()) {
                    ++reject_stats["CROSSING_COMMITTED_ROUTE"];
                    failure_reason = "CROSSING_COMMITTED_ROUTE";
                    return false;
                }
                if (same_point(p, source_point) ||
                    std::find(node_points.begin(), node_points.end(), p) != node_points.end()) {
                    ++reject_stats["HIT_NODE"];
                    failure_reason = "HIT_NODE";
                    return false;
                }
            }
            if (point_inside_forbidden_bbox(p, edge, node_bboxes, topo_nodes)) {
                ++reject_stats["HIT_BBOX"];
                failure_reason = "HIT_BBOX";
                return false;
            }
        }
        if (i + 1U < cand.points.size()) {
            const PointKey key = to_key(cand.points[i]);
            if (seen.find(key) != seen.end()) {
                ++reject_stats["SELF_INTERSECTION"];
                failure_reason = "SELF_INTERSECTION";
                return false;
            }
            seen.insert(key);
        }
    }

    const std::vector<RouteSegment> candidate_segments = polyline_segments(cand.points);
    for (std::size_t i = 0; i < candidate_segments.size(); ++i) {
        for (std::size_t j = i + 1; j < candidate_segments.size(); ++j) {
            const bool adjacent = (j == i + 1);
            ScaledPoint intersection;
            if (!segment_intersection_point(candidate_segments[i],
                                            candidate_segments[j],
                                            intersection)) {
                continue;
            }
            if (segments_overlap_with_length(candidate_segments[i],
                                             candidate_segments[j])) {
                ++reject_stats["SELF_INTERSECTION"];
                failure_reason = "SELF_INTERSECTION";
                return false;
            }
            if (adjacent &&
                same_point(candidate_segments[i].b, candidate_segments[j].a) &&
                same_point(intersection, candidate_segments[i].b)) {
                continue;
            }
            ++reject_stats["SELF_INTERSECTION"];
            failure_reason = "SELF_INTERSECTION";
            return false;
        }
    }

    for (const RouteSegment& cand_segment : candidate_segments) {
        for (const RouteSegment& committed : committed_segments) {
            ScaledPoint intersection;
            if (!segment_intersection_point(cand_segment, committed, intersection)) {
                continue;
            }
            if (segments_overlap_with_length(cand_segment, committed)) {
                ++reject_stats["OVERLAP_COMMITTED_ROUTE"];
                failure_reason = "OVERLAP_COMMITTED_ROUTE";
                return false;
            }
            if (allow_same_cluster_global_touch(edge, committed)) {
                continue;
            }
            const bool candidate_endpoint =
                same_point(intersection, edge.start) || same_point(intersection, edge.goal);
            if (candidate_endpoint && segment_endpoint(intersection, committed)) {
                continue;
            }
            ++reject_stats["CROSSING_COMMITTED_ROUTE"];
            failure_reason = "CROSSING_COMMITTED_ROUTE";
            return false;
        }
    }

    const Dir parent_exit = step_dir(cand.points.front(), cand.points[1U]);
    const Dir child_entry = opposite(step_dir(cand.points[cand.points.size() - 2U],
                                               cand.points.back()));
    if (parent_exit == Dir::None || child_entry == Dir::None) {
        ++reject_stats["NON_MANHATTAN"];
        failure_reason = "NON_MANHATTAN";
        return false;
    }
    cand.parent_exit_dir = parent_exit;
    cand.child_entry_dir = child_entry;
    cand.parent_port_available = port_free(parent_ports, parent_exit);
    cand.child_port_available = port_free(child_ports, child_entry);
    if (!cand.parent_port_available || !cand.child_port_available) {
        ++reject_stats["PORT_OCCUPIED"];
        failure_reason = "PORT_OCCUPIED";
        return false;
    }

    cand.parent_exit_factor = preferred_dir_factor(edge, parent_exit, false);
    cand.child_entry_factor = preferred_dir_factor(edge, child_entry, true);
    cand.used_preferred_parent = (cand.parent_exit_factor <= EPS);
    cand.used_preferred_child = (cand.child_entry_factor <= EPS);

    cand.wirelength = wirelength_of_polyline(cand.points, scale);
    cand.bends = bends_of_polyline(cand.points);
    if (!cand.maze_used) {
        const double direct = static_cast<double>(edge.manhattan_distance) / scale;
        if (cand.wirelength > direct + EPS) {
            ++reject_stats["PATTERN_DETOUR"];
            failure_reason = "PATTERN_DETOUR";
            return false;
        }
        if (cand.bends > 2) {
            ++reject_stats["ZSHAPE_BENDS"];
            failure_reason = "ZSHAPE_BENDS";
            return false;
        }
    }
    const double penalty = preferred_penalty_weight(edge.policy);
    const double bend_weight = bend_penalty_weight(edge.policy);
    const double direct_distance = static_cast<double>(edge.manhattan_distance) / scale;
    const double detour_penalty = std::max(0.0, cand.wirelength - direct_distance);
    double bbox_penalty = 0.0;
    for (std::size_t i = 1; i + 1 < cand.points.size(); ++i) {
            if (edge.policy != Policy::LocalClusterPatternOnly &&
                point_inside_forbidden_bbox(cand.points[i], edge, node_bboxes, topo_nodes)) {
                bbox_penalty += preferred_penalty_weight(edge.policy) * 0.5;
            }
    }
    double preferred_cost = penalty * (cand.parent_exit_factor + cand.child_entry_factor);
    double z_center_penalty = 0.0;
    if (cand.points.size() == 4U && cand.bends >= 2 &&
        !cand.maze_used) {
        const ScaledPoint& p1 = cand.points[1U];
        const ScaledPoint& p2 = cand.points[2U];
        constexpr double weight_z_center = 0.05;
        if (p1.x == p2.x) {
            const long long xm = p1.x;
            const long long cx = (edge.start.x + edge.goal.x) / 2LL;
            z_center_penalty = weight_z_center *
                static_cast<double>(std::llabs(xm - cx)) / scale;
        } else if (p1.y == p2.y) {
            const long long ym = p1.y;
            const long long cy = (edge.start.y + edge.goal.y) / 2LL;
            z_center_penalty = weight_z_center *
                static_cast<double>(std::llabs(ym - cy)) / scale;
        }
    }
    double reserved_top_port_cost = 0.0;
    if (edge.policy == Policy::ExternalAccessPatternThenMaze) {
        if (!edge.parent_is_source &&
            edge.parent_kind == common::NodeKind::ClusterTop &&
            edge.parent >= 0 &&
            static_cast<std::size_t>(edge.parent) < topo_nodes.size()) {
            const common::TopoNode& top_node =
                topo_nodes[static_cast<std::size_t>(edge.parent)];
            const int global_parent = topo_nodes[static_cast<std::size_t>(edge.parent)].parent;
            if (global_parent >= 0 &&
                static_cast<std::size_t>(global_parent) < node_points.size()) {
                const Dir toward_global =
                    dominant_dir_between(edge.start,
                                         node_points[static_cast<std::size_t>(global_parent)]);
                if (toward_global != Dir::None && cand.parent_exit_dir == toward_global) {
                    reserved_top_port_cost += 1000.0;
                }
            }
            if (edge.child_kind != common::NodeKind::ClusterBridge) {
                int bridge_child = -1;
                if (top_node.left >= 0 &&
                    static_cast<std::size_t>(top_node.left) < topo_nodes.size() &&
                    topo_nodes[static_cast<std::size_t>(top_node.left)].kind ==
                        common::NodeKind::ClusterBridge) {
                    bridge_child = top_node.left;
                }
                if (top_node.right >= 0 &&
                    static_cast<std::size_t>(top_node.right) < topo_nodes.size() &&
                    topo_nodes[static_cast<std::size_t>(top_node.right)].kind ==
                        common::NodeKind::ClusterBridge) {
                    bridge_child = top_node.right;
                }
                if (bridge_child >= 0 &&
                    static_cast<std::size_t>(bridge_child) < node_points.size()) {
                    const Dir toward_bridge =
                        dominant_dir_between(edge.start,
                                             node_points[static_cast<std::size_t>(bridge_child)]);
                    if (toward_bridge != Dir::None && cand.parent_exit_dir == toward_bridge) {
                        reserved_top_port_cost += 1000.0;
                    }
                }
            }
        }
        if (edge.child_kind == common::NodeKind::ClusterTop &&
            edge.child >= 0 &&
            static_cast<std::size_t>(edge.child) < topo_nodes.size()) {
            const int global_parent = topo_nodes[static_cast<std::size_t>(edge.child)].parent;
            if (global_parent >= 0 &&
                static_cast<std::size_t>(global_parent) < node_points.size()) {
                const Dir toward_global =
                    dominant_dir_between(edge.goal,
                                         node_points[static_cast<std::size_t>(global_parent)]);
                if (toward_global != Dir::None && cand.child_entry_dir == toward_global) {
                    reserved_top_port_cost += 1000.0;
                }
            }
        }
    }
    cand.score = cand.wirelength +
                  (cand.maze_used ? bend_weight * cand.bends : 0.0) +
                  preferred_cost + detour_penalty + bbox_penalty +
                  z_center_penalty + reserved_top_port_cost;
    cand.shape = cand.maze_used ? Shape::Maze : shape_from_polyline(cand.points);
    return true;
}

static void fill_debug_record(
    common::RouterEdgeDebug& debug,
    const EdgeInfo& edge,
    const Candidate* candidate,
    bool routed,
    const std::map<std::string, int>& reject_stats,
    int pattern_count,
    int maze_count,
    int legal_count,
    const std::string& failure_reason,
    int scale) {
    debug.edge_id = edge.topo_edge_index;
    debug.parent = edge.parent;
    debug.child = edge.child;
    debug.parent_class = edge.parent_class;
    debug.child_class = edge.child_class;
    debug.policy = policy_to_string(edge.policy);
    debug.pattern_candidate_count = pattern_count;
    debug.maze_candidate_count = maze_count;
    debug.legal_candidate_count = legal_count;
    debug.reject_stats = reject_stats;
    debug.routed = routed;
    debug.failure_reason = failure_reason;
    if (candidate != nullptr) {
        debug.selected_shape = shape_to_string(candidate->shape);
        debug.parent_exit_dir = dir_to_string(candidate->parent_exit_dir);
        debug.child_entry_dir = dir_to_string(candidate->child_entry_dir);
        debug.parent_port_available = candidate->parent_port_available;
        debug.child_port_available = candidate->child_port_available;
        debug.used_preferred_parent = candidate->used_preferred_parent;
        debug.used_preferred_child = candidate->used_preferred_child;
        debug.selected_score = candidate->score;
        debug.wirelength = candidate->wirelength;
        debug.bends = candidate->bends;
        debug.polyline.clear();
        debug.polyline.reserve(candidate->points.size());
        for (const ScaledPoint& p : candidate->points) {
            debug.polyline.push_back(unscale_point(p, scale));
        }
        if (!routed) {
            debug.selected_shape = "FAILED";
        }
    } else {
        debug.selected_shape = "FAILED";
        debug.parent_exit_dir = "NONE";
        debug.child_entry_dir = "NONE";
        debug.parent_port_available = false;
        debug.child_port_available = false;
        debug.used_preferred_parent = false;
        debug.used_preferred_child = false;
        debug.selected_score = 0.0;
        debug.wirelength = 0.0;
        debug.bends = 0;
        debug.polyline.clear();
    }
}

static std::string make_debug_route_path(const std::string& input_path) {
    std::filesystem::path path(input_path);
    const std::string base = path.stem().string();
    if (base.empty()) {
        return "route/route_debug.txt";
    }
    return "route/" + base + "_route.txt";
}

static bool ensure_route_dir(std::string& error_msg) {
    std::error_code ec;
    std::filesystem::create_directories("route", ec);
    if (ec) {
        error_msg = "Cannot create route directory: " + ec.message();
        return false;
    }
    return true;
}

static std::vector<EdgeInfo> build_edges(const common::Problem& problem,
                                         const common::TopoTree& tree,
                                         const common::LocerResult& loc_result,
                                         int scale,
                                         std::string& error_msg) {
    std::vector<EdgeInfo> edges;
    if (loc_result.node_results.size() != tree.nodes.size()) {
        error_msg = "Locer result node count does not match topology node count";
        return edges;
    }
    const ScaledPoint source = scale_point(problem.source.loc, scale);
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const common::LocerNodeResult& node = loc_result.node_results[i];
        if (!node.valid) {
            error_msg = "Locer node " + std::to_string(i) + " is invalid";
            return edges;
        }
        if (!std::isfinite(node.loc.x) || !std::isfinite(node.loc.y)) {
            error_msg = "Locer node " + std::to_string(i) + " has non-finite loc";
            return edges;
        }
    }

    std::vector<int> cluster_top_cache(tree.nodes.size(), -1);
    std::vector<int> source_depth_cache(tree.nodes.size(), 0);
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        cluster_top_cache[i] = find_cluster_top_ancestor(tree.nodes, static_cast<int>(i));
        source_depth_cache[i] = node_depth_to_source(tree.nodes, static_cast<int>(i),
                                                     source_depth_cache);
    }

    std::vector<int> source_children = tree.source_children;
    if (source_children.empty() && tree.root >= 0 &&
        static_cast<std::size_t>(tree.root) < tree.nodes.size()) {
        source_children.push_back(tree.root);
    }
    for (int child : source_children) {
        if (child < 0 || static_cast<std::size_t>(child) >= tree.nodes.size()) {
            error_msg = "Invalid source child node id " + std::to_string(child);
            return edges;
        }
        EdgeInfo edge;
        edge.parent_is_source = true;
        edge.parent = -1;
        edge.child = child;
        edge.parent_kind = common::NodeKind::Global;
        edge.child_kind = tree.nodes[static_cast<std::size_t>(child)].kind;
        edge.start = source;
        edge.goal = scale_point(loc_result.node_results[static_cast<std::size_t>(child)].loc, scale);
        edge.parent_class = node_class_to_string(common::NodeKind::Global, true);
        edge.child_class = node_class_to_string(edge.child_kind);
        edge.cluster_top = cluster_top_cache[static_cast<std::size_t>(child)];
        if (edge.cluster_top >= 0) {
            edge.cluster_bbox = tree.nodes[static_cast<std::size_t>(edge.cluster_top)].bbox;
            edge.has_cluster_bbox = true;
        }
        edge.cluster_depth = node_depth_to_cluster_top(tree.nodes, child, edge.cluster_top);
        edge.source_depth = 0;
        edge.policy = classify_policy(edge);
        edge.policy_group = policy_group(edge.policy);
        edge.manhattan_distance = manhattan_scaled(edge.start, edge.goal);
        edges.push_back(edge);
    }

    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const int parent = tree.nodes[i].parent;
        if (parent < 0) {
            continue;
        }
        if (static_cast<std::size_t>(parent) >= tree.nodes.size()) {
            error_msg = "Invalid parent node id " + std::to_string(parent) +
                        " for node " + std::to_string(i);
            return edges;
        }
        EdgeInfo edge;
        edge.parent = parent;
        edge.child = static_cast<int>(i);
        edge.parent_kind = tree.nodes[static_cast<std::size_t>(parent)].kind;
        edge.child_kind = tree.nodes[i].kind;
        edge.start = scale_point(loc_result.node_results[static_cast<std::size_t>(parent)].loc, scale);
        edge.goal = scale_point(loc_result.node_results[i].loc, scale);
        edge.parent_class = node_class_to_string(edge.parent_kind);
        edge.child_class = node_class_to_string(edge.child_kind);
        edge.cluster_top = cluster_top_cache[static_cast<std::size_t>(i)];
        if (edge.cluster_top >= 0) {
            edge.cluster_bbox = tree.nodes[static_cast<std::size_t>(edge.cluster_top)].bbox;
            edge.has_cluster_bbox = true;
        }
        edge.cluster_depth = node_depth_to_cluster_top(tree.nodes, static_cast<int>(i), edge.cluster_top);
        edge.source_depth = source_depth_cache[i];
        edge.policy = classify_policy(edge);
        edge.policy_group = policy_group(edge.policy);
        edge.manhattan_distance = manhattan_scaled(edge.start, edge.goal);
        edges.push_back(edge);
    }

    std::stable_sort(edges.begin(), edges.end(), [&](const EdgeInfo& a, const EdgeInfo& b) {
        const bool a_stage_a = a.policy != Policy::GlobalPatternThenMaze;
        const bool b_stage_a = b.policy != Policy::GlobalPatternThenMaze;
        if (a_stage_a != b_stage_a) {
            return a_stage_a > b_stage_a;
        }
        if (a_stage_a && b_stage_a) {
            if (a.cluster_top != b.cluster_top) {
                return a.cluster_top < b.cluster_top;
            }
            const int a_policy_priority =
                (a.policy == Policy::LocalClusterPatternOnly) ? 0 : 1;
            const int b_policy_priority =
                (b.policy == Policy::LocalClusterPatternOnly) ? 0 : 1;
            if (a_policy_priority != b_policy_priority) {
                return a_policy_priority < b_policy_priority;
            }
            if (a.policy == Policy::LocalClusterPatternOnly &&
                b.policy == Policy::LocalClusterPatternOnly) {
                if (a.cluster_depth != b.cluster_depth) {
                    return a.cluster_depth > b.cluster_depth;
                }
                if (a.manhattan_distance != b.manhattan_distance) {
                    return a.manhattan_distance < b.manhattan_distance;
                }
            } else if (a.policy == Policy::ExternalAccessPatternThenMaze &&
                       b.policy == Policy::ExternalAccessPatternThenMaze) {
                const int a_access_priority =
                    (a.child_kind == common::NodeKind::ClusterAccess ||
                     a.parent_kind == common::NodeKind::ClusterAccess) ? 0 : 1;
                const int b_access_priority =
                    (b.child_kind == common::NodeKind::ClusterAccess ||
                     b.parent_kind == common::NodeKind::ClusterAccess) ? 0 : 1;
                if (a_access_priority != b_access_priority) {
                    return a_access_priority < b_access_priority;
                }
                if (a.cluster_depth != b.cluster_depth) {
                    return a.cluster_depth > b.cluster_depth;
                }
                if (a.manhattan_distance != b.manhattan_distance) {
                    return a.manhattan_distance < b.manhattan_distance;
                }
            }
        } else {
            if (a.source_depth != b.source_depth) {
                return a.source_depth > b.source_depth;
            }
            const bool a_source_adjacent = a.parent_is_source;
            const bool b_source_adjacent = b.parent_is_source;
            if (a_source_adjacent != b_source_adjacent) {
                return !a_source_adjacent;
            }
            if (a.manhattan_distance != b.manhattan_distance) {
                return a.manhattan_distance < b.manhattan_distance;
            }
        }
        if (a.parent != b.parent) {
            return a.parent < b.parent;
        }
        return a.child < b.child;
    });

    for (std::size_t i = 0; i < edges.size(); ++i) {
        edges[i].topo_edge_index = static_cast<int>(i);
    }
    return edges;
}

static std::string edge_failure_message(const EdgeInfo& edge, const std::string& reason) {
    std::ostringstream oss;
    oss << "No legal route for edge " << edge.topo_edge_index
        << " (parent=" << edge.parent << ", child=" << edge.child
        << ", reason=" << reason << ")";
    return oss.str();
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_file_enable(bool enable) {
    g_debug_file_enabled = enable;
}

void debug_output(const RouterResult& result,
                  const common::Problem& problem,
                  const common::TopoTree& tree,
                  const common::LocerResult& loc_result) {
    if (!g_debug_enabled) {
        return;
    }
    (void)tree;
    int routed = 0;
    for (const auto& edge : result.edge_debugs) {
        if (edge.routed) {
            ++routed;
        }
    }
    std::cout << "[ROUTER] valid=" << (result.valid ? 1 : 0)
              << " error_msg=" << result.error_msg
              << " num_edges=" << result.edge_debugs.size()
              << " num_routed_edges=" << routed << "\n";

    std::map<std::string, std::array<int, 3>> policy_stats;
    std::map<std::string, int> policy_routed;
    for (const auto& edge : result.edge_debugs) {
        auto& stats = policy_stats[edge.policy];
        stats[0] += 1;
        if (edge.routed) {
            stats[1] += 1;
        } else {
            stats[2] += 1;
        }
        if (edge.routed) {
            ++policy_routed[edge.policy];
        }
    }
    for (const auto& kv : policy_stats) {
        std::cout << "[ROUTER] policy=" << kv.first
                  << " edge_count=" << kv.second[0]
                  << " routed_count=" << kv.second[1]
                  << " failed_count=" << kv.second[2] << "\n";
    }

    std::cout << "[ROUTER] route_order=[";
    for (std::size_t i = 0; i < result.edge_debugs.size(); ++i) {
        if (i > 0) {
            std::cout << ",";
        }
        std::cout << result.edge_debugs[i].edge_id;
    }
    std::cout << "]\n";

    const auto print_loc = [&](int node_id, bool is_source) {
        if (is_source || node_id < 0) {
            std::cout << "(" << problem.source.loc.x << "," << problem.source.loc.y << ")";
            return;
        }
        if (static_cast<std::size_t>(node_id) < loc_result.node_results.size()) {
            const common::SegmentPoint& p = loc_result.node_results[static_cast<std::size_t>(node_id)].loc;
            std::cout << "(" << p.x << "," << p.y << ")";
            return;
        }
        std::cout << "(?,?)";
    };

    for (const auto& edge : result.edge_debugs) {
        std::cout << "[ROUTER] edge_id=" << edge.edge_id
                  << " parent=" << edge.parent
                  << " child=" << edge.child
                  << " parent_class=" << edge.parent_class
                  << " child_class=" << edge.child_class
                  << " policy=" << edge.policy << "\n";
        std::cout << "[ROUTER]   parent_loc=";
        print_loc(edge.parent, edge.parent_class == "SOURCE");
        std::cout << " child_loc=";
        print_loc(edge.child, false);
        std::cout << "\n";
        std::cout << "[ROUTER]   parent_port_available="
                  << (edge.parent_port_available ? 1 : 0)
                  << " child_port_available="
                  << (edge.child_port_available ? 1 : 0)
                  << " used_preferred_parent="
                  << (edge.used_preferred_parent ? 1 : 0)
                  << " used_preferred_child="
                  << (edge.used_preferred_child ? 1 : 0) << "\n";
        std::cout << "[ROUTER]   selected_shape=" << edge.selected_shape
                  << " parent_exit_dir=" << edge.parent_exit_dir
                  << " child_entry_dir=" << edge.child_entry_dir
                  << " candidate_count=" << edge.pattern_candidate_count
                  << " maze_candidate_count=" << edge.maze_candidate_count
                  << " legal_candidate_count=" << edge.legal_candidate_count << "\n";
        std::cout << "[ROUTER]   selected_score=" << edge.selected_score
                  << " wirelength=" << edge.wirelength
                  << " bends=" << edge.bends
                  << " routed=" << (edge.routed ? 1 : 0)
                  << " failure_reason=" << edge.failure_reason << "\n";
        if (edge.maze_used) {
            std::cout << "[ROUTER]   maze_used=1 maze_expanded_nodes="
                      << edge.maze_expanded_nodes
                      << " maze_best_cost=" << edge.maze_best_cost
                      << " maze_failed_reason=" << edge.maze_failed_reason << "\n";
        }
        if (!edge.reject_stats.empty()) {
            std::cout << "[ROUTER]   reject_stats";
            for (const auto& kv : edge.reject_stats) {
                std::cout << " " << kv.first << "=" << kv.second;
            }
            std::cout << "\n";
        }
        std::cout << "[ROUTER]   polyline=";
        for (std::size_t i = 0; i < edge.polyline.size(); ++i) {
            if (i > 0) {
                std::cout << "->";
            }
            std::cout << "(" << edge.polyline[i].x << "," << edge.polyline[i].y << ")";
        }
        std::cout << "\n";
    }
}

bool write_debug_route_file(const RouterResult& result,
                            const common::Problem&,
                            const common::TopoTree&,
                            const common::LocerResult&,
                            const std::string& input_path,
                            std::string& error_msg) {
    if (!ensure_route_dir(error_msg)) {
        return false;
    }
    const std::string path = make_debug_route_path(input_path);
    std::ofstream fout(path);
    if (!fout) {
        error_msg = "Cannot open route debug file: " + path;
        return false;
    }
    fout << "# ROUTER_DEBUG_ROUTE v1\n";
    fout << "# valid=" << (result.valid ? 1 : 0) << "\n";
    fout << "# num_edges=" << result.edge_debugs.size() << "\n";
    fout << "# columns: edge_id parent child parent_class child_class policy selected_shape parent_exit_dir child_entry_dir score wirelength bends pattern_candidate_count maze_candidate_count legal_candidate_count failure_reason point_count points...\n";
    for (const auto& edge : result.edge_debugs) {
        fout << "edge " << edge.edge_id << " "
             << edge.parent << " " << edge.child << " "
             << edge.parent_class << " "
             << edge.child_class << " "
             << edge.policy << " "
             << edge.selected_shape << " "
             << edge.parent_exit_dir << " "
             << edge.child_entry_dir << " "
             << edge.selected_score << " "
             << edge.wirelength << " "
             << edge.bends << " "
             << edge.pattern_candidate_count << " "
             << edge.maze_candidate_count << " "
             << edge.legal_candidate_count << " "
             << edge.failure_reason << " "
             << edge.polyline.size();
        fout << std::fixed;
        for (const common::SegmentPoint& p : edge.polyline) {
            fout << " " << p.x << " " << p.y;
        }
        fout << "\n";
    }
    return true;
}

RouterResult run(const common::Problem& problem,
                 const common::TopoTree& tree,
                 const common::LocerResult& loc_result,
                 const std::string& input_path) {
    RouterResult result;
    if (!problem.valid) {
        result.error_msg = "Cannot run router on invalid problem: " + problem.error_msg;
        return result;
    }
    if (!tree.valid) {
        result.error_msg = "Cannot run router on invalid topology tree: " + tree.error_msg;
        return result;
    }
    if (!loc_result.valid) {
        result.error_msg = "Cannot run router on invalid locer result: " + loc_result.error_msg;
        return result;
    }
    if (tree.nodes.empty()) {
        result.error_msg = "Cannot run router on empty topology tree";
        return result;
    }
    if (loc_result.node_results.size() != tree.nodes.size()) {
        result.error_msg = "Locer result node count does not match topology node count";
        return result;
    }
    if (!std::isfinite(problem.source.loc.x) || !std::isfinite(problem.source.loc.y)) {
        result.error_msg = "Problem source location is invalid";
        return result;
    }

    const int scale = choose_scale(problem, tree, loc_result);
    std::string build_error;
    std::vector<EdgeInfo> edges = build_edges(problem, tree, loc_result, scale, build_error);
    if (!build_error.empty()) {
        result.error_msg = build_error;
        return result;
    }

    const ScaledPoint source_point = scale_point(problem.source.loc, scale);
    std::vector<ScaledPoint> node_points(tree.nodes.size());
    std::vector<ScaledBBox> node_bboxes(tree.nodes.size());
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        node_points[i] = scale_point(loc_result.node_results[i].loc, scale);
        node_bboxes[i] = scaled_bbox(tree.nodes[i].bbox, scale);
    }

    std::vector<NodePorts> node_ports(tree.nodes.size());
    NodePorts source_ports;
    std::unordered_set<PointKey, PointKeyHash> committed_points;
    std::vector<RouteSegment> committed_segments;

    result.edge_debugs.resize(edges.size());
    bool failure = false;
    std::string failure_message;

    for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
        const EdgeInfo& edge = edges[edge_index];
        common::RouterEdgeDebug debug;
        debug.edge_id = edge.topo_edge_index;
        debug.parent = edge.parent;
        debug.child = edge.child;
        debug.parent_class = edge.parent_class;
        debug.child_class = edge.child_class;
        debug.policy = policy_to_string(edge.policy);

        if (failure) {
            fill_debug_record(debug, edge, nullptr, false, std::map<std::string, int>{},
                              0, 0, 0, "NOT_VISITED", scale);
            result.edge_debugs[edge_index] = debug;
            continue;
        }

        const bool local_stage = edge.policy == Policy::LocalClusterPatternOnly;
        const ScaledBBox* related_bbox =
            (edge.has_cluster_bbox && edge.cluster_top >= 0)
                ? &node_bboxes[static_cast<std::size_t>(edge.cluster_top)]
                : nullptr;
        const std::vector<Dir> parent_dirs_unsorted =
            edge.parent_is_source ? available_dirs(source_ports)
                                  : available_dirs(node_ports[static_cast<std::size_t>(edge.parent)]);
        const std::vector<Dir> child_dirs_unsorted =
            available_dirs(node_ports[static_cast<std::size_t>(edge.child)]);
        std::vector<Dir> parent_dirs = parent_dirs_unsorted;
        std::vector<Dir> child_dirs = child_dirs_unsorted;
        std::stable_sort(parent_dirs.begin(), parent_dirs.end(),
                         [&edge](Dir a, Dir b) {
                             return preferred_dir_factor(edge, a, false) <
                                    preferred_dir_factor(edge, b, false);
                         });
        std::stable_sort(child_dirs.begin(), child_dirs.end(),
                         [&edge](Dir a, Dir b) {
                             return preferred_dir_factor(edge, a, true) <
                                    preferred_dir_factor(edge, b, true);
                         });

        const std::vector<std::vector<ScaledPoint>> pattern_candidates =
            generate_pattern_candidates(edge, node_points, node_bboxes,
                                        parent_dirs, child_dirs,
                                        problem.die_width, problem.die_height, scale);
        std::map<std::string, int> reject_stats;
        int maze_candidate_count = 0;
        int legal_count = 0;
        Candidate best;
        bool have_best = false;
        std::string best_reason;
        bool maze_attempted = false;

        for (const std::vector<ScaledPoint>& raw_candidate : pattern_candidates) {
            Candidate cand;
            cand.points = raw_candidate;
            ++debug.pattern_candidate_count;
            std::string reason;
            const bool legal = evaluate_candidate(edge, cand, node_points, node_bboxes,
                                                  source_point, committed_points,
                                                  committed_segments,
                                                  tree.nodes,
                                                  edge.parent_is_source ? source_ports : node_ports[static_cast<std::size_t>(edge.parent)],
                                                  node_ports[static_cast<std::size_t>(edge.child)],
                                                  problem.die_width, problem.die_height, scale,
                                                  reject_stats, reason);
            if (!legal) {
                continue;
            }
            ++legal_count;
            if (!have_best || cand.score < best.score) {
                best = cand;
                have_best = true;
                best_reason.clear();
            }
        }

        if (!have_best && policy_allows_maze(edge.policy)) {
            maze_attempted = true;
            std::string maze_fail_reason;
            const int bend_weight = static_cast<int>(std::lround(bend_penalty_weight(edge.policy)));
            ScaledBBox local_search_bbox;
            const ScaledBBox* search_bbox = nullptr;
            bool restrict_to_search_bbox = false;
            if (local_stage && related_bbox != nullptr) {
                local_search_bbox = *related_bbox;
                const long long margin =
                    std::max<long long>(4 * scale, std::min<long long>(20 * scale, edge.manhattan_distance / 2 + scale));
                local_search_bbox = expand_bbox_to_point(local_search_bbox, edge.start, margin);
                local_search_bbox = expand_bbox_to_point(local_search_bbox, edge.goal, margin);
                local_search_bbox = clamp_bbox_to_die(local_search_bbox,
                                                      problem.die_width,
                                                      problem.die_height,
                                                      scale);
                search_bbox = &local_search_bbox;
                restrict_to_search_bbox = true;
            }
            Candidate maze_candidate = run_maze_search(edge, node_points, node_bboxes,
                                                       source_point, committed_points,
                                                       committed_segments, tree.nodes,
                                                       problem, scale, search_bbox,
                                                       restrict_to_search_bbox, bend_weight,
                                                       maze_fail_reason);
            ++maze_candidate_count;
            debug.maze_used = true;
            debug.maze_expanded_nodes = maze_candidate.expanded_nodes;
            if (!maze_candidate.points.empty()) {
                Candidate cand = maze_candidate;
                std::string reason;
                const bool legal = evaluate_candidate(edge, cand, node_points, node_bboxes,
                                                      source_point, committed_points,
                                                      committed_segments,
                                                      tree.nodes,
                                                      edge.parent_is_source ? source_ports : node_ports[static_cast<std::size_t>(edge.parent)],
                                                      node_ports[static_cast<std::size_t>(edge.child)],
                                                      problem.die_width, problem.die_height, scale,
                                                      reject_stats, reason);
                if (legal) {
                    ++legal_count;
                    if (!have_best || cand.score < best.score) {
                        best = cand;
                        have_best = true;
                        best_reason.clear();
                    }
                } else {
                    maze_fail_reason = reason;
                }
            }
            if (have_best) {
                debug.maze_best_cost = best.score;
            } else {
                debug.maze_failed_reason = maze_fail_reason;
            }
        }

        debug.maze_candidate_count = maze_candidate_count;
        debug.legal_candidate_count = legal_count;
        debug.reject_stats = reject_stats;

        if (!have_best) {
            debug.selected_shape = "FAILED";
            debug.failure_reason = !best_reason.empty() ? best_reason
                                                        : (policy_allows_maze(edge.policy)
                                                               ? (debug.maze_failed_reason.empty()
                                                                      ? "NO_LEGAL_ROUTE"
                                                                      : debug.maze_failed_reason)
                                                               : "NO_LEGAL_PATTERN");
            debug.routed = false;
            fill_debug_record(debug, edge, nullptr, false, reject_stats,
                              debug.pattern_candidate_count, debug.maze_candidate_count,
                              debug.legal_candidate_count, debug.failure_reason, scale);
            result.edge_debugs[edge_index] = debug;
            failure = true;
            failure_message = edge_failure_message(edge, debug.failure_reason);
            continue;
        }

        add_route_to_blocked(best.points, edge, committed_points, committed_segments);
        if (edge.parent_is_source) {
            occupy_port(source_ports, best.parent_exit_dir);
        } else {
            occupy_port(node_ports[static_cast<std::size_t>(edge.parent)], best.parent_exit_dir);
        }
        occupy_port(node_ports[static_cast<std::size_t>(edge.child)], best.child_entry_dir);

        debug.selected_shape = shape_to_string(best.shape);
        debug.parent_exit_dir = dir_to_string(best.parent_exit_dir);
        debug.child_entry_dir = dir_to_string(best.child_entry_dir);
        debug.parent_port_available = best.parent_port_available;
        debug.child_port_available = best.child_port_available;
        debug.used_preferred_parent = best.used_preferred_parent;
        debug.used_preferred_child = best.used_preferred_child;
        debug.selected_score = best.score;
        debug.wirelength = best.wirelength;
        debug.bends = best.bends;
        debug.routed = true;
        debug.failure_reason = "OK";
        debug.polyline.clear();
        debug.polyline.reserve(best.points.size());
        for (const ScaledPoint& p : best.points) {
            debug.polyline.push_back(unscale_point(p, scale));
        }
        debug.maze_used = maze_attempted || best.shape == Shape::Maze;
        debug.maze_expanded_nodes = best.expanded_nodes;
        if (maze_attempted) {
            debug.maze_best_cost = best.score;
        }
        debug.maze_failed_reason = best.maze_failed_reason;
        fill_debug_record(debug, edge, &best, true, reject_stats,
                          debug.pattern_candidate_count, debug.maze_candidate_count,
                          debug.legal_candidate_count, "OK", scale);
        result.edge_debugs[edge_index] = debug;
    }

    if (failure) {
        result.valid = false;
        result.error_msg = failure_message;
    } else {
        result.valid = true;
    }

    if (g_debug_enabled) {
        debug_output(result, problem, tree, loc_result);
    }
    if (g_debug_file_enabled) {
        std::string file_error;
        result.output_path = make_debug_route_path(input_path);
        if (!write_debug_route_file(result, problem, tree, loc_result, input_path, file_error)) {
            result.valid = false;
            result.error_msg = file_error;
        }
    } else {
        result.output_path = make_debug_route_path(input_path);
    }
    return result;
}

}  // namespace router
