#include "detourer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace detourer {
namespace {

static bool g_debug_enabled = false;
static bool g_debug_file_enabled = false;

static constexpr double EPS = 1e-6;
static constexpr int kMaxDetourLevel = 100;

enum class Dir {
    None = 0,
    Up,
    Down,
    Left,
    Right,
};

struct IntPoint {
    int x = 0;
    int y = 0;
};

struct EdgeKey {
    int parent = -1;
    int child = -1;

    bool operator<(const EdgeKey& other) const {
        if (parent != other.parent) {
            return parent < other.parent;
        }
        return child < other.child;
    }
};

struct Segment {
    IntPoint a;
    IntPoint b;
    int edge_index = -1;
};

struct EdgeState {
    int edge_index = -1;
    int edge_id = -1;
    int parent = -1;
    int child = -1;
    IntPoint original_start;
    IntPoint original_goal;
    Dir original_parent_exit = Dir::None;
    Dir original_child_entry = Dir::None;
    std::vector<IntPoint> original_points;
    std::vector<IntPoint> points;
};

struct ActiveDetour {
    int edge_index = -1;
    int point_index = -1;
    int segment_index = -1;
    int anchor_index = -1;
    int level = 1;
    Dir side = Dir::None;
};

struct CandidateAnchor {
    int edge_index = -1;
    int segment_index = -1;
    int anchor_index = -1;
    Dir side = Dir::None;
    int segment_length = 0;
    double center_distance = 0.0;
    double anchor_distance = 0.0;
};

static bool operator==(const IntPoint& a, const IntPoint& b) {
    return a.x == b.x && a.y == b.y;
}

static bool operator!=(const IntPoint& a, const IntPoint& b) {
    return !(a == b);
}

static int round_to_int(double v) {
    return static_cast<int>(std::llround(v));
}

static bool near_integer(double v) {
    return std::abs(v - std::round(v)) <= EPS;
}

static IntPoint round_point(common::SegmentPoint p) {
    return IntPoint{round_to_int(p.x), round_to_int(p.y)};
}

static common::SegmentPoint to_segment_point(const IntPoint& p) {
    return common::SegmentPoint{static_cast<double>(p.x), static_cast<double>(p.y)};
}

static IntPoint source_point(const common::Problem& problem) {
    return IntPoint{problem.source.loc.x, problem.source.loc.y};
}

static std::string point_string(const IntPoint& p) {
    return "(" + std::to_string(p.x) + "," + std::to_string(p.y) + ")";
}

static int manhattan(const IntPoint& a, const IntPoint& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

static bool in_die(const common::Problem& problem, const IntPoint& p) {
    return p.x >= 0 && p.x <= problem.die_width &&
           p.y >= 0 && p.y <= problem.die_height;
}

static bool is_manhattan_segment(const IntPoint& a, const IntPoint& b) {
    return a.x == b.x || a.y == b.y;
}

static Dir step_dir(const IntPoint& a, const IntPoint& b) {
    if (a.x == b.x) {
        if (a.y < b.y) return Dir::Up;
        if (a.y > b.y) return Dir::Down;
    } else if (a.y == b.y) {
        if (a.x < b.x) return Dir::Right;
        if (a.x > b.x) return Dir::Left;
    }
    return Dir::None;
}

static Dir opposite(Dir dir) {
    switch (dir) {
        case Dir::Up: return Dir::Down;
        case Dir::Down: return Dir::Up;
        case Dir::Left: return Dir::Right;
        case Dir::Right: return Dir::Left;
        case Dir::None: return Dir::None;
    }
    return Dir::None;
}

static IntPoint dir_vector(Dir dir) {
    switch (dir) {
        case Dir::Up: return IntPoint{0, 1};
        case Dir::Down: return IntPoint{0, -1};
        case Dir::Left: return IntPoint{-1, 0};
        case Dir::Right: return IntPoint{1, 0};
        case Dir::None: return IntPoint{0, 0};
    }
    return IntPoint{0, 0};
}

static std::string dir_string(Dir dir) {
    switch (dir) {
        case Dir::Up: return "UP";
        case Dir::Down: return "DOWN";
        case Dir::Left: return "LEFT";
        case Dir::Right: return "RIGHT";
        case Dir::None: return "NONE";
    }
    return "NONE";
}

static void append_unique(std::vector<IntPoint>& points, const IntPoint& p) {
    if (points.empty() || points.back() != p) {
        points.push_back(p);
    }
}

static int polyline_length(const std::vector<IntPoint>& points) {
    int total = 0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        total += manhattan(points[i - 1U], points[i]);
    }
    return total;
}

static int bends_of_polyline(const std::vector<IntPoint>& points) {
    int bends = 0;
    for (std::size_t i = 2; i < points.size(); ++i) {
        const Dir a = step_dir(points[i - 2U], points[i - 1U]);
        const Dir b = step_dir(points[i - 1U], points[i]);
        if (a != Dir::None && b != Dir::None && a != b) {
            ++bends;
        }
    }
    return bends;
}

static std::string shape_from_bends(int bends) {
    if (bends <= 0) return "I";
    if (bends == 1) return "L";
    if (bends == 2) return "Z";
    return "MAZE";
}

static std::vector<IntPoint> expand_segment_points(const IntPoint& a,
                                                   const IntPoint& b) {
    std::vector<IntPoint> points;
    if (!is_manhattan_segment(a, b)) {
        return points;
    }
    points.push_back(a);
    if (a.x == b.x) {
        const int step = (b.y > a.y) ? 1 : -1;
        for (int y = a.y + step; y != b.y; y += step) {
            points.push_back(IntPoint{a.x, y});
        }
    } else {
        const int step = (b.x > a.x) ? 1 : -1;
        for (int x = a.x + step; x != b.x; x += step) {
            points.push_back(IntPoint{x, a.y});
        }
    }
    points.push_back(b);
    return points;
}

static bool point_on_segment(const IntPoint& p, const Segment& s) {
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

static bool segment_endpoint(const IntPoint& p, const Segment& s) {
    return p == s.a || p == s.b;
}

static bool segment_intersection(const Segment& a,
                                 const Segment& b,
                                 IntPoint& out) {
    const bool a_h = a.a.y == a.b.y;
    const bool a_v = a.a.x == a.b.x;
    const bool b_h = b.a.y == b.b.y;
    const bool b_v = b.a.x == b.b.x;
    if (!((a_h || a_v) && (b_h || b_v))) {
        return false;
    }
    if (a_h && b_v) {
        const IntPoint p{b.a.x, a.a.y};
        if (point_on_segment(p, a) && point_on_segment(p, b)) {
            out = p;
            return true;
        }
        return false;
    }
    if (a_v && b_h) {
        const IntPoint p{a.a.x, b.a.y};
        if (point_on_segment(p, a) && point_on_segment(p, b)) {
            out = p;
            return true;
        }
        return false;
    }
    if (a_h && b_h && a.a.y == b.a.y) {
        const int lo = std::max(std::min(a.a.x, a.b.x), std::min(b.a.x, b.b.x));
        const int hi = std::min(std::max(a.a.x, a.b.x), std::max(b.a.x, b.b.x));
        if (lo > hi) {
            return false;
        }
        out = IntPoint{lo, a.a.y};
        return true;
    }
    if (a_v && b_v && a.a.x == b.a.x) {
        const int lo = std::max(std::min(a.a.y, a.b.y), std::min(b.a.y, b.b.y));
        const int hi = std::min(std::max(a.a.y, a.b.y), std::max(b.a.y, b.b.y));
        if (lo > hi) {
            return false;
        }
        out = IntPoint{a.a.x, lo};
        return true;
    }
    return false;
}

static bool segments_overlap_with_length(const Segment& a, const Segment& b) {
    if (a.a.y == a.b.y && b.a.y == b.b.y && a.a.y == b.a.y) {
        const int lo = std::max(std::min(a.a.x, a.b.x), std::min(b.a.x, b.b.x));
        const int hi = std::min(std::max(a.a.x, a.b.x), std::max(b.a.x, b.b.x));
        return lo < hi;
    }
    if (a.a.x == a.b.x && b.a.x == b.b.x && a.a.x == b.a.x) {
        const int lo = std::max(std::min(a.a.y, a.b.y), std::min(b.a.y, b.b.y));
        const int hi = std::min(std::max(a.a.y, a.b.y), std::max(b.a.y, b.b.y));
        return lo < hi;
    }
    return false;
}

static std::vector<Segment> polyline_segments(const std::vector<IntPoint>& points,
                                              int edge_index) {
    std::vector<Segment> segments;
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (points[i - 1U] != points[i]) {
            segments.push_back(Segment{points[i - 1U], points[i], edge_index});
        }
    }
    return segments;
}

static bool point_strictly_inside_bbox(const IntPoint& p, const common::BBox& bbox) {
    return p.x > bbox.lx && p.x < bbox.ux &&
           p.y > bbox.ly && p.y < bbox.uy;
}

static int find_cluster_top_ancestor(const common::TopoTree& tree, int node_id) {
    int cur = node_id;
    while (cur >= 0 && static_cast<std::size_t>(cur) < tree.nodes.size()) {
        if (tree.nodes[static_cast<std::size_t>(cur)].kind == common::NodeKind::ClusterTop) {
            return cur;
        }
        cur = tree.nodes[static_cast<std::size_t>(cur)].parent;
    }
    return -1;
}

static std::vector<int> children_of(const common::TopoTree& tree, int node_id) {
    std::vector<int> children;
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) {
        return children;
    }
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    if (node.left >= 0 && static_cast<std::size_t>(node.left) < tree.nodes.size()) {
        children.push_back(node.left);
    }
    if (node.right >= 0 && static_cast<std::size_t>(node.right) < tree.nodes.size() &&
        node.right != node.left) {
        children.push_back(node.right);
    }
    if (!children.empty()) {
        return children;
    }
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        if (tree.nodes[i].parent == node_id) {
            children.push_back(static_cast<int>(i));
        }
    }
    return children;
}

static void dfs_postorder(const common::TopoTree& tree,
                          int node_id,
                          std::vector<int>& state,
                          std::vector<int>& order) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) {
        return;
    }
    int& mark = state[static_cast<std::size_t>(node_id)];
    if (mark != 0) {
        return;
    }
    mark = 1;
    for (int child : children_of(tree, node_id)) {
        dfs_postorder(tree, child, state, order);
    }
    mark = 2;
    order.push_back(node_id);
}

static std::vector<int> build_bottom_up_order(const common::TopoTree& tree) {
    std::vector<int> state(tree.nodes.size(), 0);
    std::vector<int> order;
    if (!tree.source_children.empty()) {
        for (int child : tree.source_children) {
            dfs_postorder(tree, child, state, order);
        }
    } else if (tree.root >= 0) {
        dfs_postorder(tree, tree.root, state, order);
    }
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        dfs_postorder(tree, static_cast<int>(i), state, order);
    }
    return order;
}

static bool round_router_polyline(const common::RouterEdgeDebug& edge,
                                  std::vector<IntPoint>& points,
                                  std::string& err) {
    if (!edge.routed || edge.failure_reason != "OK") {
        err = "Router edge " + std::to_string(edge.edge_id) +
              " is not successfully routed: " + edge.failure_reason;
        return false;
    }
    if (edge.polyline.size() < 2U) {
        err = "Router edge " + std::to_string(edge.edge_id) +
              " has fewer than two points";
        return false;
    }
    points.clear();
    for (const common::SegmentPoint& p : edge.polyline) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            err = "Router edge " + std::to_string(edge.edge_id) +
                  " has non-finite polyline point";
            return false;
        }
        if ((!near_integer(p.x) || !near_integer(p.y)) && g_debug_enabled) {
            std::cerr << "[DETOURER][WARN] non-integer router point on edge "
                      << edge.edge_id << ", rounded from (" << p.x << ","
                      << p.y << ")\n";
        }
        append_unique(points, round_point(p));
    }
    if (points.size() < 2U) {
        err = "Router edge " + std::to_string(edge.edge_id) +
              " has fewer than two unique rounded points";
        return false;
    }
    return true;
}

static bool build_edge_states(const common::Problem& problem,
                              const common::TopoTree& tree,
                              const common::LocerResult& loc_result,
                              const common::RouterResult& route_result,
                              std::vector<EdgeState>& edges,
                              std::map<EdgeKey, int>& edge_map,
                              std::string& err) {
    edges.clear();
    edge_map.clear();
    for (std::size_t i = 0; i < route_result.edge_debugs.size(); ++i) {
        const common::RouterEdgeDebug& debug = route_result.edge_debugs[i];
        if (debug.child < 0 || static_cast<std::size_t>(debug.child) >= tree.nodes.size()) {
            err = "Router edge " + std::to_string(debug.edge_id) + " has invalid child";
            return false;
        }
        if (debug.parent < -1 ||
            (debug.parent >= 0 && static_cast<std::size_t>(debug.parent) >= tree.nodes.size())) {
            err = "Router edge " + std::to_string(debug.edge_id) + " has invalid parent";
            return false;
        }

        EdgeState edge;
        edge.edge_index = static_cast<int>(i);
        edge.edge_id = debug.edge_id;
        edge.parent = debug.parent;
        edge.child = debug.child;
        if (!round_router_polyline(debug, edge.points, err)) {
            return false;
        }
        edge.original_points = edge.points;
        edge.original_start = debug.parent < 0
                                  ? source_point(problem)
                                  : round_point(loc_result.node_results[static_cast<std::size_t>(debug.parent)].loc);
        edge.original_goal = round_point(loc_result.node_results[static_cast<std::size_t>(debug.child)].loc);
        if (edge.points.front() != edge.original_start || edge.points.back() != edge.original_goal) {
            err = "Router edge " + std::to_string(debug.edge_id) +
                  " endpoint mismatch before detour";
            return false;
        }
        edge.original_parent_exit = step_dir(edge.points.front(), edge.points[1U]);
        edge.original_child_entry = opposite(step_dir(edge.points[edge.points.size() - 2U],
                                                       edge.points.back()));
        if (edge.original_parent_exit == Dir::None || edge.original_child_entry == Dir::None) {
            err = "Router edge " + std::to_string(debug.edge_id) +
                  " has invalid endpoint direction";
            return false;
        }
        const EdgeKey key{edge.parent, edge.child};
        if (edge_map.count(key) != 0U) {
            err = "Duplicate router edge for parent " + std::to_string(edge.parent) +
                  " child " + std::to_string(edge.child);
            return false;
        }
        edge_map[key] = edge.edge_index;
        edges.push_back(std::move(edge));
    }
    return true;
}

static bool validate_global_inputs(const common::Problem& problem,
                                   const common::TopoTree& tree,
                                   const common::LocerResult& loc_result,
                                   const common::RouterResult& route_result,
                                   std::string& err) {
    if (!problem.valid) {
        err = "Invalid problem: " + problem.error_msg;
        return false;
    }
    if (!tree.valid) {
        err = "Invalid topology tree: " + tree.error_msg;
        return false;
    }
    if (!loc_result.valid) {
        err = "Invalid locer result: " + loc_result.error_msg;
        return false;
    }
    if (!route_result.valid) {
        err = "Invalid router result: " + route_result.error_msg;
        return false;
    }
    if (tree.nodes.empty()) {
        err = "Topology tree is empty";
        return false;
    }
    if (loc_result.node_results.size() != tree.nodes.size()) {
        err = "Locer node count does not match tree node count";
        return false;
    }
    if (!in_die(problem, source_point(problem))) {
        err = "SOURCE out of die";
        return false;
    }
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const common::LocerNodeResult& node = loc_result.node_results[i];
        if (!node.valid) {
            err = "Locer node " + std::to_string(i) + " is invalid";
            return false;
        }
        if (!std::isfinite(node.loc.x) || !std::isfinite(node.loc.y)) {
            err = "Locer node " + std::to_string(i) + " has non-finite loc";
            return false;
        }
        if (!near_integer(node.loc.x) || !near_integer(node.loc.y)) {
            err = "Locer node " + std::to_string(i) + " is not on integer grid";
            return false;
        }
        if (!in_die(problem, round_point(node.loc))) {
            err = "Locer node " + std::to_string(i) + " out of die";
            return false;
        }
    }
    return true;
}

static bool point_hits_forbidden_node(const IntPoint& p,
                                      const EdgeState& edge,
                                      const common::Problem& problem,
                                      const common::LocerResult& loc_result) {
    if (p == edge.original_start || p == edge.original_goal) {
        return false;
    }
    if (p == source_point(problem)) {
        return true;
    }
    for (std::size_t i = 0; i < loc_result.node_results.size(); ++i) {
        const IntPoint loc = round_point(loc_result.node_results[i].loc);
        if (p == loc) {
            return true;
        }
    }
    return false;
}

static bool point_hits_forbidden_bbox(const IntPoint& p,
                                      const EdgeState& edge,
                                      const common::TopoTree& tree) {
    if (p == edge.original_start || p == edge.original_goal) {
        return false;
    }
    const int cluster_top = find_cluster_top_ancestor(tree, edge.child);
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        if (tree.nodes[i].kind != common::NodeKind::ClusterTop) {
            continue;
        }
        if (static_cast<int>(i) == edge.parent ||
            static_cast<int>(i) == edge.child ||
            static_cast<int>(i) == cluster_top) {
            continue;
        }
        if (point_strictly_inside_bbox(p, tree.nodes[i].bbox)) {
            return true;
        }
    }
    return false;
}

static bool validate_single_edge_polyline(const common::Problem& problem,
                                          const common::TopoTree& tree,
                                          const common::LocerResult& loc_result,
                                          const EdgeState& edge,
                                          const std::vector<IntPoint>& points,
                                          std::string& err) {
    if (points.size() < 2U) {
        err = "Edge " + std::to_string(edge.edge_id) + " has fewer than two points";
        return false;
    }
    if (points.front() != edge.original_start || points.back() != edge.original_goal) {
        err = "Edge " + std::to_string(edge.edge_id) + " endpoint changed";
        return false;
    }
    if (step_dir(points.front(), points[1U]) != edge.original_parent_exit ||
        opposite(step_dir(points[points.size() - 2U], points.back())) != edge.original_child_entry) {
        err = "Edge " + std::to_string(edge.edge_id) + " endpoint direction changed";
        return false;
    }
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!in_die(problem, points[i])) {
            err = "Edge " + std::to_string(edge.edge_id) +
                  " point out of die: " + point_string(points[i]);
            return false;
        }
        if (point_hits_forbidden_node(points[i], edge, problem, loc_result)) {
            err = "Edge " + std::to_string(edge.edge_id) +
                  " hits node/source at " + point_string(points[i]);
            return false;
        }
        if (point_hits_forbidden_bbox(points[i], edge, tree)) {
            err = "Edge " + std::to_string(edge.edge_id) +
                  " hits forbidden bbox at " + point_string(points[i]);
            return false;
        }
    }
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (points[i - 1U] == points[i]) {
            err = "Edge " + std::to_string(edge.edge_id) + " has zero-length segment";
            return false;
        }
        if (!is_manhattan_segment(points[i - 1U], points[i])) {
            err = "Edge " + std::to_string(edge.edge_id) + " has non-Manhattan segment";
            return false;
        }
        const std::vector<IntPoint> expanded =
            expand_segment_points(points[i - 1U], points[i]);
        for (const IntPoint& p : expanded) {
            if (!in_die(problem, p)) {
                err = "Edge " + std::to_string(edge.edge_id) +
                      " segment leaves die at " + point_string(p);
                return false;
            }
            if (point_hits_forbidden_node(p, edge, problem, loc_result)) {
                err = "Edge " + std::to_string(edge.edge_id) +
                      " segment hits node/source at " + point_string(p);
                return false;
            }
            if (point_hits_forbidden_bbox(p, edge, tree)) {
                err = "Edge " + std::to_string(edge.edge_id) +
                      " segment hits forbidden bbox at " + point_string(p);
                return false;
            }
        }
    }

    const std::vector<Segment> segments = polyline_segments(points, edge.edge_index);
    for (std::size_t i = 0; i < segments.size(); ++i) {
        for (std::size_t j = i + 1U; j < segments.size(); ++j) {
            const bool adjacent = (j == i + 1U);
            IntPoint intersection;
            if (!segment_intersection(segments[i], segments[j], intersection)) {
                continue;
            }
            if (segments_overlap_with_length(segments[i], segments[j])) {
                err = "Edge " + std::to_string(edge.edge_id) + " self-overlaps";
                return false;
            }
            if (adjacent && intersection == segments[i].b && intersection == segments[j].a) {
                continue;
            }
            err = "Edge " + std::to_string(edge.edge_id) +
                  " self-intersects at " + point_string(intersection);
            return false;
        }
    }
    return true;
}

static bool validate_against_other_edges(const EdgeState& edge,
                                         const std::vector<IntPoint>& points,
                                         const std::vector<EdgeState>& edges,
                                         std::string& err) {
    const std::vector<Segment> candidate_segments =
        polyline_segments(points, edge.edge_index);
    for (const Segment& cand : candidate_segments) {
        for (const EdgeState& other : edges) {
            if (other.edge_index == edge.edge_index) {
                continue;
            }
            const std::vector<Segment> other_segments =
                polyline_segments(other.points, other.edge_index);
            for (const Segment& other_segment : other_segments) {
                IntPoint intersection;
                if (!segment_intersection(cand, other_segment, intersection)) {
                    continue;
                }
                if (segments_overlap_with_length(cand, other_segment)) {
                    err = "Edge " + std::to_string(edge.edge_id) +
                          " overlaps edge " + std::to_string(other.edge_id);
                    return false;
                }
                const bool edge_topology_endpoint =
                    intersection == edge.original_start ||
                    intersection == edge.original_goal;
                const bool other_topology_endpoint =
                    intersection == other.original_start ||
                    intersection == other.original_goal;
                if (edge_topology_endpoint && other_topology_endpoint &&
                    segment_endpoint(intersection, cand) &&
                    segment_endpoint(intersection, other_segment)) {
                    continue;
                }
                err = "Edge " + std::to_string(edge.edge_id) +
                      " crosses edge " + std::to_string(other.edge_id) +
                      " at " + point_string(intersection);
                return false;
            }
        }
    }
    return true;
}

static bool validate_candidate_polyline(const common::Problem& problem,
                                        const common::TopoTree& tree,
                                        const common::LocerResult& loc_result,
                                        const EdgeState& edge,
                                        const std::vector<IntPoint>& points,
                                        const std::vector<EdgeState>& edges,
                                        std::string& err) {
    return validate_single_edge_polyline(problem, tree, loc_result, edge, points, err) &&
           validate_against_other_edges(edge, points, edges, err);
}

static bool validate_all_routes(const common::Problem& problem,
                                const common::TopoTree& tree,
                                const common::LocerResult& loc_result,
                                const std::vector<EdgeState>& edges,
                                std::string& err) {
    for (const EdgeState& edge : edges) {
        if (!validate_candidate_polyline(problem, tree, loc_result,
                                         edge, edge.points, edges, err)) {
            return false;
        }
    }
    return true;
}

static std::vector<IntPoint> insert_detour_points(const std::vector<IntPoint>& points,
                                                  int segment_index,
                                                  int anchor_index,
                                                  Dir side,
                                                  int level,
                                                  int* point_index_out) {
    const IntPoint start = points[static_cast<std::size_t>(segment_index)];
    const IntPoint end = points[static_cast<std::size_t>(segment_index + 1)];
    const Dir along = step_dir(start, end);
    const IntPoint along_vec = dir_vector(along);
    const IntPoint side_vec = dir_vector(side);
    const IntPoint a{start.x + along_vec.x * anchor_index,
                     start.y + along_vec.y * anchor_index};
    const IntPoint b{start.x + along_vec.x * (anchor_index + 1),
                     start.y + along_vec.y * (anchor_index + 1)};
    const IntPoint a_off{a.x + side_vec.x * level, a.y + side_vec.y * level};
    const IntPoint b_off{b.x + side_vec.x * level, b.y + side_vec.y * level};

    std::vector<IntPoint> out;
    out.reserve(points.size() + 4U);
    for (int i = 0; i <= segment_index; ++i) {
        append_unique(out, points[static_cast<std::size_t>(i)]);
    }
    append_unique(out, a);
    if (point_index_out != nullptr) {
        *point_index_out = static_cast<int>(out.size()) - 1;
    }
    append_unique(out, a_off);
    append_unique(out, b_off);
    append_unique(out, b);
    for (std::size_t i = static_cast<std::size_t>(segment_index + 1);
         i < points.size(); ++i) {
        append_unique(out, points[i]);
    }
    return out;
}

static bool violates_adjacent_rule(const CandidateAnchor& cand,
                                   const std::vector<ActiveDetour>& active_detours) {
    for (const ActiveDetour& detour : active_detours) {
        if (detour.edge_index != cand.edge_index ||
            detour.segment_index != cand.segment_index ||
            detour.side != cand.side) {
            continue;
        }
        const int a0 = cand.anchor_index;
        const int a1 = cand.anchor_index + 1;
        const int b0 = detour.anchor_index;
        const int b1 = detour.anchor_index + 1;
        const int gap = std::max(a0, b0) - std::min(a1, b1);
        if (gap <= 1) {
            return true;
        }
    }
    return false;
}

static std::vector<CandidateAnchor> generate_candidates_for_edge(
    const EdgeState& edge,
    const std::vector<ActiveDetour>& active_detours) {
    struct SegmentChoice {
        int segment_index = -1;
        int length = 0;
        double center_distance = 0.0;
    };

    std::vector<int> cumulative(edge.points.size(), 0);
    for (std::size_t i = 1; i < edge.points.size(); ++i) {
        cumulative[i] = cumulative[i - 1U] + manhattan(edge.points[i - 1U], edge.points[i]);
    }
    const double edge_center = static_cast<double>(cumulative.back()) / 2.0;
    std::vector<SegmentChoice> segments;
    for (std::size_t i = 1; i < edge.points.size(); ++i) {
        const int len = manhattan(edge.points[i - 1U], edge.points[i]);
        if (len < 3) {
            continue;
        }
        const double seg_center =
            static_cast<double>(cumulative[i - 1U]) + static_cast<double>(len) / 2.0;
        segments.push_back(SegmentChoice{
            static_cast<int>(i - 1U),
            len,
            std::abs(seg_center - edge_center),
        });
    }
    std::stable_sort(segments.begin(), segments.end(),
                     [](const SegmentChoice& a, const SegmentChoice& b) {
                         if (a.length != b.length) return a.length > b.length;
                         if (std::abs(a.center_distance - b.center_distance) > EPS) {
                             return a.center_distance < b.center_distance;
                         }
                         return a.segment_index < b.segment_index;
                     });

    std::vector<CandidateAnchor> out;
    for (const SegmentChoice& seg : segments) {
        const IntPoint start = edge.points[static_cast<std::size_t>(seg.segment_index)];
        const IntPoint end = edge.points[static_cast<std::size_t>(seg.segment_index + 1)];
        const Dir along = step_dir(start, end);
        std::vector<Dir> sides;
        if (along == Dir::Left || along == Dir::Right) {
            sides = {Dir::Up, Dir::Down};
        } else {
            sides = {Dir::Right, Dir::Left};
        }
        std::vector<int> anchors;
        for (int d = 1; d <= seg.length - 2; ++d) {
            anchors.push_back(d);
        }
        std::stable_sort(anchors.begin(), anchors.end(),
                         [len = seg.length](int a, int b) {
                             const double ca = std::abs((static_cast<double>(a) + 0.5) -
                                                        static_cast<double>(len) / 2.0);
                             const double cb = std::abs((static_cast<double>(b) + 0.5) -
                                                        static_cast<double>(len) / 2.0);
                             if (std::abs(ca - cb) > EPS) return ca < cb;
                             return a < b;
                         });
        for (int anchor : anchors) {
            const IntPoint along_vec = dir_vector(along);
            const IntPoint a{start.x + along_vec.x * anchor,
                             start.y + along_vec.y * anchor};
            const IntPoint b{start.x + along_vec.x * (anchor + 1),
                             start.y + along_vec.y * (anchor + 1)};
            if (manhattan(a, edge.points.front()) <= 1 ||
                manhattan(b, edge.points.front()) <= 1 ||
                manhattan(a, edge.points.back()) <= 1 ||
                manhattan(b, edge.points.back()) <= 1) {
                continue;
            }
            for (Dir side : sides) {
                CandidateAnchor cand;
                cand.edge_index = edge.edge_index;
                cand.segment_index = seg.segment_index;
                cand.anchor_index = anchor;
                cand.side = side;
                cand.segment_length = seg.length;
                cand.center_distance = seg.center_distance;
                cand.anchor_distance = std::abs((static_cast<double>(anchor) + 0.5) -
                                                static_cast<double>(seg.length) / 2.0);
                if (!violates_adjacent_rule(cand, active_detours)) {
                    out.push_back(cand);
                }
            }
        }
    }
    return out;
}

static bool compute_profiles(const common::TopoTree& tree,
                             const std::map<EdgeKey, int>& edge_map,
                             const std::vector<EdgeState>& edges,
                             const std::vector<int>& bottom_up_order,
                             std::vector<common::DetourNodeResult>& node_results,
                             std::string& err) {
    node_results.assign(tree.nodes.size(), common::DetourNodeResult{});
    for (int node_id : bottom_up_order) {
        common::DetourNodeResult node_result;
        node_result.node_id = node_id;
        const std::vector<int> children = children_of(tree, node_id);
        if (children.empty()) {
            node_result.sink_delays_to_node.push_back(0.0);
        } else {
            for (int child : children) {
                if (child < 0 || static_cast<std::size_t>(child) >= node_results.size() ||
                    !node_results[static_cast<std::size_t>(child)].valid) {
                    err = "Child delay profile is unavailable for node " +
                          std::to_string(node_id);
                    return false;
                }
                const auto edge_it = edge_map.find(EdgeKey{node_id, child});
                if (edge_it == edge_map.end()) {
                    err = "Missing routed edge for parent " + std::to_string(node_id) +
                          " child " + std::to_string(child);
                    return false;
                }
                const double edge_delay =
                    static_cast<double>(polyline_length(edges[static_cast<std::size_t>(edge_it->second)].points));
                for (double delay : node_results[static_cast<std::size_t>(child)].sink_delays_to_node) {
                    node_result.sink_delays_to_node.push_back(delay + edge_delay);
                }
            }
        }
        if (node_result.sink_delays_to_node.empty()) {
            node_result.sink_delays_to_node.push_back(0.0);
        }
        const auto [min_it, max_it] =
            std::minmax_element(node_result.sink_delays_to_node.begin(),
                                node_result.sink_delays_to_node.end());
        node_result.min_sink_delay_to_node = *min_it;
        node_result.max_sink_delay_to_node = *max_it;
        node_result.skew_to_node =
            node_result.max_sink_delay_to_node - node_result.min_sink_delay_to_node;
        node_result.valid = true;
        node_results[static_cast<std::size_t>(node_id)] = std::move(node_result);
    }
    return true;
}

static bool collect_subtree_edges(int node_id,
                                  const common::TopoTree& tree,
                                  const std::map<EdgeKey, int>& edge_map,
                                  std::set<int>& out) {
    for (int child : children_of(tree, node_id)) {
        const auto it = edge_map.find(EdgeKey{node_id, child});
        if (it != edge_map.end()) {
            out.insert(it->second);
        }
        collect_subtree_edges(child, tree, edge_map, out);
    }
    return true;
}

static std::vector<int> candidate_edges_for_side(int parent,
                                                 int child,
                                                 const common::TopoTree& tree,
                                                 const std::map<EdgeKey, int>& edge_map,
                                                 const std::vector<EdgeState>& edges) {
    std::vector<int> result;
    const auto direct = edge_map.find(EdgeKey{parent, child});
    if (direct != edge_map.end()) {
        result.push_back(direct->second);
    }
    std::set<int> subtree_edges;
    collect_subtree_edges(child, tree, edge_map, subtree_edges);
    std::vector<int> rest;
    for (int edge_index : subtree_edges) {
        if (direct == edge_map.end() || edge_index != direct->second) {
            rest.push_back(edge_index);
        }
    }
    std::stable_sort(rest.begin(), rest.end(),
                     [&](int a, int b) {
                         const int la = polyline_length(edges[static_cast<std::size_t>(a)].points);
                         const int lb = polyline_length(edges[static_cast<std::size_t>(b)].points);
                         if (la != lb) return la > lb;
                         return edges[static_cast<std::size_t>(a)].edge_id <
                                edges[static_cast<std::size_t>(b)].edge_id;
                     });
    result.insert(result.end(), rest.begin(), rest.end());
    return result;
}

static bool delta_improves(double delta, double incremental_added) {
    const double before = std::abs(delta);
    const double after = std::abs(delta - incremental_added);
    if (after + EPS < before) {
        return true;
    }
    return after <= before + EPS && incremental_added <= delta + EPS;
}

static void shift_active_detours_after_insert(std::vector<ActiveDetour>& active_detours,
                                              int edge_index,
                                              int inserted_after_segment,
                                              int point_shift) {
    for (ActiveDetour& detour : active_detours) {
        if (detour.edge_index == edge_index &&
            detour.point_index > inserted_after_segment) {
            detour.point_index += point_shift;
        }
    }
}

static bool try_insert_level_one(const common::Problem& problem,
                                 const common::TopoTree& tree,
                                 const common::LocerResult& loc_result,
                                 const std::vector<int>& candidate_edge_indices,
                                 double delta,
                                 std::vector<EdgeState>& edges,
                                 std::vector<ActiveDetour>& active_detours,
                                 common::DetourerResult& result) {
    if (!delta_improves(delta, 2.0)) {
        return false;
    }
    for (int edge_index : candidate_edge_indices) {
        EdgeState& edge = edges[static_cast<std::size_t>(edge_index)];
        const std::vector<CandidateAnchor> candidates =
            generate_candidates_for_edge(edge, active_detours);
        for (const CandidateAnchor& cand : candidates) {
            int point_index = -1;
            std::vector<IntPoint> updated =
                insert_detour_points(edge.points, cand.segment_index,
                                     cand.anchor_index, cand.side, 1, &point_index);
            std::string err;
            if (!validate_candidate_polyline(problem, tree, loc_result,
                                             edge, updated, edges, err)) {
                continue;
            }
            const int old_size = static_cast<int>(edge.points.size());
            edge.points = std::move(updated);
            const int point_shift = static_cast<int>(edge.points.size()) - old_size;
            shift_active_detours_after_insert(active_detours, edge_index,
                                              cand.segment_index, point_shift);
            active_detours.push_back(ActiveDetour{
                edge_index,
                point_index,
                cand.segment_index,
                cand.anchor_index,
                1,
                cand.side,
            });

            common::DetourRecord record;
            record.edge_id = edge.edge_id;
            record.node_parent = edge.parent;
            record.node_child = edge.child;
            record.segment_index = cand.segment_index;
            record.anchor_index = cand.anchor_index;
            record.level = 1;
            record.added_delay = 2;
            record.side = dir_string(cand.side);
            record.upgraded = false;
            result.detour_records.push_back(std::move(record));
            return true;
        }
    }
    return false;
}

static bool try_upgrade_detour(const common::Problem& problem,
                               const common::TopoTree& tree,
                               const common::LocerResult& loc_result,
                               const std::vector<int>& candidate_edge_indices,
                               int new_level,
                               double delta,
                               std::vector<EdgeState>& edges,
                               std::vector<ActiveDetour>& active_detours,
                               common::DetourerResult& result) {
    if (!delta_improves(delta, 2.0)) {
        return false;
    }
    for (int edge_index : candidate_edge_indices) {
        for (ActiveDetour& detour : active_detours) {
            if (detour.edge_index != edge_index || detour.level != new_level - 1) {
                continue;
            }
            EdgeState& edge = edges[static_cast<std::size_t>(edge_index)];
            if (detour.point_index < 0 ||
                static_cast<std::size_t>(detour.point_index + 3) >= edge.points.size()) {
                continue;
            }
            const IntPoint a = edge.points[static_cast<std::size_t>(detour.point_index)];
            const IntPoint b = edge.points[static_cast<std::size_t>(detour.point_index + 3)];
            const IntPoint side_vec = dir_vector(detour.side);
            std::vector<IntPoint> updated = edge.points;
            updated[static_cast<std::size_t>(detour.point_index + 1)] =
                IntPoint{a.x + side_vec.x * new_level, a.y + side_vec.y * new_level};
            updated[static_cast<std::size_t>(detour.point_index + 2)] =
                IntPoint{b.x + side_vec.x * new_level, b.y + side_vec.y * new_level};
            std::string err;
            if (!validate_candidate_polyline(problem, tree, loc_result,
                                             edge, updated, edges, err)) {
                continue;
            }
            edge.points = std::move(updated);
            detour.level = new_level;

            common::DetourRecord record;
            record.edge_id = edge.edge_id;
            record.node_parent = edge.parent;
            record.node_child = edge.child;
            record.segment_index = detour.segment_index;
            record.anchor_index = detour.anchor_index;
            record.level = new_level;
            record.added_delay = 2;
            record.side = dir_string(detour.side);
            record.upgraded = true;
            result.detour_records.push_back(std::move(record));
            return true;
        }
    }
    return false;
}

static bool try_detour_growth(const common::Problem& problem,
                              const common::TopoTree& tree,
                              const common::LocerResult& loc_result,
                              const std::vector<int>& candidate_edge_indices,
                              double delta,
                              std::vector<EdgeState>& edges,
                              std::vector<ActiveDetour>& active_detours,
                              common::DetourerResult& result) {
    if (try_insert_level_one(problem, tree, loc_result, candidate_edge_indices,
                             delta, edges, active_detours, result)) {
        return true;
    }
    for (int level = 2; level <= kMaxDetourLevel; ++level) {
        if (try_upgrade_detour(problem, tree, loc_result, candidate_edge_indices,
                               level, delta, edges, active_detours, result)) {
            return true;
        }
    }
    return false;
}

static bool child_worst_delay(int parent,
                              int child,
                              const std::map<EdgeKey, int>& edge_map,
                              const std::vector<EdgeState>& edges,
                              const std::vector<common::DetourNodeResult>& node_results,
                              double& out) {
    const auto edge_it = edge_map.find(EdgeKey{parent, child});
    if (edge_it == edge_map.end() ||
        child < 0 || static_cast<std::size_t>(child) >= node_results.size() ||
        !node_results[static_cast<std::size_t>(child)].valid) {
        return false;
    }
    out = node_results[static_cast<std::size_t>(child)].max_sink_delay_to_node +
          static_cast<double>(polyline_length(edges[static_cast<std::size_t>(edge_it->second)].points));
    return true;
}

static std::string make_debug_detour_path(const std::string& input_path) {
    std::filesystem::path path(input_path);
    const std::string base = path.stem().string();
    if (base.empty()) {
        return "detour/detour_debug.txt";
    }
    return "detour/" + base + "_detour.txt";
}

static bool ensure_detour_dir(std::string& error_msg) {
    std::error_code ec;
    std::filesystem::create_directories("detour", ec);
    if (ec) {
        error_msg = "Cannot create detour directory: " + ec.message();
        return false;
    }
    return true;
}

static void write_back_route_result(const std::vector<EdgeState>& edges,
                                    common::RouterResult& route_result) {
    for (const EdgeState& edge : edges) {
        common::RouterEdgeDebug& debug =
            route_result.edge_debugs[static_cast<std::size_t>(edge.edge_index)];
        debug.polyline.clear();
        debug.polyline.reserve(edge.points.size());
        for (const IntPoint& p : edge.points) {
            debug.polyline.push_back(to_segment_point(p));
        }
        debug.wirelength = static_cast<double>(polyline_length(edge.points));
        debug.bends = bends_of_polyline(edge.points);
        debug.selected_shape = shape_from_bends(debug.bends);
    }
}

static bool final_profile_matches(const common::TopoTree& tree,
                                  const std::map<EdgeKey, int>& edge_map,
                                  const std::vector<EdgeState>& edges,
                                  const std::vector<int>& bottom_up_order,
                                  const common::DetourerResult& result,
                                  std::string& err) {
    std::vector<common::DetourNodeResult> recomputed;
    if (!compute_profiles(tree, edge_map, edges, bottom_up_order, recomputed, err)) {
        return false;
    }
    if (recomputed.size() != result.node_results.size()) {
        err = "Detour profile size mismatch";
        return false;
    }
    for (std::size_t i = 0; i < recomputed.size(); ++i) {
        if (recomputed[i].valid != result.node_results[i].valid) {
            err = "Detour profile validity mismatch at node " + std::to_string(i);
            return false;
        }
        if (!recomputed[i].valid) {
            continue;
        }
        if (std::abs(recomputed[i].max_sink_delay_to_node -
                     result.node_results[i].max_sink_delay_to_node) > EPS ||
            std::abs(recomputed[i].min_sink_delay_to_node -
                     result.node_results[i].min_sink_delay_to_node) > EPS ||
            std::abs(recomputed[i].skew_to_node -
                     result.node_results[i].skew_to_node) > EPS) {
            err = "Detour profile mismatch at node " + std::to_string(i);
            return false;
        }
    }
    return true;
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_file_enable(bool enable) {
    g_debug_file_enabled = enable;
}

void debug_output(const common::DetourerResult& result,
                  const common::Problem&,
                  const common::TopoTree&,
                  const common::LocerResult&,
                  const common::RouterResult& route_result) {
    if (!g_debug_enabled) {
        return;
    }
    std::cout << "[DETOURER] valid=" << (result.valid ? 1 : 0)
              << " error_msg=" << result.error_msg
              << " num_edges=" << route_result.edge_debugs.size()
              << " num_nodes=" << result.node_results.size()
              << " detour_count=" << result.detour_records.size() << "\n";
    for (const common::DetourNodeResult& node : result.node_results) {
        if (!node.valid) {
            continue;
        }
        std::cout << "[DETOURER] node=" << node.node_id
                  << " sink_delay_count=" << node.sink_delays_to_node.size()
                  << " min_delay=" << node.min_sink_delay_to_node
                  << " max_delay=" << node.max_sink_delay_to_node
                  << " skew=" << node.skew_to_node << "\n";
    }
    for (const common::DetourRecord& record : result.detour_records) {
        std::cout << "[DETOURER] detour edge_id=" << record.edge_id
                  << " parent=" << record.node_parent
                  << " child=" << record.node_child
                  << " segment_index=" << record.segment_index
                  << " anchor_index=" << record.anchor_index
                  << " level=" << record.level
                  << " added_delay=" << record.added_delay
                  << " side=" << record.side
                  << " upgraded=" << (record.upgraded ? 1 : 0) << "\n";
    }
}

bool write_debug_detour_file(const common::DetourerResult& result,
                             const common::RouterResult& route_result,
                             const std::string& input_path,
                             std::string& error_msg) {
    if (!ensure_detour_dir(error_msg)) {
        return false;
    }
    const std::string path = make_debug_detour_path(input_path);
    std::ofstream fout(path);
    if (!fout) {
        error_msg = "Cannot open detour debug file: " + path;
        return false;
    }
    fout << "# DETOURER_DEBUG v1\n";
    fout << "# valid=" << (result.valid ? 1 : 0) << "\n";
    fout << "# num_edges=" << route_result.edge_debugs.size() << "\n";
    fout << "# num_nodes=" << result.node_results.size() << "\n";
    for (const common::RouterEdgeDebug& edge : route_result.edge_debugs) {
        const int final_delay = static_cast<int>(std::llround(edge.wirelength));
        fout << "edge " << edge.edge_id << " "
             << edge.parent << " " << edge.child << " "
             << final_delay << " "
             << final_delay << " "
             << 0 << " ";
        int detour_count = 0;
        for (const common::DetourRecord& record : result.detour_records) {
            if (record.edge_id == edge.edge_id && !record.upgraded) {
                ++detour_count;
            }
        }
        fout << detour_count << " " << edge.polyline.size();
        fout << std::fixed;
        for (const common::SegmentPoint& p : edge.polyline) {
            fout << " " << p.x << " " << p.y;
        }
        fout << "\n";
    }
    for (const common::DetourRecord& record : result.detour_records) {
        fout << "detour " << record.edge_id << " "
             << record.segment_index << " "
             << record.anchor_index << " "
             << record.level << " "
             << record.added_delay << " "
             << record.side << " "
             << (record.upgraded ? 1 : 0) << "\n";
    }
    for (const common::DetourNodeResult& node : result.node_results) {
        if (!node.valid) {
            continue;
        }
        fout << "node " << node.node_id << " "
             << node.sink_delays_to_node.size() << " "
             << node.min_sink_delay_to_node << " "
             << node.max_sink_delay_to_node << " "
             << node.skew_to_node << "\n";
    }
    return true;
}

common::DetourerResult run(const common::Problem& problem,
                           const common::TopoTree& tree,
                           const common::LocerResult& loc_result,
                           common::RouterResult& route_result,
                           const std::string& input_path) {
    common::DetourerResult result;

    std::string err;
    if (!validate_global_inputs(problem, tree, loc_result, route_result, err)) {
        result.error_msg = err;
        return result;
    }

    std::vector<EdgeState> edges;
    std::map<EdgeKey, int> edge_map;
    if (!build_edge_states(problem, tree, loc_result, route_result,
                           edges, edge_map, err)) {
        result.error_msg = err;
        return result;
    }

    if (!validate_all_routes(problem, tree, loc_result, edges, err)) {
        result.error_msg = "Initial router result is not legal for detourer: " + err;
        return result;
    }

    const std::vector<int> bottom_up_order = build_bottom_up_order(tree);
    if (!compute_profiles(tree, edge_map, edges, bottom_up_order,
                          result.node_results, err)) {
        result.error_msg = err;
        return result;
    }

    std::vector<ActiveDetour> active_detours;
    for (int node_id : bottom_up_order) {
        const std::vector<int> children = children_of(tree, node_id);
        if (children.size() < 2U) {
            continue;
        }
        if (children.size() > 2U && g_debug_enabled) {
            std::cerr << "[DETOURER][WARN] node " << node_id
                      << " has " << children.size()
                      << " children; using worst-delay balancing\n";
        }

        bool changed = true;
        int guard = 0;
        while (changed && guard < 256) {
            ++guard;
            changed = false;
            if (!compute_profiles(tree, edge_map, edges, bottom_up_order,
                                  result.node_results, err)) {
                result.error_msg = err;
                return result;
            }
            int short_child = -1;
            double short_delay = 1e30;
            double long_delay = -1e30;
            for (int child : children) {
                double worst = 0.0;
                if (!child_worst_delay(node_id, child, edge_map, edges,
                                       result.node_results, worst)) {
                    continue;
                }
                if (worst < short_delay) {
                    short_delay = worst;
                    short_child = child;
                }
                long_delay = std::max(long_delay, worst);
            }
            if (short_child < 0 || !std::isfinite(short_delay) ||
                !std::isfinite(long_delay)) {
                break;
            }
            const double delta = long_delay - short_delay;
            if (delta <= EPS) {
                break;
            }
            const std::vector<int> candidates =
                candidate_edges_for_side(node_id, short_child, tree, edge_map, edges);
            if (candidates.empty()) {
                break;
            }
            changed = try_detour_growth(problem, tree, loc_result, candidates,
                                        delta, edges, active_detours, result);
        }
    }

    if (!compute_profiles(tree, edge_map, edges, bottom_up_order,
                          result.node_results, err)) {
        result.error_msg = err;
        return result;
    }
    if (!validate_all_routes(problem, tree, loc_result, edges, err)) {
        result.error_msg = "Final detoured route is illegal: " + err;
        return result;
    }
    if (!final_profile_matches(tree, edge_map, edges, bottom_up_order, result, err)) {
        result.error_msg = err;
        return result;
    }

    write_back_route_result(edges, route_result);
    result.valid = true;

    if (g_debug_enabled) {
        debug_output(result, problem, tree, loc_result, route_result);
    }
    if (g_debug_file_enabled) {
        std::string file_error;
        if (!write_debug_detour_file(result, route_result, input_path, file_error)) {
            result.valid = false;
            result.error_msg = file_error;
        }
    }
    return result;
}

}  // namespace detourer
