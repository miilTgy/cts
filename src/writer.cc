#include "writer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <system_error>
#include <vector>

namespace writer {
namespace {

bool g_debug_enabled = false;
static constexpr double EPS = 1e-6;

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

struct OutputRoute {
    std::string sink_id;
    std::vector<IntPoint> points;
};

static bool operator==(const IntPoint& a, const IntPoint& b) {
    return a.x == b.x && a.y == b.y;
}

static bool is_near_integer(double v) {
    return std::abs(v - std::round(v)) <= EPS;
}

static int round_to_int(double v) {
    return static_cast<int>(std::llround(v));
}

static IntPoint round_point(common::SegmentPoint p) {
    return IntPoint{round_to_int(p.x), round_to_int(p.y)};
}

static IntPoint source_point(const common::Problem& problem) {
    return IntPoint{problem.source.loc.x, problem.source.loc.y};
}

static IntPoint sink_point(const common::Sink& sink) {
    return IntPoint{sink.loc.x, sink.loc.y};
}

static bool in_die(const common::Problem& problem, const IntPoint& p) {
    return p.x >= 0 && p.x <= problem.die_width &&
           p.y >= 0 && p.y <= problem.die_height;
}

static std::string point_to_string(const IntPoint& p) {
    return "(" + std::to_string(p.x) + "," + std::to_string(p.y) + ")";
}

static void append_point(std::vector<IntPoint>& pts, IntPoint p) {
    if (pts.empty() || !(pts.back() == p)) {
        pts.push_back(p);
    }
}

static int polyline_length(const std::vector<IntPoint>& pts) {
    int len = 0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        len += std::abs(pts[i].x - pts[i - 1].x) +
               std::abs(pts[i].y - pts[i - 1].y);
    }
    return len;
}

static std::string basename_without_ext(const std::string& input_path) {
    std::filesystem::path path(input_path);
    return path.stem().string();
}

static std::string make_output_path(const std::string& input_path) {
    return "result/" + basename_without_ext(input_path) + "_solution.txt";
}

static bool ensure_result_dir(std::string& err) {
    std::error_code ec;
    std::filesystem::create_directories("result", ec);
    if (ec) {
        err = "Cannot create result directory: " + ec.message();
        return false;
    }
    return true;
}

static bool validate_global_inputs(const common::Problem& problem,
                                   const common::TopoTree& tree,
                                   const common::LocerResult& loc_result,
                                   const common::RouterResult& router_result,
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
    if (!router_result.valid) {
        err = "Invalid router result: " + router_result.error_msg;
        return false;
    }
    if (tree.nodes.empty()) {
        err = "Topology tree is empty";
        return false;
    }
    if (tree.root < 0 && tree.source_children.empty()) {
        err = "Topology has neither root nor source_children";
        return false;
    }
    if (tree.root >= 0 &&
        static_cast<std::size_t>(tree.root) >= tree.nodes.size()) {
        err = "Invalid topology root";
        return false;
    }
    if (loc_result.node_results.size() != tree.nodes.size()) {
        err = "Locer node count does not match tree node count";
        return false;
    }
    if (problem.sinks.empty()) {
        err = "Problem has no sinks";
        return false;
    }
    if (!in_die(problem, source_point(problem))) {
        err = "SOURCE out of die";
        return false;
    }
    for (const common::Sink& sink : problem.sinks) {
        if (!in_die(problem, sink_point(sink))) {
            err = "SINK " + sink.id + " out of die";
            return false;
        }
    }
    for (int child : tree.source_children) {
        if (child < 0 || static_cast<std::size_t>(child) >= tree.nodes.size()) {
            err = "Invalid source child node id " + std::to_string(child);
            return false;
        }
    }
    return true;
}

static bool build_rounded_locs(const common::Problem& problem,
                               const common::TopoTree& tree,
                               const common::LocerResult& loc_result,
                               std::vector<IntPoint>& rounded_locs,
                               std::string& err) {
    rounded_locs.resize(tree.nodes.size());
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const common::LocerNodeResult& loc_node = loc_result.node_results[i];
        if (!loc_node.valid) {
            err = "Locer node " + std::to_string(i) + " is invalid";
            return false;
        }
        if (!std::isfinite(loc_node.loc.x) || !std::isfinite(loc_node.loc.y)) {
            err = "Locer node " + std::to_string(i) + " has non-finite loc";
            return false;
        }
        if ((!is_near_integer(loc_node.loc.x) || !is_near_integer(loc_node.loc.y)) &&
            g_debug_enabled) {
            std::cerr << "[Writer][WARN] non-integer locer loc at node " << i
                      << ", rounded from (" << loc_node.loc.x << ","
                      << loc_node.loc.y << ") to ("
                      << round_to_int(loc_node.loc.x) << ","
                      << round_to_int(loc_node.loc.y) << ")\n";
        }
        rounded_locs[i] = round_point(loc_node.loc);
        if (!in_die(problem, rounded_locs[i])) {
            err = "Rounded locer loc for node " + std::to_string(i) +
                  " out of die: " + point_to_string(rounded_locs[i]);
            return false;
        }
    }
    return true;
}

static bool validate_polyline_points(const common::Problem& problem,
                                     const std::vector<IntPoint>& points,
                                     const std::string& label,
                                     std::string& err) {
    if (points.size() < 2U) {
        err = label + " has fewer than two points";
        return false;
    }
    for (const IntPoint& p : points) {
        if (!in_die(problem, p)) {
            err = label + " point out of die: " + point_to_string(p);
            return false;
        }
    }
    for (std::size_t i = 1; i < points.size(); ++i) {
        const IntPoint& a = points[i - 1U];
        const IntPoint& b = points[i];
        if (a == b) {
            err = label + " contains zero-length segment";
            return false;
        }
        if (a.x != b.x && a.y != b.y) {
            err = label + " contains diagonal segment";
            return false;
        }
    }
    return true;
}

static bool round_router_polyline(const common::Problem& problem,
                                  const common::RouterEdgeDebug& edge,
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
    points.reserve(edge.polyline.size());
    for (const common::SegmentPoint& p : edge.polyline) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            err = "Router edge " + std::to_string(edge.edge_id) +
                  " has non-finite polyline point";
            return false;
        }
        if ((!is_near_integer(p.x) || !is_near_integer(p.y)) && g_debug_enabled) {
            std::cerr << "[Writer][WARN] non-integer router point on edge "
                      << edge.edge_id << ", rounded from (" << p.x << ","
                      << p.y << ") to (" << round_to_int(p.x) << ","
                      << round_to_int(p.y) << ")\n";
        }
        append_point(points, round_point(p));
    }
    return validate_polyline_points(
        problem, points, "Router edge " + std::to_string(edge.edge_id), err);
}

static bool build_edge_map(const common::Problem& problem,
                           const common::TopoTree& tree,
                           const std::vector<IntPoint>& rounded_locs,
                           const common::RouterResult& router_result,
                           std::map<EdgeKey, std::vector<IntPoint>>& edge_routes,
                           std::string& err) {
    for (const common::RouterEdgeDebug& edge : router_result.edge_debugs) {
        if (edge.child < 0 ||
            static_cast<std::size_t>(edge.child) >= tree.nodes.size()) {
            err = "Router edge " + std::to_string(edge.edge_id) +
                  " has invalid child " + std::to_string(edge.child);
            return false;
        }
        if (edge.parent < -1 ||
            (edge.parent >= 0 &&
             static_cast<std::size_t>(edge.parent) >= tree.nodes.size())) {
            err = "Router edge " + std::to_string(edge.edge_id) +
                  " has invalid parent " + std::to_string(edge.parent);
            return false;
        }

        std::vector<IntPoint> points;
        if (!round_router_polyline(problem, edge, points, err)) {
            return false;
        }

        const IntPoint expected_from =
            edge.parent < 0 ? source_point(problem)
                            : rounded_locs[static_cast<std::size_t>(edge.parent)];
        const IntPoint expected_to =
            rounded_locs[static_cast<std::size_t>(edge.child)];
        if (!(points.front() == expected_from) || !(points.back() == expected_to)) {
            err = "Router edge " + std::to_string(edge.edge_id) +
                  " endpoint mismatch: expected " + point_to_string(expected_from) +
                  " -> " + point_to_string(expected_to) + ", got " +
                  point_to_string(points.front()) + " -> " +
                  point_to_string(points.back());
            return false;
        }

        const EdgeKey key{edge.parent, edge.child};
        if (edge_routes.count(key) != 0U) {
            err = "Duplicate router edge for parent " + std::to_string(edge.parent) +
                  " child " + std::to_string(edge.child);
            return false;
        }
        edge_routes[key] = points;
    }
    return true;
}

static int find_leaf_for_sink(const common::TopoTree& tree, int sink_index) {
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        if (tree.nodes[i].is_sink && tree.nodes[i].sink_index == sink_index) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

static bool build_node_path_to_source_child(const common::TopoTree& tree,
                                            int leaf,
                                            std::vector<int>& path,
                                            std::string& err) {
    path.clear();
    int cur = leaf;
    while (cur >= 0) {
        if (static_cast<std::size_t>(cur) >= tree.nodes.size()) {
            err = "Tree path contains out-of-range node";
            return false;
        }
        path.push_back(cur);
        cur = tree.nodes[static_cast<std::size_t>(cur)].parent;
    }
    std::reverse(path.begin(), path.end());
    if (path.empty()) {
        err = "Leaf path is empty";
        return false;
    }

    if (tree.source_children.empty()) {
        if (tree.root < 0 || path.front() != tree.root) {
            err = "Leaf path does not reach topology root";
            return false;
        }
        return true;
    }

    if (std::find(tree.source_children.begin(), tree.source_children.end(),
                  path.front()) == tree.source_children.end()) {
        err = "Leaf path does not reach a SOURCE child";
        return false;
    }
    return true;
}

static bool append_edge_route(const std::map<EdgeKey, std::vector<IntPoint>>& edge_routes,
                              int parent,
                              int child,
                              std::vector<IntPoint>& points,
                              std::string& err) {
    const EdgeKey key{parent, child};
    const auto it = edge_routes.find(key);
    if (it == edge_routes.end()) {
        err = "Missing router polyline for parent " + std::to_string(parent) +
              " child " + std::to_string(child);
        return false;
    }
    const std::vector<IntPoint>& edge_points = it->second;
    if (points.empty()) {
        points = edge_points;
        return true;
    }
    if (!(points.back() == edge_points.front())) {
        err = "Route append mismatch for parent " + std::to_string(parent) +
              " child " + std::to_string(child);
        return false;
    }
    for (std::size_t i = 1; i < edge_points.size(); ++i) {
        points.push_back(edge_points[i]);
    }
    return true;
}

static bool validate_route(const common::Problem& problem,
                           const OutputRoute& route,
                           const IntPoint& expected_sink,
                           std::string& err) {
    if (!validate_polyline_points(problem, route.points,
                                  "Route " + route.sink_id, err)) {
        return false;
    }
    if (!(route.points.front() == source_point(problem))) {
        err = "Route " + route.sink_id + " does not start at SOURCE";
        return false;
    }
    if (!(route.points.back() == expected_sink)) {
        err = "Route " + route.sink_id + " does not end at sink";
        return false;
    }
    return true;
}

static bool build_routes(const common::Problem& problem,
                         const common::TopoTree& tree,
                         const std::map<EdgeKey, std::vector<IntPoint>>& edge_routes,
                         std::vector<OutputRoute>& routes,
                         std::string& err) {
    std::set<std::string> seen_sinks;
    for (std::size_t sink_index = 0; sink_index < problem.sinks.size(); ++sink_index) {
        const int leaf = find_leaf_for_sink(tree, static_cast<int>(sink_index));
        if (leaf < 0) {
            err = "Cannot find leaf for sink index " + std::to_string(sink_index);
            return false;
        }

        std::vector<int> path;
        if (!build_node_path_to_source_child(tree, leaf, path, err)) {
            return false;
        }

        std::vector<IntPoint> points;
        if (!append_edge_route(edge_routes, -1, path.front(), points, err)) {
            return false;
        }
        for (std::size_t i = 1; i < path.size(); ++i) {
            if (!append_edge_route(edge_routes, path[i - 1U], path[i], points, err)) {
                return false;
            }
        }

        OutputRoute route;
        route.sink_id = problem.sinks[sink_index].id;
        route.points = points;
        if (seen_sinks.count(route.sink_id) != 0U) {
            err = "Duplicate sink id in problem: " + route.sink_id;
            return false;
        }
        seen_sinks.insert(route.sink_id);

        const IntPoint sink_loc = sink_point(problem.sinks[sink_index]);
        if (!validate_route(problem, route, sink_loc, err)) {
            return false;
        }
        routes.push_back(route);
    }
    return true;
}

static bool write_output_file(const std::string& output_path,
                               const std::vector<OutputRoute>& routes,
                               const common::BuffererResult& bufferer_result,
                               std::string& err) {
    std::ofstream fout(output_path);
    if (!fout) {
        err = "Cannot open output file: " + output_path;
        return false;
    }

    fout << "NUM_BUFS " << bufferer_result.buffers.size() << "\n";
    for (const common::BufferInsertion& buf : bufferer_result.buffers) {
        int bx = static_cast<int>(std::llround(buf.loc.x));
        int by = static_cast<int>(std::llround(buf.loc.y));
        fout << "BUF " << buf.id << " " << buf.type_name << " "
             << bx << " " << by << "\n";
    }
    fout << "NUM_ROUTES " << routes.size() << "\n";
    for (const OutputRoute& route : routes) {
        fout << "ROUTE " << route.sink_id << " " << route.points.size() << "\n";
        for (const IntPoint& p : route.points) {
            fout << p.x << " " << p.y << "\n";
        }
    }
    return true;
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

WriterResult write_solution(const std::string& input_path,
                             const common::Problem& problem,
                             const common::TopoTree& tree,
                             const common::LocerResult& loc_result,
                             const common::RouterResult& router_result,
                             const common::BuffererResult& bufferer_result) {
    WriterResult result;
    result.output_path = make_output_path(input_path);

    std::string err;
    if (!validate_global_inputs(problem, tree, loc_result, router_result, err) ||
        !ensure_result_dir(err)) {
        result.error_msg = err;
        return result;
    }

    std::vector<IntPoint> rounded_locs;
    std::map<EdgeKey, std::vector<IntPoint>> edge_routes;
    std::vector<OutputRoute> routes;
    if (!build_rounded_locs(problem, tree, loc_result, rounded_locs, err) ||
        !build_edge_map(problem, tree, rounded_locs, router_result, edge_routes, err) ||
        !build_routes(problem, tree, edge_routes, routes, err) ||
        !write_output_file(result.output_path, routes, bufferer_result, err)) {
        result.error_msg = err;
        return result;
    }

    if (g_debug_enabled) {
        std::cout << "[Writer] output_path=" << result.output_path << "\n";
        std::cout << "[Writer] num_buffers=" << bufferer_result.buffers.size() << "\n";
        std::cout << "[Writer] num_routes=" << routes.size() << "\n";
        for (const OutputRoute& route : routes) {
            std::cout << "[Writer] route " << route.sink_id
                      << " P=" << route.points.size()
                      << " length=" << polyline_length(route.points) << "\n";
        }
    }

    result.valid = true;
    return result;
}

}  // namespace writer
