#include "writer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
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

struct OutputBuffer {
    std::string id;
    int type_index = -1;
    std::string type_name;
    IntPoint loc;
    int tree_node_id = -1;
};

struct OutputRoute {
    std::string sink_id;
    std::vector<IntPoint> points;
};

static bool operator==(const IntPoint& a, const IntPoint& b) {
    return a.x == b.x && a.y == b.y;
}

static bool operator<(const IntPoint& a, const IntPoint& b) {
    if (a.x != b.x) {
        return a.x < b.x;
    }
    return a.y < b.y;
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

static void append_minimal_connector(std::vector<IntPoint>& pts, IntPoint from, IntPoint to) {
    if (from == to) {
        append_point(pts, to);
    } else if (from.x == to.x || from.y == to.y) {
        append_point(pts, to);
    } else {
        append_point(pts, IntPoint{to.x, from.y});
        append_point(pts, to);
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
                                   const common::TopologyTree& tree,
                                   const common::BottomUpResult& bu_result,
                                   const common::TopDownResult& td_result,
                                   std::string& err) {
    if (!problem.valid) {
        err = "Invalid problem: " + problem.error_msg;
        return false;
    }
    if (!tree.valid) {
        err = "Invalid topology tree: " + tree.error_msg;
        return false;
    }
    if (!bu_result.valid) {
        err = "Invalid BU result: " + bu_result.error_msg;
        return false;
    }
    if (!td_result.valid) {
        err = "Invalid TD result: " + td_result.error_msg;
        return false;
    }
    if (tree.nodes.empty()) {
        err = "Topology tree is empty";
        return false;
    }
    if (tree.root < 0 || static_cast<std::size_t>(tree.root) >= tree.nodes.size()) {
        err = "Invalid topology root";
        return false;
    }
    if (td_result.node_results.size() != tree.nodes.size() ||
        bu_result.node_results.size() != tree.nodes.size()) {
        err = "Result node count does not match tree node count";
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
    return true;
}

static bool build_rounded_locs(const common::Problem& problem,
                               const common::TopologyTree& tree,
                               const common::TopDownResult& td_result,
                               std::vector<IntPoint>& rounded_locs,
                               std::string& err) {
    rounded_locs.resize(tree.nodes.size());
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const common::TopDownNodeResult& td_node = td_result.node_results[i];
        if (!td_node.valid) {
            err = "TD node " + std::to_string(i) + " is invalid";
            return false;
        }
        if ((!is_near_integer(td_node.loc.x) || !is_near_integer(td_node.loc.y)) &&
            g_debug_enabled) {
            std::cerr << "[Writer][WARN] non-integer TD loc at node " << i
                      << ", rounded from (" << td_node.loc.x << ","
                      << td_node.loc.y << ") to ("
                      << round_to_int(td_node.loc.x) << ","
                      << round_to_int(td_node.loc.y) << ")\n";
        }
        rounded_locs[i] = round_point(td_node.loc);
        if (!in_die(problem, rounded_locs[i])) {
            err = "Rounded TD loc for node " + std::to_string(i) +
                  " out of die: " + point_to_string(rounded_locs[i]);
            return false;
        }
        if (td_node.min_delay > td_node.max_delay + EPS || td_node.skew < -EPS) {
            err = "TD delay fields invalid at node " + std::to_string(i);
            return false;
        }
    }
    return true;
}

static bool collect_buffers(const common::Problem& problem,
                            const common::TopologyTree& tree,
                            const common::TopDownResult& td_result,
                            const std::vector<IntPoint>& rounded_locs,
                            std::vector<OutputBuffer>& buffers,
                            std::string& err) {
    std::set<IntPoint> occupied;
    occupied.insert(source_point(problem));
    for (const common::Sink& sink : problem.sinks) {
        occupied.insert(sink_point(sink));
    }

    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const common::TopDownNodeResult& td_node = td_result.node_results[i];
        if (!td_node.has_buffer) {
            continue;
        }
        const int type_index = td_node.buffer_type_index;
        if (type_index < 0 ||
            static_cast<std::size_t>(type_index) >= problem.buffer_types.size()) {
            err = "Buffer at node " + std::to_string(i) + " has invalid type";
            return false;
        }
        const common::BufferType& type =
            problem.buffer_types[static_cast<std::size_t>(type_index)];
        if (tree.nodes[i].sink_count > type.max_fanout) {
            err = "Buffer fanout violation at node " + std::to_string(i);
            return false;
        }
        const IntPoint loc = rounded_locs[i];
        if (occupied.count(loc) != 0U) {
            err = "Buffer coordinate conflicts with SOURCE/SINK/buffer at " +
                  point_to_string(loc);
            return false;
        }
        occupied.insert(loc);
        OutputBuffer buffer;
        buffer.id = "B" + std::to_string(buffers.size());
        buffer.type_index = type_index;
        buffer.type_name = type.name;
        buffer.loc = loc;
        buffer.tree_node_id = static_cast<int>(i);
        buffers.push_back(buffer);
    }
    return true;
}

static int find_leaf_for_sink(const common::TopologyTree& tree, int sink_index) {
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        if (tree.nodes[i].is_leaf && tree.nodes[i].sink_index == sink_index) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

static bool build_node_path(const common::TopologyTree& tree,
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
    if (path.empty() || path.front() != tree.root) {
        err = "Leaf path does not reach root";
        return false;
    }
    return true;
}

static bool append_td_branch_polyline(
    const common::Problem& problem,
    std::vector<IntPoint>& pts,
    const std::vector<common::SegmentPoint>& final_route_to_parent,
    IntPoint expected_from,
    IntPoint expected_to,
    int child_id,
    std::string& err) {
    if (final_route_to_parent.empty()) {
        err = "TD result missing/invalid final_route_to_parent for node " +
              std::to_string(child_id);
        return false;
    }
    if (pts.empty() || !(pts.back() == expected_from)) {
        err = "Writer branch append point mismatch before node " +
              std::to_string(child_id);
        return false;
    }

    std::vector<IntPoint> branch;
    branch.reserve(final_route_to_parent.size());
    for (const common::SegmentPoint& p : final_route_to_parent) {
        const IntPoint rounded = round_point(p);
        if (!in_die(problem, rounded)) {
            err = "TD final_route_to_parent point out of die for node " +
                  std::to_string(child_id) + ": " + point_to_string(rounded);
            return false;
        }
        append_point(branch, rounded);
    }
    if (branch.empty() || !(branch.front() == expected_from) ||
        !(branch.back() == expected_to)) {
        err = "TD result missing/invalid final_route_to_parent for node " +
              std::to_string(child_id);
        return false;
    }
    for (std::size_t i = 1; i < branch.size(); ++i) {
        if (branch[i - 1U] == branch[i]) {
            continue;
        }
        if (branch[i - 1U].x != branch[i].x &&
            branch[i - 1U].y != branch[i].y) {
            err = "TD final_route_to_parent has diagonal segment for node " +
                  std::to_string(child_id);
            return false;
        }
    }
    for (std::size_t i = 1; i < branch.size(); ++i) {
        append_point(pts, branch[i]);
    }
    return true;
}

static bool validate_route(const common::Problem& problem,
                           const OutputRoute& route,
                           const IntPoint& expected_sink,
                           std::string& err) {
    if (route.points.size() < 2U) {
        err = "Route " + route.sink_id + " has fewer than two points";
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
    for (const IntPoint& p : route.points) {
        if (!in_die(problem, p)) {
            err = "Route " + route.sink_id + " point out of die: " +
                  point_to_string(p);
            return false;
        }
    }
    for (std::size_t i = 1; i < route.points.size(); ++i) {
        const IntPoint& a = route.points[i - 1U];
        const IntPoint& b = route.points[i];
        if (a == b) {
            err = "Route " + route.sink_id + " contains zero-length segment";
            return false;
        }
        if (a.x != b.x && a.y != b.y) {
            err = "Route " + route.sink_id + " contains diagonal segment";
            return false;
        }
    }
    return true;
}

static bool build_routes(const common::Problem& problem,
                         const common::TopologyTree& tree,
                         const common::TopDownResult& td_result,
                         const std::vector<IntPoint>& rounded_locs,
                         std::vector<OutputRoute>& routes,
                         std::string& err) {
    for (std::size_t sink_index = 0; sink_index < problem.sinks.size(); ++sink_index) {
        const int leaf = find_leaf_for_sink(tree, static_cast<int>(sink_index));
        if (leaf < 0) {
            err = "Cannot find leaf for sink index " + std::to_string(sink_index);
            return false;
        }
        std::vector<int> path;
        if (!build_node_path(tree, leaf, path, err)) {
            return false;
        }

        std::vector<IntPoint> pts;
        const IntPoint src = source_point(problem);
        append_point(pts, src);
        append_minimal_connector(pts, src,
                                 rounded_locs[static_cast<std::size_t>(tree.root)]);

        for (std::size_t i = 1; i < path.size(); ++i) {
            const int parent = path[i - 1U];
            const int child = path[i];
            const common::TopDownNodeResult& child_td =
                td_result.node_results[static_cast<std::size_t>(child)];
            if (!append_td_branch_polyline(
                    problem, pts, child_td.final_route_to_parent,
                    rounded_locs[static_cast<std::size_t>(parent)],
                    rounded_locs[static_cast<std::size_t>(child)],
                    child, err)) {
                return false;
            }
        }

        const IntPoint leaf_loc = rounded_locs[static_cast<std::size_t>(leaf)];
        const IntPoint sink_loc = sink_point(problem.sinks[sink_index]);
        if (!(pts.back() == leaf_loc)) {
            append_minimal_connector(pts, pts.back(), leaf_loc);
        }
        append_minimal_connector(pts, leaf_loc, sink_loc);

        OutputRoute route;
        route.sink_id = problem.sinks[sink_index].id;
        route.points = pts;
        if (!validate_route(problem, route, sink_loc, err)) {
            return false;
        }
        routes.push_back(route);
    }
    return true;
}

static bool write_output_file(const std::string& output_path,
                              const std::vector<OutputBuffer>& buffers,
                              const std::vector<OutputRoute>& routes,
                              std::string& err) {
    std::ofstream fout(output_path);
    if (!fout) {
        err = "Cannot open output file: " + output_path;
        return false;
    }

    fout << "NUM_BUFS " << buffers.size() << "\n";
    for (const OutputBuffer& buffer : buffers) {
        fout << "BUF " << buffer.id << " " << buffer.type_name << " "
             << buffer.loc.x << " " << buffer.loc.y << "\n";
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
                            const common::TopologyTree& tree,
                            const common::BottomUpResult& bu_result,
                            const common::TopDownResult& td_result) {
    WriterResult result;
    result.output_path = make_output_path(input_path);

    std::string err;
    if (!validate_global_inputs(problem, tree, bu_result, td_result, err) ||
        !ensure_result_dir(err)) {
        result.error_msg = err;
        return result;
    }

    std::vector<IntPoint> rounded_locs;
    std::vector<OutputBuffer> buffers;
    std::vector<OutputRoute> routes;
    if (!build_rounded_locs(problem, tree, td_result, rounded_locs, err) ||
        !collect_buffers(problem, tree, td_result, rounded_locs, buffers, err) ||
        !build_routes(problem, tree, td_result, rounded_locs, routes, err) ||
        !write_output_file(result.output_path, buffers, routes, err)) {
        result.error_msg = err;
        return result;
    }

    if (g_debug_enabled) {
        std::cout << "[Writer] output_path=" << result.output_path << "\n";
        std::cout << "[Writer] num_buffers=" << buffers.size() << "\n";
        for (const OutputBuffer& buffer : buffers) {
            std::cout << "[Writer] buffer " << buffer.id
                      << " type=" << buffer.type_name
                      << " node=" << buffer.tree_node_id
                      << " loc=" << point_to_string(buffer.loc)
                      << " fanout="
                      << tree.nodes[static_cast<std::size_t>(buffer.tree_node_id)].sink_count
                      << "\n";
        }
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
