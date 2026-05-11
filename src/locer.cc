#include "locer.h"

#include "bu.h"
#include "td.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace locer {
namespace {

static bool g_debug_enabled = false;
static bool g_debug_file_enabled = false;

static constexpr double EPS = 1e-9;
static constexpr double INF = 1e100;
static constexpr double GLOBAL_MIN_GAP = 1.0;
static constexpr double GLOBAL_TOP_ATTACHMENT_WEIGHT = 1000.0;

struct BBoxD {
    double xmin = 0.0;
    double xmax = 0.0;
    double ymin = 0.0;
    double ymax = 0.0;
    bool valid = false;
};

enum class Side {
    Left,
    Right,
    Bottom,
    Top,
    Unknown
};

enum class PrimaryAxis {
    X,
    Y
};

struct PhysicalClusterInfo {
    int cluster_id = -1;
    int cluster_top_id = -1;
    BBoxD cluster_bbox;
    Side preferred_side = Side::Unknown;
    std::vector<int> access_nodes;
    std::vector<int> sink_indices;
};

struct ClusterDebugInfo {
    int cluster_id = -1;
    int cluster_top_id = -1;
    BBoxD cluster_bbox;
    Side preferred_side = Side::Unknown;
    common::SegmentPoint external_anchor;
    std::string anchor_source;
    double anchor_dx = 0.0;
    double anchor_dy = 0.0;
    bool tie_used = false;
    std::vector<Side> tie_candidates;
    std::array<double, 4> side_scores{{0.0, 0.0, 0.0, 0.0}};
    std::vector<int> access_nodes;
    std::vector<common::SegmentPoint> access_locs;
    int dme_access_subtree_count = 0;
};

struct CachedDmeRun {
    int access_id = -1;
    common::ClusterDmeInput input;
    common::BottomUpResult bu_result;
};

struct OuterDebugInfo {
    bool valid = false;
    BBoxD related_bbox;
    Side preferred_side = Side::Unknown;
    Side actual_side = Side::Unknown;
    bool fallback_side_used = false;
    PrimaryAxis primary_axis = PrimaryAxis::X;
    bool ascending_order = true;
    int order_rank = -1;
    int order_count = 0;
    double source_axis = 0.0;
    double order_lower = 0.0;
    double order_upper = 0.0;
    bool order_fallback_used = false;
    bool source_side_ok = true;
    double crossing_penalty = 0.0;
    double nearest_bbox_distance = 0.0;
    std::vector<int> neighbor_ids;
};

struct GlobalOrderContext {
    bool valid = false;
    PrimaryAxis primary_axis = PrimaryAxis::X;
    bool ascending = true;
    double source_axis = 0.0;
    double source_orth = 0.0;
    double candidate_radius = 0.0;
    std::vector<int> ordered_nodes;
    std::map<int, std::size_t> rank_by_node;
};

static std::vector<ClusterDebugInfo> g_last_cluster_debug_info;
static std::vector<OuterDebugInfo> g_last_outer_debug_info;

static common::SegmentPoint to_segment_point(const common::Point& p) {
    return common::SegmentPoint{static_cast<double>(p.x),
                                static_cast<double>(p.y)};
}

static double manhattan(const common::SegmentPoint& a,
                        const common::SegmentPoint& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

static bool finite_point(const common::SegmentPoint& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::abs(p.x) < INF / 2.0 && std::abs(p.y) < INF / 2.0;
}

static bool is_near_integer(double v) {
    return std::abs(v - std::round(v)) <= 1e-6;
}

static bool is_integer_point(const common::SegmentPoint& p) {
    return is_near_integer(p.x) && is_near_integer(p.y);
}

static std::string primary_axis_to_string(PrimaryAxis axis) {
    return axis == PrimaryAxis::X ? "X" : "Y";
}

static double project_axis(const common::SegmentPoint& p, PrimaryAxis axis) {
    return axis == PrimaryAxis::X ? p.x : p.y;
}

static double project_orth(const common::SegmentPoint& p, PrimaryAxis axis) {
    return axis == PrimaryAxis::X ? p.y : p.x;
}

static common::SegmentPoint compose_point(PrimaryAxis axis,
                                          double axis_value,
                                          double orth_value) {
    if (axis == PrimaryAxis::X) {
        return common::SegmentPoint{axis_value, orth_value};
    }
    return common::SegmentPoint{orth_value, axis_value};
}

static BBoxD bbox_from_sinks(const common::Problem& problem,
                             const std::vector<int>& sink_indices) {
    BBoxD b;
    if (sink_indices.empty()) {
        return b;
    }
    b.xmin = b.ymin = INF;
    b.xmax = b.ymax = -INF;
    for (int idx : sink_indices) {
        if (idx < 0 || static_cast<std::size_t>(idx) >= problem.sinks.size()) {
            return BBoxD{};
        }
        const common::Point& p = problem.sinks[static_cast<std::size_t>(idx)].loc;
        b.xmin = std::min(b.xmin, static_cast<double>(p.x));
        b.xmax = std::max(b.xmax, static_cast<double>(p.x));
        b.ymin = std::min(b.ymin, static_cast<double>(p.y));
        b.ymax = std::max(b.ymax, static_cast<double>(p.y));
    }
    b.valid = true;
    return b;
}

static common::SegmentPoint bbox_center(const BBoxD& b) {
    return common::SegmentPoint{(b.xmin + b.xmax) / 2.0,
                                (b.ymin + b.ymax) / 2.0};
}

static bool inside_or_on_bbox(const common::SegmentPoint& p, const BBoxD& b) {
    return b.valid &&
           p.x >= b.xmin - EPS && p.x <= b.xmax + EPS &&
           p.y >= b.ymin - EPS && p.y <= b.ymax + EPS;
}

static bool inside_strict_bbox(const common::SegmentPoint& p, const BBoxD& b) {
    return b.valid &&
           p.x > b.xmin + EPS && p.x < b.xmax - EPS &&
           p.y > b.ymin + EPS && p.y < b.ymax - EPS;
}

static bool outside_bbox(const common::SegmentPoint& p, const BBoxD& b) {
    return b.valid &&
           (p.x < b.xmin - EPS || p.x > b.xmax + EPS ||
            p.y < b.ymin - EPS || p.y > b.ymax + EPS);
}

static BBoxD union_bbox(const BBoxD& a, const BBoxD& b) {
    if (!a.valid) return b;
    if (!b.valid) return a;
    BBoxD out;
    out.xmin = std::min(a.xmin, b.xmin);
    out.xmax = std::max(a.xmax, b.xmax);
    out.ymin = std::min(a.ymin, b.ymin);
    out.ymax = std::max(a.ymax, b.ymax);
    out.valid = true;
    return out;
}

static bool bbox_equal(const BBoxD& a, const BBoxD& b) {
    if (a.valid != b.valid) {
        return false;
    }
    if (!a.valid) {
        return true;
    }
    return std::abs(a.xmin - b.xmin) <= EPS &&
           std::abs(a.xmax - b.xmax) <= EPS &&
           std::abs(a.ymin - b.ymin) <= EPS &&
           std::abs(a.ymax - b.ymax) <= EPS;
}

static bool in_grid(const common::SegmentPoint& p, const common::Problem& problem) {
    return p.x >= -EPS && p.x <= problem.die_width + EPS &&
           p.y >= -EPS && p.y <= problem.die_height + EPS;
}

static common::SegmentPoint clamp_to_grid(common::SegmentPoint p,
                                          const common::Problem& problem) {
    p.x = std::max(0.0, std::min(static_cast<double>(problem.die_width), p.x));
    p.y = std::max(0.0, std::min(static_cast<double>(problem.die_height), p.y));
    return p;
}

static common::SegmentPoint snap_to_integer_grid(common::SegmentPoint p,
                                                 const common::Problem& problem) {
    p.x = std::round(p.x);
    p.y = std::round(p.y);
    return clamp_to_grid(p, problem);
}

static std::string node_class_string(common::NodeKind kind) {
    switch (kind) {
        case common::NodeKind::Sink:
            return "sink";
        case common::NodeKind::ClusterInternal:
            return "internal";
        case common::NodeKind::ClusterAccess:
            return "access";
        case common::NodeKind::ClusterBridge:
            return "bridge";
        case common::NodeKind::ClusterTop:
            return "top";
        case common::NodeKind::Global:
            return "global";
    }
    return "unknown";
}

static std::string side_to_string(Side side) {
    switch (side) {
        case Side::Left:
            return "LEFT";
        case Side::Right:
            return "RIGHT";
        case Side::Bottom:
            return "BOTTOM";
        case Side::Top:
            return "TOP";
        case Side::Unknown:
            return "UNKNOWN";
    }
    return "UNKNOWN";
}

static std::string side_to_root_loc_mode(Side side) {
    return "ACCESS_" + side_to_string(side);
}

static std::vector<Side> all_sides() {
    return {Side::Left, Side::Right, Side::Bottom, Side::Top};
}

static std::vector<Side> deterministic_side_order() {
    return {Side::Top, Side::Right, Side::Bottom, Side::Left};
}

static int side_index(Side side) {
    switch (side) {
        case Side::Left:
            return 0;
        case Side::Right:
            return 1;
        case Side::Bottom:
            return 2;
        case Side::Top:
            return 3;
        case Side::Unknown:
            return -1;
    }
    return -1;
}

static std::vector<Side> other_sides(Side side) {
    std::vector<Side> sides;
    for (Side s : all_sides()) {
        if (s != side) sides.push_back(s);
    }
    return sides;
}

static bool has_treer_node_guidance_loc(const common::TopoTree& tree,
                                        int node_id) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) {
        return false;
    }
    return finite_point(to_segment_point(
        tree.nodes[static_cast<std::size_t>(node_id)].loc));
}

static common::SegmentPoint get_treer_node_guidance_loc(
    const common::TopoTree& tree,
    int node_id) {
    return to_segment_point(tree.nodes[static_cast<std::size_t>(node_id)].loc);
}

static bool point_on_required_side(const common::SegmentPoint& p,
                                   const BBoxD& b,
                                   Side side) {
    if (!b.valid) return false;
    switch (side) {
        case Side::Left:
            return p.x < b.xmin - EPS;
        case Side::Right:
            return p.x > b.xmax + EPS;
        case Side::Bottom:
            return p.y < b.ymin - EPS;
        case Side::Top:
            return p.y > b.ymax + EPS;
        case Side::Unknown:
            return outside_bbox(p, b);
    }
    return false;
}

static Side actual_side_of_point_to_bbox(const common::SegmentPoint& p,
                                         const BBoxD& b) {
    if (!b.valid) return Side::Unknown;
    double best = EPS;
    Side side = Side::Unknown;
    const double left = b.xmin - p.x;
    const double right = p.x - b.xmax;
    const double bottom = b.ymin - p.y;
    const double top = p.y - b.ymax;
    if (left > best) {
        best = left;
        side = Side::Left;
    }
    if (right > best) {
        best = right;
        side = Side::Right;
    }
    if (bottom > best) {
        best = bottom;
        side = Side::Bottom;
    }
    if (top > best) {
        side = Side::Top;
    }
    return side;
}

static double distance_to_bbox_boundary(const common::SegmentPoint& p,
                                        const BBoxD& b) {
    if (!b.valid) return 0.0;
    if (inside_or_on_bbox(p, b)) {
        return 0.0;
    }
    const double dx = p.x < b.xmin ? b.xmin - p.x :
                      (p.x > b.xmax ? p.x - b.xmax : 0.0);
    const double dy = p.y < b.ymin ? b.ymin - p.y :
                      (p.y > b.ymax ? p.y - b.ymax : 0.0);
    return dx + dy;
}

static std::vector<int> collect_access_nodes_under_cluster_top(
    const common::TopoTree& tree,
    int cluster_top_id) {
    std::vector<int> access_nodes;
    if (cluster_top_id < 0 ||
        static_cast<std::size_t>(cluster_top_id) >= tree.nodes.size()) {
        return access_nodes;
    }
    std::vector<int> stack{cluster_top_id};
    std::vector<int> visited(tree.nodes.size(), 0);
    while (!stack.empty()) {
        const int node_id = stack.back();
        stack.pop_back();
        if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) {
            continue;
        }
        const std::size_t idx = static_cast<std::size_t>(node_id);
        if (visited[idx] != 0) {
            continue;
        }
        visited[idx] = 1;
        const common::TopoNode& node = tree.nodes[idx];
        if (node.kind == common::NodeKind::ClusterAccess) {
            access_nodes.push_back(node_id);
        }
        if (node.left >= 0) {
            stack.push_back(node.left);
        }
        if (node.right >= 0) {
            stack.push_back(node.right);
        }
    }
    std::sort(access_nodes.begin(), access_nodes.end());
    access_nodes.erase(std::unique(access_nodes.begin(), access_nodes.end()),
                       access_nodes.end());
    return access_nodes;
}

static std::vector<PhysicalClusterInfo> build_physical_clusters(
    const common::Problem& problem,
    const common::TopoTree& tree,
    std::string& err) {
    std::vector<int> cluster_tops;
    for (const common::TopoNode& node : tree.nodes) {
        if (node.kind == common::NodeKind::ClusterTop) {
            cluster_tops.push_back(node.id);
        }
    }
    std::sort(cluster_tops.begin(), cluster_tops.end());

    std::vector<PhysicalClusterInfo> clusters;
    clusters.reserve(cluster_tops.size());
    for (std::size_t i = 0; i < cluster_tops.size(); ++i) {
        const int top_id = cluster_tops[i];
        const common::TopoNode& top = tree.nodes[static_cast<std::size_t>(top_id)];
        PhysicalClusterInfo cluster;
        cluster.cluster_id = static_cast<int>(i);
        cluster.cluster_top_id = top_id;
        cluster.sink_indices = top.sink_indices;
        cluster.cluster_bbox = bbox_from_sinks(problem, top.sink_indices);
        if (!cluster.cluster_bbox.valid) {
            err = "Failed to compute cluster bbox from ClusterTop sinks for node " +
                  std::to_string(top_id);
            return {};
        }
        cluster.access_nodes = collect_access_nodes_under_cluster_top(tree, top_id);
        clusters.push_back(cluster);
    }
    return clusters;
}

static std::vector<int> build_sink_to_cluster(
    const common::Problem& problem,
    const std::vector<PhysicalClusterInfo>& clusters) {
    std::vector<int> sink_to_cluster(problem.sinks.size(), -1);
    for (const PhysicalClusterInfo& cluster : clusters) {
        for (int sink_idx : cluster.sink_indices) {
            if (sink_idx >= 0 &&
                static_cast<std::size_t>(sink_idx) < sink_to_cluster.size()) {
                sink_to_cluster[static_cast<std::size_t>(sink_idx)] =
                    cluster.cluster_id;
            }
        }
    }
    return sink_to_cluster;
}

static std::vector<int> cluster_ids_for_sinks(
    const std::vector<int>& sink_indices,
    const std::vector<int>& sink_to_cluster) {
    std::vector<int> ids;
    for (int sink_idx : sink_indices) {
        if (sink_idx < 0 ||
            static_cast<std::size_t>(sink_idx) >= sink_to_cluster.size()) {
            continue;
        }
        const int cluster_id = sink_to_cluster[static_cast<std::size_t>(sink_idx)];
        if (cluster_id >= 0) ids.push_back(cluster_id);
    }
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
}

static BBoxD bbox_for_cluster_ids(const std::vector<int>& cluster_ids,
                                  const std::vector<PhysicalClusterInfo>& clusters) {
    BBoxD out;
    for (int cluster_id : cluster_ids) {
        if (cluster_id < 0 ||
            static_cast<std::size_t>(cluster_id) >= clusters.size()) {
            continue;
        }
        out = union_bbox(out,
                         clusters[static_cast<std::size_t>(cluster_id)].cluster_bbox);
    }
    return out;
}

static Side shared_preferred_side_for_clusters(
    const std::vector<int>& cluster_ids,
    const std::vector<PhysicalClusterInfo>& clusters) {
    Side shared = Side::Unknown;
    for (int cluster_id : cluster_ids) {
        if (cluster_id < 0 ||
            static_cast<std::size_t>(cluster_id) >= clusters.size()) {
            continue;
        }
        const Side side = clusters[static_cast<std::size_t>(cluster_id)].preferred_side;
        if (side == Side::Unknown) return Side::Unknown;
        if (shared == Side::Unknown) {
            shared = side;
        } else if (shared != side) {
            return Side::Unknown;
        }
    }
    return shared;
}

static common::SegmentPoint choose_access_loc(const common::MergingSegment& ms,
                                              Side side);

static bool segment_crosses_bbox(const common::SegmentPoint& a,
                                 const common::SegmentPoint& b,
                                 const BBoxD& box);

static common::SegmentPoint side_escape_point(const BBoxD& bbox, Side side) {
    const common::SegmentPoint c = bbox_center(bbox);
    switch (side) {
        case Side::Left:
            return common::SegmentPoint{bbox.xmin - 1.0, c.y};
        case Side::Right:
            return common::SegmentPoint{bbox.xmax + 1.0, c.y};
        case Side::Bottom:
            return common::SegmentPoint{c.x, bbox.ymin - 1.0};
        case Side::Top:
            return common::SegmentPoint{c.x, bbox.ymax + 1.0};
        case Side::Unknown:
            return c;
    }
    return c;
}

static common::SegmentPoint die_center(const common::Problem& problem) {
    return common::SegmentPoint{static_cast<double>(problem.die_width) / 2.0,
                                static_cast<double>(problem.die_height) / 2.0};
}

static double max_die_span(const common::Problem& problem) {
    return static_cast<double>(std::max(problem.die_width, problem.die_height));
}

static common::SegmentPoint resolve_external_anchor_for_cluster_top(
    const common::Problem& problem,
    const common::TopoTree& tree,
    int cluster_top_id,
    std::string& anchor_source) {
    if (cluster_top_id >= 0 &&
        static_cast<std::size_t>(cluster_top_id) < tree.nodes.size()) {
        const common::TopoNode& top =
            tree.nodes[static_cast<std::size_t>(cluster_top_id)];
        if (has_treer_node_guidance_loc(tree, top.parent)) {
            anchor_source = "parent_treer_loc";
            return get_treer_node_guidance_loc(tree, top.parent);
        }

        int ancestor = top.parent;
        while (ancestor >= 0 &&
               static_cast<std::size_t>(ancestor) < tree.nodes.size()) {
            const common::TopoNode& node =
                tree.nodes[static_cast<std::size_t>(ancestor)];
            if ((node.kind == common::NodeKind::Global ||
                 node.kind == common::NodeKind::ClusterTop) &&
                has_treer_node_guidance_loc(tree, ancestor)) {
                anchor_source = "ancestor_treer_loc";
                return get_treer_node_guidance_loc(tree, ancestor);
            }
            ancestor = node.parent;
        }
    }

    const common::SegmentPoint source = to_segment_point(problem.source.loc);
    if (finite_point(source) && in_grid(source, problem)) {
        anchor_source = "source";
        return source;
    }

    anchor_source = "die_center";
    return die_center(problem);
}

static GlobalOrderContext build_global_order_context(const common::Problem& problem,
                                                     const common::TopoTree& tree) {
    GlobalOrderContext ctx;
    std::vector<int> global_nodes;
    global_nodes.reserve(tree.nodes.size());
    for (const common::TopoNode& node : tree.nodes) {
        if (node.kind == common::NodeKind::Global) {
            global_nodes.push_back(node.id);
        }
    }
    if (global_nodes.empty()) {
        return ctx;
    }

    std::vector<common::SegmentPoint> guidance_points;
    guidance_points.reserve(global_nodes.size() + 1);
    for (int node_id : global_nodes) {
        if (has_treer_node_guidance_loc(tree, node_id)) {
            guidance_points.push_back(get_treer_node_guidance_loc(tree, node_id));
        }
    }
    const common::SegmentPoint source = to_segment_point(problem.source.loc);
    if (finite_point(source)) {
        guidance_points.push_back(source);
    }
    if (guidance_points.empty()) {
        guidance_points.push_back(die_center(problem));
    }

    double xmin = INF, xmax = -INF, ymin = INF, ymax = -INF;
    for (const common::SegmentPoint& p : guidance_points) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    const double span_x = xmax - xmin;
    const double span_y = ymax - ymin;
    ctx.primary_axis = span_x >= span_y ? PrimaryAxis::X : PrimaryAxis::Y;
    ctx.source_axis = project_axis(source, ctx.primary_axis);
    ctx.source_orth = project_orth(source, ctx.primary_axis);
    ctx.candidate_radius = std::max(8.0, 0.10 * max_die_span(problem));

    double root_axis = ctx.source_axis;
    if (tree.root >= 0 && static_cast<std::size_t>(tree.root) < tree.nodes.size() &&
        tree.nodes[static_cast<std::size_t>(tree.root)].kind == common::NodeKind::Global &&
        has_treer_node_guidance_loc(tree, tree.root)) {
        root_axis = project_axis(get_treer_node_guidance_loc(tree, tree.root),
                                 ctx.primary_axis);
    } else if (!global_nodes.empty() && has_treer_node_guidance_loc(tree, global_nodes.front())) {
        root_axis = project_axis(get_treer_node_guidance_loc(tree, global_nodes.front()),
                                 ctx.primary_axis);
    }
    ctx.ascending = root_axis >= ctx.source_axis;

    auto node_key = [&](int node_id) {
        const common::SegmentPoint p = has_treer_node_guidance_loc(tree, node_id)
                                           ? get_treer_node_guidance_loc(tree, node_id)
                                           : die_center(problem);
        return std::pair<double, double>{project_axis(p, ctx.primary_axis),
                                         project_orth(p, ctx.primary_axis)};
    };
    std::sort(global_nodes.begin(), global_nodes.end(), [&](int a, int b) {
        const auto ka = node_key(a);
        const auto kb = node_key(b);
        if (std::abs(ka.first - kb.first) > EPS) {
            return ctx.ascending ? ka.first < kb.first : ka.first > kb.first;
        }
        if (std::abs(ka.second - kb.second) > EPS) {
            return ka.second < kb.second;
        }
        return a < b;
    });

    ctx.ordered_nodes = global_nodes;
    for (std::size_t i = 0; i < ctx.ordered_nodes.size(); ++i) {
        const int node_id = ctx.ordered_nodes[i];
        ctx.rank_by_node[node_id] = i;
    }
    ctx.valid = true;
    return ctx;
}

static common::SegmentPoint global_order_reference_loc(const common::Problem& problem,
                                                       const common::TopoTree& tree,
                                                       int node_id) {
    if (has_treer_node_guidance_loc(tree, node_id)) {
        return get_treer_node_guidance_loc(tree, node_id);
    }
    return die_center(problem);
}

static bool global_order_side_ok(const GlobalOrderContext& ctx,
                                 double axis_value) {
    if (!ctx.valid) return true;
    if (ctx.ascending) {
        return axis_value >= ctx.source_axis - EPS;
    }
    return axis_value <= ctx.source_axis + EPS;
}

static bool global_order_bounds_for_node(const common::Problem& problem,
                                         const common::TopoTree& tree,
                                         const GlobalOrderContext& ctx,
                                         int node_id,
                                         double& lower,
                                         double& upper) {
    if (!ctx.valid) {
        return false;
    }
    auto it = ctx.rank_by_node.find(node_id);
    if (it == ctx.rank_by_node.end()) {
        return false;
    }
    const std::size_t rank = it->second;
    const std::size_t count = ctx.ordered_nodes.size();
    const double gap = GLOBAL_MIN_GAP;
    const double current_axis =
        project_axis(global_order_reference_loc(problem, tree, node_id),
                     ctx.primary_axis);
    lower = -INF;
    upper = INF;
    if (ctx.ascending) {
        lower = (rank == 0) ? ctx.source_axis + gap
                            : project_axis(
                                  global_order_reference_loc(problem, tree,
                                                             ctx.ordered_nodes[rank - 1]),
                                  ctx.primary_axis) +
                                  gap;
        upper = (rank + 1 < count)
                    ? project_axis(
                          global_order_reference_loc(problem, tree,
                                                     ctx.ordered_nodes[rank + 1]),
                          ctx.primary_axis) -
                          gap
                    : current_axis + ctx.candidate_radius;
    } else {
        upper = (rank == 0) ? ctx.source_axis - gap
                            : project_axis(
                                  global_order_reference_loc(problem, tree,
                                                             ctx.ordered_nodes[rank - 1]),
                                  ctx.primary_axis) -
                                  gap;
        lower = (rank + 1 < count)
                    ? project_axis(
                          global_order_reference_loc(problem, tree,
                                                     ctx.ordered_nodes[rank + 1]),
                          ctx.primary_axis) +
                          gap
                    : current_axis - ctx.candidate_radius;
    }
    if (lower > upper) {
        const double mid = (lower + upper) / 2.0;
        lower = mid;
        upper = mid;
    }
    return true;
}

static std::vector<common::SegmentPoint> generate_global_order_candidates(
    const common::Problem& problem,
    const common::TopoTree& tree,
    const GlobalOrderContext& ctx,
    const BBoxD& related_bbox,
    int node_id,
    const std::vector<common::SegmentPoint>& neighbor_locs,
    const common::SegmentPoint& parent_est,
    bool fallback_radius) {
    std::vector<common::SegmentPoint> candidates;
    if (!ctx.valid) {
        return candidates;
    }
    double lower = 0.0;
    double upper = 0.0;
    if (!global_order_bounds_for_node(problem, tree, ctx, node_id, lower, upper)) {
        return candidates;
    }

    const common::SegmentPoint ref = global_order_reference_loc(problem, tree, node_id);
    const double ref_axis = project_axis(ref, ctx.primary_axis);
    const double ref_orth = project_orth(ref, ctx.primary_axis);
    const double radius = fallback_radius ? ctx.candidate_radius * 2.0
                                          : ctx.candidate_radius;
    const double axis_radius = std::max(1.0, radius);
    const double source_axis = ctx.source_axis;
    const double source_orth = ctx.source_orth;
    const common::SegmentPoint bbox_c = related_bbox.valid ? bbox_center(related_bbox)
                                                           : die_center(problem);

    auto in_axis_interval = [&](double axis_value) {
        return axis_value >= lower - EPS && axis_value <= upper + EPS &&
               global_order_side_ok(ctx, axis_value);
    };

    auto add_candidate = [&](double axis_value, double orth_value) {
        if (!std::isfinite(axis_value) || !std::isfinite(orth_value)) return;
        common::SegmentPoint p = compose_point(ctx.primary_axis, axis_value, orth_value);
        if (!finite_point(p) || !in_grid(p, problem)) return;
        if (!in_axis_interval(axis_value)) return;
        for (const common::SegmentPoint& old : candidates) {
            if (std::abs(old.x - p.x) <= EPS && std::abs(old.y - p.y) <= EPS) {
                return;
            }
        }
        candidates.push_back(p);
    };

    std::vector<double> axis_values{
        lower,
        upper,
        ref_axis,
        (lower + upper) / 2.0,
        ctx.ascending ? std::min(upper, source_axis + 1.0) : std::max(lower, source_axis - 1.0),
        ctx.ascending ? std::min(upper, ref_axis + axis_radius * 0.25)
                      : std::max(lower, ref_axis - axis_radius * 0.25),
        ctx.ascending ? std::max(lower, ref_axis - axis_radius * 0.25)
                      : std::min(upper, ref_axis + axis_radius * 0.25),
        project_axis(parent_est, ctx.primary_axis),
        project_axis(bbox_c, ctx.primary_axis),
    };
    if (related_bbox.valid) {
        if (ctx.primary_axis == PrimaryAxis::X) {
            axis_values.push_back(related_bbox.xmin);
            axis_values.push_back(related_bbox.xmax);
        } else {
            axis_values.push_back(related_bbox.ymin);
            axis_values.push_back(related_bbox.ymax);
        }
    }

    std::vector<double> orth_values{
        source_orth,
        ref_orth,
        project_orth(parent_est, ctx.primary_axis),
        project_orth(die_center(problem), ctx.primary_axis),
        project_orth(bbox_c, ctx.primary_axis),
    };
    for (const common::SegmentPoint& n : neighbor_locs) {
        orth_values.push_back(project_orth(n, ctx.primary_axis));
    }

    if (!neighbor_locs.empty()) {
        double sum = 0.0;
        for (const common::SegmentPoint& n : neighbor_locs) {
            sum += project_orth(n, ctx.primary_axis);
        }
        orth_values.push_back(sum / static_cast<double>(neighbor_locs.size()));
    }

    std::sort(axis_values.begin(), axis_values.end());
    axis_values.erase(std::unique(axis_values.begin(), axis_values.end(),
                                  [](double a, double b) {
                                      return std::abs(a - b) <= EPS;
                                  }),
                      axis_values.end());
    std::sort(orth_values.begin(), orth_values.end());
    orth_values.erase(std::unique(orth_values.begin(), orth_values.end(),
                                  [](double a, double b) {
                                      return std::abs(a - b) <= EPS;
                                  }),
                      orth_values.end());

    for (double axis_value : axis_values) {
        if (!in_axis_interval(axis_value)) continue;
        for (double orth_value : orth_values) {
            add_candidate(axis_value, orth_value);
        }
    }

    return candidates;
}

static double global_order_violation_penalty(const common::Problem& problem,
                                             const common::TopoTree& tree,
                                             const GlobalOrderContext& ctx,
                                             int node_id,
                                             const common::SegmentPoint& p,
                                             double& lower,
                                             double& upper) {
    if (!global_order_bounds_for_node(problem, tree, ctx, node_id, lower, upper)) {
        return INF;
    }
    const double axis_value = project_axis(p, ctx.primary_axis);
    if (axis_value < lower - EPS || axis_value > upper + EPS) {
        return INF;
    }
    if (!global_order_side_ok(ctx, axis_value)) {
        return INF;
    }
    return 0.0;
}

static double source_trunk_penalty(const GlobalOrderContext& ctx,
                                   const common::SegmentPoint& p) {
    return std::abs(project_orth(p, ctx.primary_axis) - ctx.source_orth);
}

static int find_attached_top_for_global(const common::TopoTree& tree, int node_id) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) {
        return -1;
    }
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    if (node.left >= 0 &&
        static_cast<std::size_t>(node.left) < tree.nodes.size() &&
        tree.nodes[static_cast<std::size_t>(node.left)].kind == common::NodeKind::ClusterTop) {
        return node.left;
    }
    if (node.right >= 0 &&
        static_cast<std::size_t>(node.right) < tree.nodes.size() &&
        tree.nodes[static_cast<std::size_t>(node.right)].kind == common::NodeKind::ClusterTop) {
        return node.right;
    }
    return -1;
}

static double global_top_attachment_penalty(const common::SegmentPoint& p,
                                            const common::SegmentPoint& top_loc) {
    return manhattan(p, top_loc);
}

static double stage0_tie_score_for_side(
    const common::Problem& problem,
    const BBoxD& cluster_bbox,
    const std::vector<BBoxD>& cluster_bboxes,
    const common::SegmentPoint& anchor,
    Side side) {
    const common::SegmentPoint escape = side_escape_point(cluster_bbox, side);
    double score = 0.0;
    if (!in_grid(escape, problem)) {
        score += 100.0;
    }
    for (const BBoxD& peer : cluster_bboxes) {
        if (!peer.valid || bbox_equal(peer, cluster_bbox)) {
            continue;
        }
        score += 1.0 / (1.0 + manhattan(escape, bbox_center(peer)));
        if (segment_crosses_bbox(escape, anchor, peer)) {
            score += 10.0;
        }
    }
    return score;
}

static Side choose_lower_stage0_tie_score(
    Side a,
    Side b,
    const std::array<double, 4>& side_scores) {
    const double score_a = side_scores[static_cast<std::size_t>(side_index(a))];
    const double score_b = side_scores[static_cast<std::size_t>(side_index(b))];
    if (std::abs(score_a - score_b) > EPS) {
        return score_a < score_b ? a : b;
    }
    for (Side side : deterministic_side_order()) {
        if (side == a || side == b) {
            return side;
        }
    }
    return a;
}

static Side choose_preferred_side_from_anchor(
    const common::Problem& problem,
    const PhysicalClusterInfo& cluster,
    const std::vector<BBoxD>& cluster_bboxes,
    const common::SegmentPoint& anchor,
    std::array<double, 4>& side_scores,
    double& dx,
    double& dy,
    bool& tie_used,
    std::vector<Side>& tie_candidates) {
    const common::SegmentPoint c = bbox_center(cluster.cluster_bbox);
    dx = anchor.x - c.x;
    dy = anchor.y - c.y;

    side_scores = {{INF, INF, INF, INF}};
    for (Side side : all_sides()) {
        side_scores[static_cast<std::size_t>(side_index(side))] =
            stage0_tie_score_for_side(problem, cluster.cluster_bbox,
                                      cluster_bboxes, anchor, side);
    }

    const Side x_side = dx >= 0.0 ? Side::Right : Side::Left;
    const Side y_side = dy >= 0.0 ? Side::Top : Side::Bottom;
    const double width = cluster.cluster_bbox.xmax - cluster.cluster_bbox.xmin;
    const double height = cluster.cluster_bbox.ymax - cluster.cluster_bbox.ymin;
    const double tie_eps = std::max(1.0, 0.05 * std::max(width, height));
    tie_used = std::abs(std::abs(dx) - std::abs(dy)) <= tie_eps;

    if (tie_used) {
        tie_candidates = {x_side, y_side};
        if (x_side == y_side) {
            return x_side;
        }
        return choose_lower_stage0_tie_score(x_side, y_side, side_scores);
    }

    tie_candidates.clear();
    if (std::abs(dx) >= std::abs(dy)) {
        return x_side;
    }
    return y_side;
}

static common::SegmentPoint choose_access_loc(const common::MergingSegment& ms,
                                              Side side) {
    std::vector<common::SegmentPoint> candidates{ms.p1, ms.p2,
        common::SegmentPoint{(ms.p1.x + ms.p2.x) / 2.0,
                             (ms.p1.y + ms.p2.y) / 2.0}};
    auto best_it = candidates.begin();
    for (auto it = candidates.begin(); it != candidates.end(); ++it) {
        switch (side) {
            case Side::Left:
                if (it->x < best_it->x) best_it = it;
                break;
            case Side::Right:
                if (it->x > best_it->x) best_it = it;
                break;
            case Side::Bottom:
                if (it->y < best_it->y) best_it = it;
                break;
            case Side::Top:
                if (it->y > best_it->y) best_it = it;
                break;
            case Side::Unknown:
                best_it = candidates.begin() + 2;
                break;
        }
    }
    return *best_it;
}

static bool is_dme_topology_kind(common::NodeKind kind) {
    return kind == common::NodeKind::Sink ||
           kind == common::NodeKind::ClusterInternal ||
           kind == common::NodeKind::ClusterAccess;
}

static common::DmeNodeClass to_dme_class(const common::TopoNode& node) {
    if (node.kind == common::NodeKind::Sink) return common::DmeNodeClass::Sink;
    if (node.kind == common::NodeKind::ClusterAccess) return common::DmeNodeClass::Access;
    return common::DmeNodeClass::Internal;
}

static bool build_cluster_dfs(const common::TopoTree& tree,
                              int origin_id,
                              int parent_local,
                              common::ClusterDmeInput& input,
                              std::map<int, int>& origin_to_local,
                              std::string& err) {
    if (origin_id < 0 || static_cast<std::size_t>(origin_id) >= tree.nodes.size()) {
        err = "Cluster DME subtree references invalid topology node";
        return false;
    }
    const common::TopoNode& topo = tree.nodes[static_cast<std::size_t>(origin_id)];
    if (!is_dme_topology_kind(topo.kind)) {
        err = "Cluster DME subtree includes outer node " + std::to_string(origin_id);
        return false;
    }
    if (origin_to_local.count(origin_id) != 0U) {
        err = "Cluster DME subtree reuses topology node " + std::to_string(origin_id);
        return false;
    }

    const int local_id = static_cast<int>(input.nodes.size());
    origin_to_local[origin_id] = local_id;
    common::ClusterDmeNode node;
    node.local_id = local_id;
    node.origin_node_id = origin_id;
    node.node_class = to_dme_class(topo);
    node.parent = parent_local;
    node.sink_index = topo.sink_index;
    node.sink_count = static_cast<int>(topo.sink_indices.size());
    input.nodes.push_back(node);

    if (node.node_class != common::DmeNodeClass::Sink) {
        if (topo.left < 0 || topo.right < 0) {
            err = "Cluster DME internal/access node has missing children";
            return false;
        }
        if (!build_cluster_dfs(tree, topo.left, local_id, input, origin_to_local, err) ||
            !build_cluster_dfs(tree, topo.right, local_id, input, origin_to_local, err)) {
            return false;
        }
        input.nodes[static_cast<std::size_t>(local_id)].left =
            origin_to_local[topo.left];
        input.nodes[static_cast<std::size_t>(local_id)].right =
            origin_to_local[topo.right];
    }
    return true;
}

static common::ClusterDmeInput build_cluster_input(const common::TopoTree& tree,
                                                   int access_id,
                                                   int cluster_id) {
    common::ClusterDmeInput input;
    input.cluster_id = cluster_id;
    input.root_local_id = 0;
    input.root_origin_node_id = access_id;
    std::map<int, int> origin_to_local;
    std::string err;
    if (!build_cluster_dfs(tree, access_id, -1, input, origin_to_local, err)) {
        input.valid = false;
        input.error_msg = err;
        return input;
    }
    input.valid = true;
    return input;
}

static void stats_from_delays(common::LocerNodeResult& node) {
    if (node.sink_delays_to_node.empty()) {
        node.min_sink_delay_to_node = 0.0;
        node.max_sink_delay_to_node = 0.0;
        node.skew_to_node = 0.0;
        return;
    }
    const auto minmax = std::minmax_element(node.sink_delays_to_node.begin(),
                                            node.sink_delays_to_node.end());
    node.min_sink_delay_to_node = *minmax.first;
    node.max_sink_delay_to_node = *minmax.second;
    node.skew_to_node = node.max_sink_delay_to_node - node.min_sink_delay_to_node;
}

static void update_profile_from_children(common::LocerResult& result,
                                         const common::TopoTree& tree,
                                         int node_id,
                                         bool prefer_bu_edges,
                                         const common::ClusterDmeInput* input,
                                         const common::BottomUpResult* bu_result) {
    common::LocerNodeResult& out = result.node_results[static_cast<std::size_t>(node_id)];
    const common::TopoNode& topo = tree.nodes[static_cast<std::size_t>(node_id)];
    out.sink_delays_to_node.clear();
    if (topo.kind == common::NodeKind::Sink) {
        out.sink_delays_to_node.push_back(0.0);
        stats_from_delays(out);
        return;
    }

    auto append_child = [&](int child_id, double edge) {
        if (child_id < 0) return;
        const common::LocerNodeResult& child =
            result.node_results[static_cast<std::size_t>(child_id)];
        for (double d : child.sink_delays_to_node) {
            out.sink_delays_to_node.push_back(d + edge);
        }
    };

    if (prefer_bu_edges && input != nullptr && bu_result != nullptr) {
        std::map<int, int> origin_to_local;
        for (const common::ClusterDmeNode& node : input->nodes) {
            origin_to_local[node.origin_node_id] = node.local_id;
        }
        const int local = origin_to_local[node_id];
        const common::BottomUpNodeResult& bu_node =
            bu_result->node_results[static_cast<std::size_t>(local)];
        append_child(topo.left, bu_node.edge_to_left);
        append_child(topo.right, bu_node.edge_to_right);
    } else {
        if (topo.left >= 0) {
            append_child(topo.left,
                         manhattan(out.loc, result.node_results[static_cast<std::size_t>(topo.left)].loc));
        }
        if (topo.right >= 0) {
            append_child(topo.right,
                         manhattan(out.loc, result.node_results[static_cast<std::size_t>(topo.right)].loc));
        }
    }
    stats_from_delays(out);
}

static void write_dme_results(common::LocerResult& loc_result,
                              const common::Problem& problem,
                              const common::TopoTree& tree,
                              const common::ClusterDmeInput& input,
                              const common::BottomUpResult& bu_result,
                              const common::TopDownResult& td_result) {
    for (const common::ClusterDmeNode& local_node : input.nodes) {
        const int node_id = local_node.origin_node_id;
        const common::TopoNode& topo = tree.nodes[static_cast<std::size_t>(node_id)];
        const common::TopDownNodeResult& td_node =
            td_result.node_results[static_cast<std::size_t>(local_node.local_id)];
        common::LocerNodeResult& out =
            loc_result.node_results[static_cast<std::size_t>(node_id)];
        out.node_id = node_id;
        out.valid = true;
        out.loc = snap_to_integer_grid(td_node.loc, problem);
        out.node_class = node_class_string(topo.kind);
        out.cluster_id = input.cluster_id;
        if (local_node.node_class == common::DmeNodeClass::Access) {
            out.loc_mode = "DME_ACCESS_ROOT";
        } else if (local_node.node_class == common::DmeNodeClass::Sink) {
            out.loc_mode = "DME_SINK";
        } else {
            out.loc_mode = "DME_INTERNAL";
        }
        out.loc_score = td_node.loc_score;
        out.candidate_count = td_node.candidate_count;
        out.wire_est_to_parent = td_node.geometric_distance_to_parent;
        if (local_node.node_class == common::DmeNodeClass::Sink &&
            local_node.sink_index >= 0 &&
            static_cast<std::size_t>(local_node.sink_index) < problem.sinks.size()) {
            out.loc = to_segment_point(problem.sinks[static_cast<std::size_t>(local_node.sink_index)].loc);
        }
    }

    for (auto it = input.nodes.rbegin(); it != input.nodes.rend(); ++it) {
        update_profile_from_children(loc_result, tree, it->origin_node_id,
                                     true, &input, &bu_result);
    }
}

static void add_candidate(std::vector<common::SegmentPoint>& candidates,
                          common::SegmentPoint p,
                          const common::Problem& problem) {
    if (!finite_point(p) || !in_grid(p, problem)) {
        return;
    }
    for (const common::SegmentPoint& old : candidates) {
        if (std::abs(old.x - p.x) <= EPS && std::abs(old.y - p.y) <= EPS) {
            return;
        }
    }
    candidates.push_back(p);
}

static std::vector<common::SegmentPoint> generate_bbox_side_candidates(
    const BBoxD& bbox,
    const std::vector<common::SegmentPoint>& neighbor_locs,
    const common::SegmentPoint& parent_est,
    const common::Problem& problem,
    const std::vector<Side>& sides,
    bool include_inside_offsets,
    bool include_bbox_center_anchor) {
    std::vector<common::SegmentPoint> candidates;
    std::vector<double> xs{parent_est.x};
    std::vector<double> ys{parent_est.y};
    if (include_bbox_center_anchor) {
        xs.push_back(bbox_center(bbox).x);
        ys.push_back(bbox_center(bbox).y);
    }
    for (const common::SegmentPoint& p : neighbor_locs) {
        xs.push_back(p.x);
        ys.push_back(p.y);
    }
    const double margins[] = {1.0, 2.0, 4.0, 8.0};
    for (double m : margins) {
        for (Side side : sides) {
            if (side == Side::Left) {
                for (double y : ys) {
                    add_candidate(candidates,
                                  common::SegmentPoint{bbox.xmin - m, y},
                                  problem);
                    if (include_inside_offsets) {
                        add_candidate(candidates,
                                      common::SegmentPoint{bbox.xmin + m, y},
                                      problem);
                    }
                }
            } else if (side == Side::Right) {
                for (double y : ys) {
                    add_candidate(candidates,
                                  common::SegmentPoint{bbox.xmax + m, y},
                                  problem);
                    if (include_inside_offsets) {
                        add_candidate(candidates,
                                      common::SegmentPoint{bbox.xmax - m, y},
                                      problem);
                    }
                }
            } else if (side == Side::Bottom) {
                for (double x : xs) {
                    add_candidate(candidates,
                                  common::SegmentPoint{x, bbox.ymin - m},
                                  problem);
                    if (include_inside_offsets) {
                        add_candidate(candidates,
                                      common::SegmentPoint{x, bbox.ymin + m},
                                      problem);
                    }
                }
            } else if (side == Side::Top) {
                for (double x : xs) {
                    add_candidate(candidates,
                                  common::SegmentPoint{x, bbox.ymax + m},
                                  problem);
                    if (include_inside_offsets) {
                        add_candidate(candidates,
                                      common::SegmentPoint{x, bbox.ymax - m},
                                      problem);
                    }
                }
            }
        }
    }
    return candidates;
}

static bool segment_crosses_bbox(const common::SegmentPoint& a,
                                 const common::SegmentPoint& b,
                                 const BBoxD& box) {
    if (!box.valid) return false;
    if (inside_strict_bbox(a, box) || inside_strict_bbox(b, box)) return true;
    if (std::abs(a.x - b.x) <= EPS) {
        return a.x > box.xmin + EPS && a.x < box.xmax - EPS &&
               std::max(std::min(a.y, b.y), box.ymin) <
                   std::min(std::max(a.y, b.y), box.ymax);
    }
    if (std::abs(a.y - b.y) <= EPS) {
        return a.y > box.ymin + EPS && a.y < box.ymax - EPS &&
               std::max(std::min(a.x, b.x), box.xmin) <
                   std::min(std::max(a.x, b.x), box.xmax);
    }
    const common::SegmentPoint bend1{b.x, a.y};
    const common::SegmentPoint bend2{a.x, b.y};
    return segment_crosses_bbox(a, bend1, box) ||
           segment_crosses_bbox(bend1, b, box) ||
           segment_crosses_bbox(a, bend2, box) ||
           segment_crosses_bbox(bend2, b, box);
}

static double crossing_penalty(const common::SegmentPoint& candidate,
                               const std::vector<common::SegmentPoint>& neighbor_locs,
                               const std::vector<BBoxD>& cluster_bboxes,
                               const BBoxD& related_bbox) {
    double penalty = 0.0;
    for (const common::SegmentPoint& n : neighbor_locs) {
        for (const BBoxD& box : cluster_bboxes) {
            if (box.valid &&
                !(std::abs(box.xmin - related_bbox.xmin) <= EPS &&
                  std::abs(box.xmax - related_bbox.xmax) <= EPS &&
                  std::abs(box.ymin - related_bbox.ymin) <= EPS &&
                  std::abs(box.ymax - related_bbox.ymax) <= EPS) &&
                segment_crosses_bbox(candidate, n, box)) {
                penalty += 1.0;
            }
        }
    }
    return penalty;
}

static double congestion_penalty(const common::SegmentPoint& p,
                                 const std::vector<BBoxD>& cluster_bboxes,
                                 const std::vector<common::SegmentPoint>& placed_outer) {
    double penalty = 0.0;
    for (const BBoxD& b : cluster_bboxes) {
        if (!b.valid) continue;
        const bool x_inside = p.x >= b.xmin - EPS && p.x <= b.xmax + EPS;
        const bool y_inside = p.y >= b.ymin - EPS && p.y <= b.ymax + EPS;
        if (x_inside) {
            penalty += 1.0 / (1.0 + std::min(std::abs(p.y - b.ymin),
                                             std::abs(p.y - b.ymax)));
        }
        if (y_inside) {
            penalty += 1.0 / (1.0 + std::min(std::abs(p.x - b.xmin),
                                             std::abs(p.x - b.xmax)));
        }
    }
    for (const common::SegmentPoint& old : placed_outer) {
        const double d = manhattan(p, old);
        if (d < 10.0) {
            penalty += (10.0 - d) / 10.0;
        }
    }
    return penalty;
}

static double skew_penalty_for_candidate(const common::LocerResult& result,
                                         const common::TopoTree& tree,
                                         int node_id,
                                         const common::SegmentPoint& candidate) {
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    std::vector<double> child_worst;
    std::vector<double> all_delays;
    auto append_child = [&](int child_id) {
        if (child_id < 0) return true;
        const common::LocerNodeResult& child =
            result.node_results[static_cast<std::size_t>(child_id)];
        if (!child.valid || child.sink_delays_to_node.empty()) return false;
        const double edge = manhattan(candidate, child.loc);
        child_worst.push_back(child.max_sink_delay_to_node + edge);
        for (double d : child.sink_delays_to_node) {
            all_delays.push_back(d + edge);
        }
        return true;
    };
    if (!append_child(node.left) || !append_child(node.right)) return INF;
    if (child_worst.empty()) return 0.0;
    const auto worst_minmax =
        std::minmax_element(child_worst.begin(), child_worst.end());
    double full_skew = 0.0;
    if (!all_delays.empty()) {
        const auto full_minmax =
            std::minmax_element(all_delays.begin(), all_delays.end());
        full_skew = *full_minmax.second - *full_minmax.first;
    }
    return (*worst_minmax.second - *worst_minmax.first) + 0.1 * full_skew;
}

static double bbox_inside_penalty(const common::SegmentPoint& p, const BBoxD& bbox) {
    if (!inside_or_on_bbox(p, bbox)) {
        return 0.0;
    }
    const double dx = std::min(std::abs(p.x - bbox.xmin), std::abs(p.x - bbox.xmax));
    const double dy = std::min(std::abs(p.y - bbox.ymin), std::abs(p.y - bbox.ymax));
    return 1.0 + std::min(dx, dy);
}

static double corridor_width_for_bbox(const BBoxD& bbox) {
    if (!bbox.valid) {
        return 2.0;
    }
    const double width = bbox.xmax - bbox.xmin;
    const double height = bbox.ymax - bbox.ymin;
    return std::max(2.0, 0.15 * std::max(width, height));
}

static bool point_in_selected_side_corridor(const common::SegmentPoint& p,
                                            const BBoxD& bbox,
                                            Side side) {
    if (!bbox.valid || side == Side::Unknown) {
        return false;
    }
    const double w = corridor_width_for_bbox(bbox);
    switch (side) {
        case Side::Left:
            return p.x <= bbox.xmin + w + EPS;
        case Side::Right:
            return p.x >= bbox.xmax - w - EPS;
        case Side::Bottom:
            return p.y <= bbox.ymin + w + EPS;
        case Side::Top:
            return p.y >= bbox.ymax - w - EPS;
        case Side::Unknown:
            return false;
    }
    return false;
}

static bool point_deep_inside_bbox(const common::SegmentPoint& p,
                                   const BBoxD& bbox,
                                   Side side) {
    return inside_or_on_bbox(p, bbox) &&
           !point_in_selected_side_corridor(p, bbox, side);
}

static double deep_inside_bbox_penalty(const common::SegmentPoint& p,
                                       const BBoxD& bbox,
                                       Side side) {
    if (!point_deep_inside_bbox(p, bbox, side)) {
        return 0.0;
    }
    const double depth_x = std::min(std::abs(p.x - bbox.xmin),
                                    std::abs(p.x - bbox.xmax));
    const double depth_y = std::min(std::abs(p.y - bbox.ymin),
                                    std::abs(p.y - bbox.ymax));
    return 100.0 + std::min(depth_x, depth_y);
}

static double preferred_side_monotonic_violation_penalty(
    const common::SegmentPoint& candidate,
    const std::vector<common::SegmentPoint>& child_locs,
    Side preferred_side) {
    if (preferred_side == Side::Unknown) {
        return 0.0;
    }
    double penalty = 0.0;
    for (const common::SegmentPoint& child : child_locs) {
        switch (preferred_side) {
            case Side::Left:
                if (candidate.x > child.x + EPS) {
                    penalty += candidate.x - child.x;
                }
                break;
            case Side::Right:
                if (candidate.x < child.x - EPS) {
                    penalty += child.x - candidate.x;
                }
                break;
            case Side::Bottom:
                if (candidate.y > child.y + EPS) {
                    penalty += candidate.y - child.y;
                }
                break;
            case Side::Top:
                if (candidate.y < child.y - EPS) {
                    penalty += child.y - candidate.y;
                }
                break;
            case Side::Unknown:
                break;
        }
    }
    return penalty;
}

static double side_bonus(const common::SegmentPoint& p,
                         const BBoxD& bbox,
                         Side preferred_side) {
    if (preferred_side == Side::Unknown) {
        return 0.0;
    }
    const Side actual = actual_side_of_point_to_bbox(p, bbox);
    if (actual == preferred_side) {
        return -1.0;
    }
    return 0.0;
}

static double total_manhattan_to_neighbors(
    const common::SegmentPoint& p,
    const std::vector<common::SegmentPoint>& neighbors,
    const common::SegmentPoint& parent_est) {
    double wire = 0.25 * manhattan(p, parent_est);
    for (const common::SegmentPoint& n : neighbors) {
        wire += manhattan(p, n);
    }
    return wire;
}

static double imbalance_penalty_to_children(
    const common::SegmentPoint& p,
    const std::vector<common::SegmentPoint>& neighbors) {
    if (neighbors.size() < 2) {
        return 0.0;
    }
    std::vector<double> distances;
    distances.reserve(neighbors.size());
    for (const common::SegmentPoint& n : neighbors) {
        distances.push_back(manhattan(p, n));
    }
    const auto minmax = std::minmax_element(distances.begin(), distances.end());
    return *minmax.second - *minmax.first;
}

static std::vector<common::SegmentPoint> child_locs(const common::LocerResult& result,
                                                    const common::TopoTree& tree,
                                                    int node_id) {
    std::vector<common::SegmentPoint> out;
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    if (node.left >= 0 && result.node_results[static_cast<std::size_t>(node.left)].valid) {
        out.push_back(result.node_results[static_cast<std::size_t>(node.left)].loc);
    }
    if (node.right >= 0 && result.node_results[static_cast<std::size_t>(node.right)].valid) {
        out.push_back(result.node_results[static_cast<std::size_t>(node.right)].loc);
    }
    return out;
}

static bool candidate_inside_any_cluster(const common::SegmentPoint& p,
                                         const std::vector<BBoxD>& cluster_bboxes) {
    for (const BBoxD& b : cluster_bboxes) {
        if (inside_strict_bbox(p, b)) {
            return true;
        }
    }
    return false;
}

static std::vector<int> child_ids(const common::TopoTree& tree, int node_id) {
    std::vector<int> out;
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    if (node.left >= 0) out.push_back(node.left);
    if (node.right >= 0) out.push_back(node.right);
    return out;
}

static bool place_outer_node(common::LocerResult& result,
                             const common::Problem& problem,
                             const common::TopoTree& tree,
                             int node_id,
                             const std::vector<PhysicalClusterInfo>& clusters,
                             const std::vector<BBoxD>& cluster_bboxes,
                             const std::vector<int>& sink_to_cluster,
                             const GlobalOrderContext* global_ctx,
                             std::vector<common::SegmentPoint>& placed_outer,
                             std::string& err) {
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    const std::vector<int> covered_clusters =
        cluster_ids_for_sinks(node.sink_indices, sink_to_cluster);
    BBoxD related_bbox = bbox_for_cluster_ids(covered_clusters, clusters);
    if (!related_bbox.valid) {
        related_bbox = bbox_from_sinks(problem, node.sink_indices);
    }
    if (!related_bbox.valid) {
        err = "Outer node has invalid related bbox";
        return false;
    }

    const std::vector<common::SegmentPoint> neighbors = child_locs(result, tree, node_id);
    const std::vector<int> neighbor_ids = child_ids(tree, node_id);
    common::SegmentPoint parent_est = to_segment_point(problem.source.loc);
    if (node.parent >= 0 && static_cast<std::size_t>(node.parent) < tree.nodes.size()) {
        parent_est = to_segment_point(tree.nodes[static_cast<std::size_t>(node.parent)].loc);
    }

    const bool is_bridge = node.kind == common::NodeKind::ClusterBridge;
    const bool is_top = node.kind == common::NodeKind::ClusterTop;
    const bool is_bridge_or_top = is_bridge || is_top;
    const bool is_global = node.kind == common::NodeKind::Global;
    const Side preferred_side =
        is_bridge_or_top ? shared_preferred_side_for_clusters(covered_clusters, clusters)
                         : Side::Unknown;

    auto side_switch_penalty = [&](const common::SegmentPoint& p) {
        if (!is_bridge_or_top || covered_clusters.empty()) return 0.0;
        const Side actual = actual_side_of_point_to_bbox(p, related_bbox);
        if (actual == Side::Unknown && is_bridge) {
            return 0.25;
        }
        double penalty = 0.0;
        for (int cluster_id : covered_clusters) {
            if (cluster_id < 0 ||
                static_cast<std::size_t>(cluster_id) >= clusters.size()) {
                continue;
            }
            const Side cluster_side =
                clusters[static_cast<std::size_t>(cluster_id)].preferred_side;
            if (cluster_side != Side::Unknown && actual != cluster_side) {
                penalty += 1.0;
            }
        }
        return penalty;
    };

    auto finalize = [&](const common::SegmentPoint& best,
                        double best_score,
                        int candidate_count,
                        double best_congestion,
                        double best_crossing,
                        double best_skew,
                        bool best_inside,
                        bool best_fallback,
                        Side best_actual_side,
                        bool best_source_side_ok,
                        double best_order_lower,
                        double best_order_upper,
                        const std::string& fallback_suffix,
                        const std::string& loc_mode) -> bool {
        common::LocerNodeResult& out =
            result.node_results[static_cast<std::size_t>(node_id)];
        out.node_id = node_id;
        out.valid = true;
        out.loc = snap_to_integer_grid(best, problem);
        out.node_class = node_class_string(node.kind);
        out.cluster_id = covered_clusters.size() == 1 ? covered_clusters.front() : -1;
        out.loc_mode = loc_mode + fallback_suffix;
        out.loc_score = best_score;
        out.candidate_count = candidate_count;
        out.inside_related_bbox = best_inside;
        out.congestion_penalty = best_congestion;
        out.lshape_penalty = best_crossing;
        out.skew_penalty = best_skew;
        if (node.parent >= 0 &&
            static_cast<std::size_t>(node.parent) < tree.nodes.size() &&
            result.node_results[static_cast<std::size_t>(node.parent)].valid) {
            out.wire_est_to_parent =
                manhattan(out.loc,
                          result.node_results[static_cast<std::size_t>(node.parent)].loc);
        } else if (node.parent >= 0 &&
                   static_cast<std::size_t>(node.parent) < tree.nodes.size()) {
            out.wire_est_to_parent =
                manhattan(out.loc,
                          to_segment_point(
                              tree.nodes[static_cast<std::size_t>(node.parent)].loc));
        } else {
            out.wire_est_to_parent =
                manhattan(out.loc, to_segment_point(problem.source.loc));
        }
        update_profile_from_children(result, tree, node_id, false, nullptr, nullptr);
        placed_outer.push_back(out.loc);
        if (static_cast<std::size_t>(node_id) < g_last_outer_debug_info.size()) {
            OuterDebugInfo& dbg =
                g_last_outer_debug_info[static_cast<std::size_t>(node_id)];
            dbg.valid = true;
            dbg.related_bbox = related_bbox;
            dbg.preferred_side = preferred_side;
            dbg.actual_side = best_actual_side;
            dbg.fallback_side_used = best_fallback && is_bridge_or_top;
            dbg.crossing_penalty = best_crossing;
            dbg.nearest_bbox_distance = distance_to_bbox_boundary(out.loc, related_bbox);
            dbg.neighbor_ids = neighbor_ids;
            dbg.source_side_ok = best_source_side_ok;
            if (is_global && global_ctx != nullptr && global_ctx->valid) {
                dbg.primary_axis = global_ctx->primary_axis;
                dbg.ascending_order = global_ctx->ascending;
                dbg.order_count = static_cast<int>(global_ctx->ordered_nodes.size());
                dbg.source_axis = global_ctx->source_axis;
                auto rank_it = global_ctx->rank_by_node.find(node_id);
                if (rank_it != global_ctx->rank_by_node.end()) {
                    dbg.order_rank = static_cast<int>(rank_it->second);
                }
                dbg.order_lower = best_order_lower;
                dbg.order_upper = best_order_upper;
                dbg.order_fallback_used = best_fallback;
            }
        }
        return true;
    };

    if (is_bridge) {
        std::vector<Side> primary_sides =
            preferred_side != Side::Unknown ? std::vector<Side>{preferred_side}
                                            : all_sides();
        std::vector<Side> fallback_sides =
            preferred_side != Side::Unknown ? other_sides(preferred_side)
                                            : std::vector<Side>{};

        common::SegmentPoint best;
        double best_score = INF;
        int legal_count = 0;
        int candidate_count = 0;
        double best_congestion = 0.0;
        double best_crossing = 0.0;
        double best_skew = 0.0;
        bool best_inside = false;
        bool best_fallback = false;
        Side best_actual_side = Side::Unknown;

        auto score_pass = [&](const std::vector<Side>& sides, bool fallback_phase) {
            std::vector<common::SegmentPoint> candidates =
                generate_bbox_side_candidates(related_bbox, neighbors, parent_est,
                                              problem, sides, true, false);
            candidate_count += static_cast<int>(candidates.size());
            for (const common::SegmentPoint& p : candidates) {
                if (!in_grid(p, problem)) {
                    continue;
                }
                if (preferred_side != Side::Unknown && !fallback_phase &&
                    !point_in_selected_side_corridor(p, related_bbox, preferred_side)) {
                    continue;
                }
                const double skew = skew_penalty_for_candidate(result, tree, node_id, p);
                if (!std::isfinite(skew) || skew >= INF / 2.0) {
                    continue;
                }
                ++legal_count;
                const double cong = congestion_penalty(p, cluster_bboxes, placed_outer);
                const double crossing =
                    crossing_penalty(p, neighbors, cluster_bboxes, related_bbox);
                const double wire = total_manhattan_to_neighbors(p, neighbors, parent_est);
                const double side_penalty = side_switch_penalty(p);
                const double mono_penalty =
                    preferred_side_monotonic_violation_penalty(p, neighbors, preferred_side);
                const double deep_penalty =
                    deep_inside_bbox_penalty(p, related_bbox, preferred_side);
                double score = 300.0 * skew +
                               200.0 * cong +
                               100.0 * crossing +
                                20.0 * wire +
                                50.0 * bbox_inside_penalty(p, related_bbox) +
                               250.0 * deep_penalty +
                               300.0 * mono_penalty +
                                10.0 * side_penalty +
                                30.0 * side_bonus(p, related_bbox, preferred_side);
                if (score < best_score) {
                    best_score = score;
                    best = p;
                    best_congestion = cong;
                    best_crossing = crossing;
                    best_skew = skew;
                    best_inside = inside_or_on_bbox(p, related_bbox);
                    best_fallback = fallback_phase;
                    best_actual_side = actual_side_of_point_to_bbox(p, related_bbox);
                }
            }
        };

        score_pass(primary_sides, false);
        if (best_score >= INF / 2.0 && !fallback_sides.empty()) {
            legal_count = 0;
            score_pass(fallback_sides, true);
        }
        if (best_score >= INF / 2.0 || legal_count == 0) {
            err = "No legal bridge placement candidate for node " + std::to_string(node_id);
            return false;
        }
        return finalize(best, best_score, candidate_count, best_congestion,
                        best_crossing, best_skew, best_inside, best_fallback,
                        best_actual_side, true, 0.0, 0.0,
                        best_fallback ? "_FALLBACK_SIDE" : "",
                        "BRIDGE_CONGESTION_AWARE");
    }

    if (is_top) {
        std::vector<Side> primary_sides =
            preferred_side != Side::Unknown ? std::vector<Side>{preferred_side}
                                            : all_sides();
        std::vector<Side> fallback_sides =
            preferred_side != Side::Unknown ? other_sides(preferred_side)
                                            : std::vector<Side>{};

        common::SegmentPoint best;
        double best_score = INF;
        int legal_count = 0;
        int candidate_count = 0;
        double best_congestion = 0.0;
        double best_crossing = 0.0;
        double best_skew = 0.0;
        bool best_inside = false;
        bool best_fallback = false;
        Side best_actual_side = Side::Unknown;

        auto score_pass = [&](const std::vector<Side>& sides, bool fallback_phase) {
            std::vector<common::SegmentPoint> candidates =
                generate_bbox_side_candidates(related_bbox, neighbors, parent_est,
                                              problem, sides, false, true);
            candidate_count += static_cast<int>(candidates.size());
            for (const common::SegmentPoint& p : candidates) {
                if (!in_grid(p, problem)) {
                    continue;
                }
                if (!outside_bbox(p, related_bbox)) {
                    continue;
                }
                if (preferred_side != Side::Unknown && !fallback_phase &&
                    !point_on_required_side(p, related_bbox, preferred_side)) {
                    continue;
                }
                const double skew = skew_penalty_for_candidate(result, tree, node_id, p);
                if (!std::isfinite(skew) || skew >= INF / 2.0) {
                    continue;
                }
                ++legal_count;
                const double cong = congestion_penalty(p, cluster_bboxes, placed_outer);
                const double crossing =
                    crossing_penalty(p, neighbors, cluster_bboxes, related_bbox);
                const double wire = total_manhattan_to_neighbors(p, neighbors, parent_est);
                const double side_penalty = side_switch_penalty(p);
                double score = 300.0 * skew +
                               200.0 * cong +
                               100.0 * crossing +
                                20.0 * wire +
                                50.0 * side_penalty +
                                30.0 * side_bonus(p, related_bbox, preferred_side);
                if (score < best_score) {
                    best_score = score;
                    best = p;
                    best_congestion = cong;
                    best_crossing = crossing;
                    best_skew = skew;
                    best_inside = inside_or_on_bbox(p, related_bbox);
                    best_fallback = fallback_phase;
                    best_actual_side = actual_side_of_point_to_bbox(p, related_bbox);
                }
            }
        };

        score_pass(primary_sides, false);
        if (best_score >= INF / 2.0 && !fallback_sides.empty()) {
            legal_count = 0;
            score_pass(fallback_sides, true);
        }
        if (best_score >= INF / 2.0 || legal_count == 0) {
            err = "No legal top placement candidate for node " + std::to_string(node_id);
            return false;
        }
        return finalize(best, best_score, candidate_count, best_congestion,
                        best_crossing, best_skew, best_inside, best_fallback,
                        best_actual_side, true, 0.0, 0.0,
                        best_fallback ? "_FALLBACK_SIDE" : "",
                        "TOP_CONGESTION_AWARE");
    }

    if (is_global) {
        if (global_ctx == nullptr || !global_ctx->valid) {
            err = "Global order context is unavailable for node " + std::to_string(node_id);
            return false;
        }

        common::SegmentPoint best;
        double best_score = INF;
        int legal_count = 0;
        int candidate_count = 0;
        double best_congestion = 0.0;
        double best_crossing = 0.0;
        double best_skew = 0.0;
        bool best_inside = false;
        bool best_fallback = false;
        Side best_actual_side = Side::Unknown;
        bool best_source_side_ok = true;
        double best_order_lower = 0.0;
        double best_order_upper = 0.0;

        auto score_pass = [&](bool fallback_phase) {
            const std::vector<common::SegmentPoint> candidates =
                generate_global_order_candidates(problem, tree, *global_ctx,
                                                 related_bbox, node_id,
                                                 neighbors, parent_est,
                                                 fallback_phase);
            candidate_count += static_cast<int>(candidates.size());
            bool any_legal_here = false;
            for (const common::SegmentPoint& p : candidates) {
                if (candidate_inside_any_cluster(p, cluster_bboxes)) {
                    continue;
                }
                double lower = 0.0;
                double upper = 0.0;
                const double order_penalty =
                    global_order_violation_penalty(problem, tree, *global_ctx,
                                                   node_id, p, lower, upper);
                if (!std::isfinite(order_penalty) || order_penalty >= INF / 2.0) {
                    continue;
                }
                const bool source_side_ok =
                    global_order_side_ok(*global_ctx,
                                         project_axis(p, global_ctx->primary_axis));
                if (!source_side_ok) {
                    continue;
                }
                const double skew = skew_penalty_for_candidate(result, tree, node_id, p);
                if (!std::isfinite(skew) || skew >= INF / 2.0) {
                    continue;
                }
                any_legal_here = true;
                ++legal_count;
                const double cong = congestion_penalty(p, cluster_bboxes, placed_outer);
                const double crossing =
                    crossing_penalty(p, neighbors, cluster_bboxes, related_bbox);
                const double wire = total_manhattan_to_neighbors(p, neighbors, parent_est);
                const double trunk_penalty = source_trunk_penalty(*global_ctx, p);
                const int attached_top_id = find_attached_top_for_global(tree, node_id);
                const common::SegmentPoint attached_top_loc =
                    (attached_top_id >= 0 &&
                     static_cast<std::size_t>(attached_top_id) < result.node_results.size() &&
                     result.node_results[static_cast<std::size_t>(attached_top_id)].valid)
                        ? result.node_results[static_cast<std::size_t>(attached_top_id)].loc
                        : (attached_top_id >= 0 &&
                           static_cast<std::size_t>(attached_top_id) < tree.nodes.size())
                              ? to_segment_point(
                                    tree.nodes[static_cast<std::size_t>(attached_top_id)].loc)
                              : common::SegmentPoint{0.0, 0.0};
                const double top_attachment_penalty =
                    attached_top_id >= 0
                        ? global_top_attachment_penalty(p, attached_top_loc)
                        : 0.0;
                const double score =
                    400.0 * skew +
                    250.0 * cong +
                    120.0 * crossing +
                     20.0 * wire +
                    GLOBAL_TOP_ATTACHMENT_WEIGHT * top_attachment_penalty +
                     50.0 * imbalance_penalty_to_children(p, neighbors) +
                    220.0 * trunk_penalty +
                     (inside_or_on_bbox(p, related_bbox) ? 25.0 : 0.0);
                if (score < best_score) {
                    best_score = score;
                    best = p;
                    best_congestion = cong;
                    best_crossing = crossing;
                    best_skew = skew;
                    best_inside = inside_or_on_bbox(p, related_bbox);
                    best_fallback = fallback_phase;
                    best_actual_side = actual_side_of_point_to_bbox(p, related_bbox);
                    best_source_side_ok = source_side_ok;
                    best_order_lower = lower;
                    best_order_upper = upper;
                }
            }
            return any_legal_here;
        };

        const bool found_primary = score_pass(false);
        bool found_any = found_primary;
        if (!found_primary) {
            legal_count = 0;
            found_any = score_pass(true);
        }
        if (!found_any || best_score >= INF / 2.0 || legal_count == 0) {
            err = "No legal global placement candidate for node " + std::to_string(node_id);
            return false;
        }
        return finalize(best, best_score, candidate_count, best_congestion,
                        best_crossing, best_skew, best_inside, best_fallback,
                        best_actual_side, best_source_side_ok,
                        best_order_lower, best_order_upper,
                        best_fallback ? "_FALLBACK_ORDER" : "",
                        "GLOBAL_CONGESTION_AWARE");
    }

    err = "Unsupported outer node kind at node " + std::to_string(node_id);
    return false;
}

static bool assign_dme_fallback_node(common::LocerResult& result,
                                     const common::Problem& problem,
                                     const common::TopoTree& tree,
                                     int node_id,
                                     std::string& err) {
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    common::LocerNodeResult& out = result.node_results[static_cast<std::size_t>(node_id)];
    out.node_id = node_id;
    out.valid = true;
    out.node_class = node_class_string(node.kind);
    out.cluster_id = -1;
    out.candidate_count = 1;
    if (node.kind == common::NodeKind::Sink) {
        if (node.sink_index < 0 ||
            static_cast<std::size_t>(node.sink_index) >= problem.sinks.size()) {
            err = "Fallback sink node has invalid sink index";
            return false;
        }
        out.loc = to_segment_point(problem.sinks[static_cast<std::size_t>(node.sink_index)].loc);
        out.loc_mode = "DME_SINK";
        out.sink_delays_to_node = {0.0};
        stats_from_delays(out);
        return true;
    }
    out.loc = snap_to_integer_grid(to_segment_point(node.loc), problem);
    out.loc_mode = "DME_TOPOLOGY_FALLBACK";
    update_profile_from_children(result, tree, node_id, false, nullptr, nullptr);
    return true;
}

static bool place_tree_recursive(common::LocerResult& result,
                                 const common::Problem& problem,
                                 const common::TopoTree& tree,
                                 int node_id,
                                 const std::vector<PhysicalClusterInfo>& clusters,
                                 const std::vector<BBoxD>& cluster_bboxes,
                                 const std::vector<int>& sink_to_cluster,
                                 const GlobalOrderContext* global_ctx,
                                 std::vector<common::SegmentPoint>& placed_outer,
                                 std::vector<int>& state,
                                 std::string& err) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) {
        err = "Locer traversal references invalid topology node";
        return false;
    }
    const std::size_t idx = static_cast<std::size_t>(node_id);
    if (state[idx] == 1) {
        err = "Cycle detected during locer outer placement";
        return false;
    }
    if (state[idx] == 2) {
        return true;
    }
    state[idx] = 1;
    const common::TopoNode& node = tree.nodes[idx];
    if (node.left >= 0 &&
        !place_tree_recursive(result, problem, tree, node.left, clusters,
                              cluster_bboxes, sink_to_cluster, global_ctx,
                              placed_outer, state, err)) {
        return false;
    }
    if (node.right >= 0 &&
        !place_tree_recursive(result, problem, tree, node.right, clusters,
                              cluster_bboxes, sink_to_cluster, global_ctx,
                              placed_outer, state, err)) {
        return false;
    }
    if (!result.node_results[idx].valid) {
        if (node.kind == common::NodeKind::ClusterBridge ||
            node.kind == common::NodeKind::ClusterTop ||
            node.kind == common::NodeKind::Global) {
            if (!place_outer_node(result, problem, tree, node_id, clusters,
                                  cluster_bboxes, sink_to_cluster, global_ctx,
                                  placed_outer, err)) {
                return false;
            }
        } else if (node.kind == common::NodeKind::Sink ||
                   node.kind == common::NodeKind::ClusterInternal ||
                   node.kind == common::NodeKind::ClusterAccess) {
            if (!assign_dme_fallback_node(result, problem, tree, node_id, err)) {
                return false;
            }
        } else {
            err = "DME node " + std::to_string(node_id) +
                  " was not assigned before outer placement";
            return false;
        }
    }
    state[idx] = 2;
    return true;
}

static std::string basename_without_ext(const std::string& input_path) {
    if (input_path.empty()) return "";
    std::filesystem::path p(input_path);
    return p.stem().string();
}

static bool ensure_loc_dir(std::string& err) {
    std::error_code ec;
    std::filesystem::create_directories("loc", ec);
    if (ec) {
        err = "Cannot create loc directory: " + ec.message();
        return false;
    }
    return true;
}

static bool validate_final_result(const common::Problem& problem,
                                  const common::TopoTree& tree,
                                  const std::vector<BBoxD>& cluster_bboxes,
                                  const std::vector<PhysicalClusterInfo>& clusters,
                                  const std::vector<int>& sink_to_cluster,
                                  const GlobalOrderContext* global_ctx,
                                  const common::LocerResult& result,
                                  std::string& err) {
    if (result.node_results.size() != tree.nodes.size()) {
        err = "Locer result node count does not match topology";
        return false;
    }
    for (const PhysicalClusterInfo& cluster : clusters) {
        if (cluster.preferred_side == Side::Unknown) {
            err = "Physical cluster has unknown preferred side: cluster_id " +
                  std::to_string(cluster.cluster_id);
            return false;
        }
        if (cluster.cluster_top_id < 0 ||
            static_cast<std::size_t>(cluster.cluster_top_id) >= tree.nodes.size()) {
            err = "Physical cluster references invalid ClusterTop node";
            return false;
        }
        const common::TopoNode& top =
            tree.nodes[static_cast<std::size_t>(cluster.cluster_top_id)];
        const BBoxD expected_bbox = bbox_from_sinks(problem, top.sink_indices);
        if (!bbox_equal(cluster.cluster_bbox, expected_bbox)) {
            err = "Cluster bbox mismatch against ClusterTop sinks at node " +
                  std::to_string(cluster.cluster_top_id);
            return false;
        }
        for (int access_id : cluster.access_nodes) {
            if (access_id < 0 || static_cast<std::size_t>(access_id) >= tree.nodes.size()) {
                err = "Physical cluster has invalid access node id";
                return false;
            }
            const common::LocerNodeResult& access_result =
                result.node_results[static_cast<std::size_t>(access_id)];
            if (access_result.cluster_id != cluster.cluster_id) {
                err = "Access node cluster_id mismatch in physical cluster " +
                      std::to_string(cluster.cluster_id);
                return false;
            }
        }
    }
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const common::TopoNode& node = tree.nodes[i];
        const common::LocerNodeResult& out = result.node_results[i];
        if (!out.valid || out.node_id != static_cast<int>(i) || !finite_point(out.loc)) {
            err = "Locer node result invalid at node " + std::to_string(i);
            return false;
        }
        if (!in_grid(out.loc, problem)) {
            err = "Locer node loc out of die at node " + std::to_string(i);
            return false;
        }
        if (!is_integer_point(out.loc)) {
            err = "Locer node loc is not integer grid coordinate at node " +
                  std::to_string(i);
            return false;
        }
        if (out.sink_delays_to_node.empty()) {
            err = "Locer node has empty delay profile at node " + std::to_string(i);
            return false;
        }
        if (out.min_sink_delay_to_node > out.max_sink_delay_to_node + EPS ||
            out.skew_to_node < -EPS) {
            err = "Locer node delay profile invalid at node " + std::to_string(i);
            return false;
        }
        if (node.kind == common::NodeKind::ClusterBridge ||
            node.kind == common::NodeKind::ClusterTop) {
            const std::vector<int> covered_clusters =
                cluster_ids_for_sinks(node.sink_indices, sink_to_cluster);
            BBoxD related = bbox_for_cluster_ids(covered_clusters, clusters);
            if (!related.valid) {
                related = bbox_from_sinks(problem, node.sink_indices);
            }
            const Side required =
                shared_preferred_side_for_clusters(covered_clusters, clusters);
            const bool fallback_used =
                out.loc_mode.find("_FALLBACK_SIDE") != std::string::npos;
            if (node.kind == common::NodeKind::ClusterBridge &&
                required != Side::Unknown && !fallback_used) {
                if (!point_in_selected_side_corridor(out.loc, related, required)) {
                    err = "Bridge node is not on the selected-side corridor at node " +
                          std::to_string(i);
                    return false;
                }
                if (preferred_side_monotonic_violation_penalty(
                        out.loc, child_locs(result, tree, static_cast<int>(i)),
                        required) > EPS) {
                    err = "Bridge node violates preferred-side monotonic constraint at node " +
                          std::to_string(i);
                    return false;
                }
            }
            if (node.kind == common::NodeKind::ClusterTop &&
                !outside_bbox(out.loc, related)) {
                err = "Top node is not outside related bbox at node " +
                      std::to_string(i);
                return false;
            }
            if (node.kind == common::NodeKind::ClusterTop &&
                required != Side::Unknown && !fallback_used &&
                !point_on_required_side(out.loc, related, required)) {
                err = "Top node does not follow shared preferred side at node " +
                      std::to_string(i);
                return false;
            }
        }
        if (node.kind == common::NodeKind::Global &&
            candidate_inside_any_cluster(out.loc, cluster_bboxes)) {
            err = "Global node is inside a cluster bbox at node " + std::to_string(i);
            return false;
        }
        if (node.kind == common::NodeKind::Global) {
            if (global_ctx == nullptr || !global_ctx->valid) {
                err = "Global order context missing during validation at node " +
                      std::to_string(i);
                return false;
            }
            double lower = 0.0;
            double upper = 0.0;
            if (!global_order_bounds_for_node(problem, tree, *global_ctx,
                                              static_cast<int>(i), lower, upper)) {
                err = "Global node is missing an order interval at node " +
                      std::to_string(i);
                return false;
            }
            const double axis_value = project_axis(out.loc, global_ctx->primary_axis);
            if (axis_value < lower - EPS || axis_value > upper + EPS) {
                err = "Global node violates order interval at node " +
                      std::to_string(i);
                return false;
            }
            if (!global_order_side_ok(*global_ctx, axis_value)) {
                err = "Global node violates source-aware ordering at node " +
                      std::to_string(i);
                return false;
            }
            if (out.loc_mode.find("_FALLBACK_ORDER") != std::string::npos &&
                (i >= g_last_outer_debug_info.size() ||
                 !g_last_outer_debug_info[i].valid ||
                 !g_last_outer_debug_info[i].order_fallback_used)) {
                err = "Global node fallback order was not reflected in debug metadata at node " +
                      std::to_string(i);
                return false;
            }
            if (i < g_last_outer_debug_info.size() &&
                g_last_outer_debug_info[i].valid) {
                const OuterDebugInfo& dbg = g_last_outer_debug_info[i];
                if (dbg.order_count != static_cast<int>(global_ctx->ordered_nodes.size())) {
                    err = "Global debug order count mismatch at node " +
                          std::to_string(i);
                    return false;
                }
                if (dbg.order_rank < 0 ||
                    dbg.order_rank >= dbg.order_count) {
                    err = "Global debug order rank invalid at node " +
                          std::to_string(i);
                    return false;
                }
                if (!dbg.source_side_ok) {
                    err = "Global node debug metadata marks source-side violation at node " +
                          std::to_string(i);
                    return false;
                }
            }
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

void debug_output(const LocerResult& result,
                  const common::Problem&,
                  const common::TopoTree& tree) {
    if (!g_debug_enabled) return;
    std::cout << "[LOCER] valid=" << (result.valid ? 1 : 0)
              << " error_msg=" << result.error_msg
              << " num_nodes=" << result.node_results.size() << "\n";
    for (const ClusterDebugInfo& cluster : g_last_cluster_debug_info) {
        std::cout << "[LOCER] physical_cluster cluster_id=" << cluster.cluster_id
                  << " cluster_top_id=" << cluster.cluster_top_id
                  << " bbox=(" << cluster.cluster_bbox.xmin << ","
                  << cluster.cluster_bbox.ymin << ")-("
                  << cluster.cluster_bbox.xmax << ","
                  << cluster.cluster_bbox.ymax << ")"
                  << " external_anchor=(" << cluster.external_anchor.x
                  << "," << cluster.external_anchor.y << ")"
                  << " anchor_source=" << cluster.anchor_source
                  << " dx=" << cluster.anchor_dx
                  << " dy=" << cluster.anchor_dy
                  << " preferred_side=" << side_to_string(cluster.preferred_side)
                  << " tie_used=" << (cluster.tie_used ? 1 : 0)
                  << " dme_access_subtree_count="
                  << cluster.dme_access_subtree_count << "\n";
        std::cout << "[LOCER]   tie_candidates=[";
        for (std::size_t i = 0; i < cluster.tie_candidates.size(); ++i) {
            if (i > 0) std::cout << ",";
            std::cout << side_to_string(cluster.tie_candidates[i]);
        }
        std::cout << "]\n";
        std::cout << "[LOCER]   side_scores LEFT="
                  << cluster.side_scores[0]
                  << " RIGHT=" << cluster.side_scores[1]
                  << " BOTTOM=" << cluster.side_scores[2]
                  << " TOP=" << cluster.side_scores[3] << "\n";
        std::cout << "[LOCER]   access_nodes=[";
        for (std::size_t i = 0; i < cluster.access_nodes.size(); ++i) {
            if (i > 0) std::cout << ",";
            std::cout << cluster.access_nodes[i];
        }
        std::cout << "] access_locs=[";
        for (std::size_t i = 0; i < cluster.access_locs.size(); ++i) {
            if (i > 0) std::cout << ",";
            std::cout << "(" << cluster.access_locs[i].x
                      << "," << cluster.access_locs[i].y << ")";
        }
        std::cout << "]\n";
    }
    for (std::size_t i = 0; i < result.node_results.size(); ++i) {
        if (i >= tree.nodes.size()) continue;
        const common::TopoNode& node = tree.nodes[i];
        const common::LocerNodeResult& out = result.node_results[i];
        std::cout << "[LOCER] node_id=" << out.node_id
                  << " class=" << out.node_class
                  << " cluster_id=" << out.cluster_id
                  << " parent=" << node.parent
                  << " left=" << node.left
                  << " right=" << node.right
                  << " loc=(" << out.loc.x << "," << out.loc.y << ")"
                  << " loc_mode=" << out.loc_mode
                  << " candidate_count=" << out.candidate_count
                  << " loc_score=" << out.loc_score << "\n";
        std::cout << "[LOCER]   inside_related_bbox="
                  << (out.inside_related_bbox ? 1 : 0)
                  << " congestion_penalty=" << out.congestion_penalty
                  << " lshape_penalty=" << out.lshape_penalty
                  << " wire_est_to_parent=" << out.wire_est_to_parent << "\n";
        std::cout << "[LOCER]   sink_delay_count="
                  << out.sink_delays_to_node.size()
                  << " min_sink_delay=" << out.min_sink_delay_to_node
                  << " max_sink_delay=" << out.max_sink_delay_to_node
                  << " skew_to_node=" << out.skew_to_node
                  << " skew_penalty=" << out.skew_penalty << "\n";
        if (i < g_last_outer_debug_info.size() &&
            g_last_outer_debug_info[i].valid) {
            const OuterDebugInfo& dbg = g_last_outer_debug_info[i];
            std::cout << "[LOCER]   related_bbox=("
                      << dbg.related_bbox.xmin << ","
                      << dbg.related_bbox.ymin << ")-("
                      << dbg.related_bbox.xmax << ","
                      << dbg.related_bbox.ymax << ")"
                      << " preferred_side="
                      << side_to_string(dbg.preferred_side)
                      << " actual_side=" << side_to_string(dbg.actual_side)
                      << " fallback_side_used="
                      << (dbg.fallback_side_used ? 1 : 0)
                      << " primary_axis="
                      << primary_axis_to_string(dbg.primary_axis)
                      << " ascending_order="
                      << (dbg.ascending_order ? 1 : 0)
                      << " order_rank=" << dbg.order_rank
                      << " order_count=" << dbg.order_count
                      << " source_axis=" << dbg.source_axis
                      << " order_lower=" << dbg.order_lower
                      << " order_upper=" << dbg.order_upper
                      << " order_fallback_used="
                      << (dbg.order_fallback_used ? 1 : 0)
                      << " source_side_ok="
                      << (dbg.source_side_ok ? 1 : 0)
                      << " nearest_bbox_distance="
                      << dbg.nearest_bbox_distance
                      << " estimated_crossing_penalty="
                      << dbg.crossing_penalty << "\n";
            std::cout << "[LOCER]   neighbor_ids=[";
            for (std::size_t j = 0; j < dbg.neighbor_ids.size(); ++j) {
                if (j > 0) std::cout << ",";
                std::cout << dbg.neighbor_ids[j];
            }
            std::cout << "]\n";
        }
    }
}

bool write_debug_loc_file(const LocerResult& result,
                          const common::Problem&,
                          const common::TopoTree& tree,
                          const std::string& input_path,
                          std::string& error_msg) {
    if (!ensure_loc_dir(error_msg)) {
        return false;
    }
    const std::string base = basename_without_ext(input_path);
    const std::string path = base.empty() ? "loc/loc_debug.txt"
                                          : "loc/" + base + "_loc.txt";
    std::ofstream fout(path);
    if (!fout) {
        error_msg = "Cannot open loc debug file: " + path;
        return false;
    }
    fout << "# LOCER_DEBUG_LOC v1\n";
    fout << "# valid=" << (result.valid ? 1 : 0) << "\n";
    fout << "# num_nodes=" << result.node_results.size() << "\n";
    fout << "# stage0_columns: cluster_id cluster_top_id bbox_lx bbox_ly "
            "bbox_ux bbox_uy anchor_x anchor_y anchor_source dx dy "
            "preferred_side tie_used tie_candidates score_left score_right "
            "score_bottom score_top\n";
    for (const ClusterDebugInfo& cluster : g_last_cluster_debug_info) {
        fout << "stage0 " << cluster.cluster_id << " "
             << cluster.cluster_top_id << " "
             << cluster.cluster_bbox.xmin << " "
             << cluster.cluster_bbox.ymin << " "
             << cluster.cluster_bbox.xmax << " "
             << cluster.cluster_bbox.ymax << " "
             << cluster.external_anchor.x << " "
             << cluster.external_anchor.y << " "
             << cluster.anchor_source << " "
             << cluster.anchor_dx << " "
             << cluster.anchor_dy << " "
             << side_to_string(cluster.preferred_side) << " "
             << (cluster.tie_used ? 1 : 0) << " ";
        if (cluster.tie_candidates.empty()) {
            fout << "NONE";
        } else {
            for (std::size_t j = 0; j < cluster.tie_candidates.size(); ++j) {
                if (j > 0) fout << ",";
                fout << side_to_string(cluster.tie_candidates[j]);
            }
        }
        fout << " " << cluster.side_scores[0]
             << " " << cluster.side_scores[1]
             << " " << cluster.side_scores[2]
             << " " << cluster.side_scores[3] << "\n";
    }
    fout << "# columns: node_id class cluster_id x y loc_mode parent left right "
            "candidate_count loc_score inside_related_bbox congestion_penalty "
            "lshape_penalty wire_est_to_parent sink_delay_count min_sink_delay "
            "max_sink_delay skew_to_node skew_penalty\n";
    for (std::size_t i = 0; i < result.node_results.size(); ++i) {
        const common::LocerNodeResult& out = result.node_results[i];
        const common::TopoNode& node = tree.nodes[i];
        fout << "node " << out.node_id << " " << out.node_class << " "
             << out.cluster_id << " " << out.loc.x << " " << out.loc.y << " "
             << out.loc_mode << " " << node.parent << " " << node.left << " "
             << node.right << " " << out.candidate_count << " " << out.loc_score
             << " " << (out.inside_related_bbox ? 1 : 0) << " "
             << out.congestion_penalty << " " << out.lshape_penalty << " "
             << out.wire_est_to_parent << " "
             << out.sink_delays_to_node.size() << " "
             << out.min_sink_delay_to_node << " "
             << out.max_sink_delay_to_node << " " << out.skew_to_node << " "
             << out.skew_penalty << "\n";
    }
    fout << "# outer_columns: node_id related_bbox_lx related_bbox_ly "
            "related_bbox_ux related_bbox_uy inherited_preferred_side "
            "actual_side fallback_side_used primary_axis ascending_order "
            "order_rank order_count source_axis order_lower "
            "order_upper order_fallback_used source_side_ok "
            "nearest_bbox_distance estimated_crossing_penalty neighbor_ids\n";
    for (std::size_t i = 0; i < g_last_outer_debug_info.size(); ++i) {
        const OuterDebugInfo& dbg = g_last_outer_debug_info[i];
        if (!dbg.valid) {
            continue;
        }
        fout << "outer " << i << " "
             << dbg.related_bbox.xmin << " "
             << dbg.related_bbox.ymin << " "
             << dbg.related_bbox.xmax << " "
             << dbg.related_bbox.ymax << " "
             << side_to_string(dbg.preferred_side) << " "
             << side_to_string(dbg.actual_side) << " "
             << (dbg.fallback_side_used ? 1 : 0) << " "
             << primary_axis_to_string(dbg.primary_axis) << " "
             << (dbg.ascending_order ? 1 : 0) << " "
             << dbg.order_rank << " "
             << dbg.order_count << " "
             << dbg.source_axis << " "
             << dbg.order_lower << " "
             << dbg.order_upper << " "
             << (dbg.order_fallback_used ? 1 : 0) << " "
             << (dbg.source_side_ok ? 1 : 0) << " "
             << dbg.nearest_bbox_distance << " "
             << dbg.crossing_penalty << " ";
        if (dbg.neighbor_ids.empty()) {
            fout << "NONE";
        } else {
            for (std::size_t j = 0; j < dbg.neighbor_ids.size(); ++j) {
                if (j > 0) fout << ",";
                fout << dbg.neighbor_ids[j];
            }
        }
        fout << "\n";
    }
    return true;
}

LocerResult run(const common::Problem& problem,
                const common::TopoTree& tree,
                const std::string& input_path) {
    LocerResult result;
    g_last_cluster_debug_info.clear();
    g_last_outer_debug_info.clear();
    if (!problem.valid) {
        result.error_msg = "Cannot run locer on invalid problem: " + problem.error_msg;
        return result;
    }
    if (!tree.valid) {
        result.error_msg = "Cannot run locer on invalid topology tree: " + tree.error_msg;
        return result;
    }
    if (tree.nodes.empty()) {
        result.error_msg = "Cannot run locer on empty topology tree";
        return result;
    }

    result.node_results.resize(tree.nodes.size());
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        result.node_results[i].node_id = static_cast<int>(i);
        result.node_results[i].node_class = node_class_string(tree.nodes[i].kind);
    }

    std::string err;
    std::vector<PhysicalClusterInfo> clusters =
        build_physical_clusters(problem, tree, err);
    if (!err.empty()) {
        result.error_msg = err;
        return result;
    }

    std::vector<BBoxD> cluster_bboxes;
    cluster_bboxes.reserve(clusters.size());
    for (const PhysicalClusterInfo& cluster : clusters) {
        cluster_bboxes.push_back(cluster.cluster_bbox);
    }
    const std::vector<int> sink_to_cluster =
        build_sink_to_cluster(problem, clusters);
    const GlobalOrderContext global_ctx = build_global_order_context(problem, tree);
    g_last_outer_debug_info.resize(tree.nodes.size());

    std::vector<Side> access_side(tree.nodes.size(), Side::Unknown);
    for (PhysicalClusterInfo& cluster : clusters) {
        ClusterDebugInfo cluster_debug;
        cluster_debug.cluster_id = cluster.cluster_id;
        cluster_debug.cluster_top_id = cluster.cluster_top_id;
        cluster_debug.cluster_bbox = cluster.cluster_bbox;
        cluster_debug.access_nodes = cluster.access_nodes;
        cluster_debug.dme_access_subtree_count =
            static_cast<int>(cluster.access_nodes.size());

        std::string anchor_source;
        const common::SegmentPoint anchor =
            resolve_external_anchor_for_cluster_top(
                problem, tree, cluster.cluster_top_id, anchor_source);
        cluster_debug.external_anchor = anchor;
        cluster_debug.anchor_source = anchor_source;
        cluster.preferred_side = choose_preferred_side_from_anchor(
            problem, cluster, cluster_bboxes, anchor, cluster_debug.side_scores,
            cluster_debug.anchor_dx, cluster_debug.anchor_dy,
            cluster_debug.tie_used, cluster_debug.tie_candidates);
        cluster_debug.preferred_side = cluster.preferred_side;
        g_last_cluster_debug_info.push_back(cluster_debug);
    }

    for (PhysicalClusterInfo& cluster : clusters) {
        std::vector<CachedDmeRun> cached_runs;
        cached_runs.reserve(cluster.access_nodes.size());
        for (int access_id : cluster.access_nodes) {
            common::ClusterDmeInput input =
                build_cluster_input(tree, access_id, cluster.cluster_id);
            if (!input.valid) {
                result.error_msg = "Build cluster DME input failed for cluster " +
                                   std::to_string(cluster.cluster_id) +
                                   " access " + std::to_string(access_id) +
                                   ": " + input.error_msg;
                return result;
            }
            common::BottomUpResult bu_result = bu::run(problem, input);
            if (!bu_result.valid) {
                result.error_msg = "BU failed for cluster " +
                                   std::to_string(cluster.cluster_id) +
                                   " access " + std::to_string(access_id) +
                                   ": " + bu_result.error_msg;
                return result;
            }
            CachedDmeRun cached;
            cached.access_id = access_id;
            cached.input = input;
            cached.bu_result = bu_result;
            cached_runs.push_back(cached);
        }

        for (const CachedDmeRun& cached : cached_runs) {
            const int access_id = cached.access_id;
            access_side[static_cast<std::size_t>(access_id)] = cluster.preferred_side;
            common::TopDownConfig config;
            config.has_root_loc = true;
            config.root_loc = choose_access_loc(
                cached.bu_result.node_results[static_cast<std::size_t>(
                    cached.input.root_local_id)].ms,
                cluster.preferred_side);
            config.root_loc_mode = side_to_root_loc_mode(cluster.preferred_side);
            common::TopDownResult td_result =
                td::run(problem, cached.input, cached.bu_result, config);
            if (!td_result.valid) {
                result.error_msg = "TD failed for cluster " +
                                   std::to_string(cluster.cluster_id) +
                                   " access " + std::to_string(access_id) +
                                   ": " + td_result.error_msg;
                return result;
            }
            write_dme_results(result, problem, tree, cached.input,
                              cached.bu_result, td_result);
            if (cluster.cluster_id >= 0 &&
                static_cast<std::size_t>(cluster.cluster_id) <
                    g_last_cluster_debug_info.size()) {
                g_last_cluster_debug_info[static_cast<std::size_t>(
                    cluster.cluster_id)].access_locs.push_back(config.root_loc);
            }
            if (g_debug_enabled) {
                std::cout << "[LOCER] cluster_id=" << cluster.cluster_id
                          << " preferred_side="
                          << side_to_string(cluster.preferred_side)
                          << " access_node=" << access_id
                          << " access_loc=(" << config.root_loc.x << ","
                          << config.root_loc.y << ")"
                          << " dme_node_count=" << cached.input.nodes.size()
                          << "\n";
            }
        }
    }

    std::vector<common::SegmentPoint> placed_outer;
    std::vector<int> state(tree.nodes.size(), 0);
    if (!tree.source_children.empty()) {
        for (int child : tree.source_children) {
            if (!place_tree_recursive(result, problem, tree, child, clusters,
                                      cluster_bboxes, sink_to_cluster,
                                      &global_ctx, placed_outer, state, err)) {
                result.error_msg = err;
                return result;
            }
        }
    } else if (tree.root >= 0) {
        if (!place_tree_recursive(result, problem, tree, tree.root, clusters,
                                  cluster_bboxes, sink_to_cluster,
                                  &global_ctx, placed_outer, state, err)) {
            result.error_msg = err;
            return result;
        }
    } else {
        result.error_msg = "Topology tree has neither root nor source children";
        return result;
    }

    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        if (tree.nodes[i].kind == common::NodeKind::ClusterAccess &&
            access_side[i] == Side::Unknown) {
            result.error_msg = "Access node missing physical cluster assignment at node " +
                               std::to_string(i);
            return result;
        }
    }
    for (const PhysicalClusterInfo& cluster : clusters) {
        for (int access_id : cluster.access_nodes) {
            if (access_side[static_cast<std::size_t>(access_id)] !=
                cluster.preferred_side) {
                result.error_msg =
                    "Inconsistent preferred side assignment in physical cluster " +
                    std::to_string(cluster.cluster_id);
                return result;
            }
        }
    }

    if (!validate_final_result(problem, tree, cluster_bboxes, clusters,
                               sink_to_cluster, &global_ctx, result, err)) {
        result.error_msg = err;
        return result;
    }

    result.valid = true;
    if (g_debug_enabled) {
        debug_output(result, problem, tree);
    }
    if (g_debug_file_enabled) {
        std::string file_err;
        if (!write_debug_loc_file(result, problem, tree, input_path, file_err)) {
            result.valid = false;
            result.error_msg = file_err;
        }
    }
    return result;
}

}  // namespace locer
