#include "td.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace td {
namespace {

bool g_debug_enabled = false;

static constexpr double EPS = 1e-9;
static constexpr double SNAP_EPS = 1e-6;
static constexpr double INF = 1e100;
static constexpr int MAX_CANDIDATES = 32;

struct TRR {
    double u_min = 0.0;
    double u_max = 0.0;
    double v_min = 0.0;
    double v_max = 0.0;
    bool valid = false;
};

static double to_u(double x, double y) {
    return x + y;
}

static double to_v(double x, double y) {
    return x - y;
}

static common::SegmentPoint from_uv(double u, double v) {
    return common::SegmentPoint{(u + v) / 2.0, (u - v) / 2.0};
}

static bool approx_le(double a, double b) {
    return a <= b + EPS;
}

static bool near(double a, double b) {
    return std::abs(a - b) <= SNAP_EPS;
}

static double clamp_double(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

static double manhattan(const common::SegmentPoint& a,
                        const common::SegmentPoint& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

static bool is_finite_point(const common::SegmentPoint& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::abs(p.x) < INF / 2.0 && std::abs(p.y) < INF / 2.0;
}

static common::MergingSegment invalid_segment() {
    return common::MergingSegment{};
}

static common::MergingSegment point_segment(double x, double y) {
    common::MergingSegment ms;
    ms.p1 = common::SegmentPoint{x, y};
    ms.p2 = ms.p1;
    ms.valid = true;
    return ms;
}

static TRR invalid_trr() {
    return TRR{};
}

static TRR normalize_trr(double u_min, double u_max, double v_min, double v_max) {
    TRR trr;
    trr.u_min = std::min(u_min, u_max);
    trr.u_max = std::max(u_min, u_max);
    trr.v_min = std::min(v_min, v_max);
    trr.v_max = std::max(v_min, v_max);
    trr.valid = approx_le(trr.u_min, trr.u_max) &&
                approx_le(trr.v_min, trr.v_max);
    return trr.valid ? trr : invalid_trr();
}

static bool is_valid_ms_segment(const common::MergingSegment& ms) {
    if (!ms.valid || !is_finite_point(ms.p1) || !is_finite_point(ms.p2)) {
        return false;
    }
    const double du = std::abs(to_u(ms.p1.x, ms.p1.y) - to_u(ms.p2.x, ms.p2.y));
    const double dv = std::abs(to_v(ms.p1.x, ms.p1.y) - to_v(ms.p2.x, ms.p2.y));
    return du <= SNAP_EPS || dv <= SNAP_EPS;
}

static TRR segment_to_trr(const common::MergingSegment& ms) {
    if (!is_valid_ms_segment(ms)) {
        return invalid_trr();
    }
    return normalize_trr(to_u(ms.p1.x, ms.p1.y), to_u(ms.p2.x, ms.p2.y),
                         to_v(ms.p1.x, ms.p1.y), to_v(ms.p2.x, ms.p2.y));
}

static TRR expand_trr(const TRR& base, double radius) {
    if (!base.valid || radius < -EPS) {
        return invalid_trr();
    }
    const double r = std::max(0.0, radius);
    return normalize_trr(base.u_min - r, base.u_max + r,
                         base.v_min - r, base.v_max + r);
}

static TRR intersect_trr(const TRR& a, const TRR& b) {
    if (!a.valid || !b.valid) {
        return invalid_trr();
    }
    return normalize_trr(std::max(a.u_min, b.u_min), std::min(a.u_max, b.u_max),
                         std::max(a.v_min, b.v_min), std::min(a.v_max, b.v_max));
}

static bool point_in_trr(const common::SegmentPoint& p, const TRR& r) {
    if (!r.valid) {
        return false;
    }
    const double u = to_u(p.x, p.y);
    const double v = to_v(p.x, p.y);
    return u >= r.u_min - SNAP_EPS && u <= r.u_max + SNAP_EPS &&
           v >= r.v_min - SNAP_EPS && v <= r.v_max + SNAP_EPS;
}

static bool point_on_ms(const common::SegmentPoint& p,
                        const common::MergingSegment& ms) {
    return is_valid_ms_segment(ms) && point_in_trr(p, segment_to_trr(ms));
}

static common::MergingSegment trr_to_representative_ms(const TRR& trr) {
    if (!trr.valid) {
        return invalid_segment();
    }
    if (trr.u_max - trr.u_min >= trr.v_max - trr.v_min) {
        const double v_mid = (trr.v_min + trr.v_max) / 2.0;
        common::MergingSegment ms;
        ms.p1 = from_uv(trr.u_min, v_mid);
        ms.p2 = from_uv(trr.u_max, v_mid);
        ms.valid = true;
        return ms;
    }
    const double u_mid = (trr.u_min + trr.u_max) / 2.0;
    common::MergingSegment ms;
    ms.p1 = from_uv(u_mid, trr.v_min);
    ms.p2 = from_uv(u_mid, trr.v_max);
    ms.valid = true;
    return ms;
}

static common::SegmentPoint midpoint_of_ms(const common::MergingSegment& ms) {
    return common::SegmentPoint{(ms.p1.x + ms.p2.x) / 2.0,
                                (ms.p1.y + ms.p2.y) / 2.0};
}

static common::SegmentPoint nearest_point_on_ms_to_point(
    const common::MergingSegment& ms,
    const common::SegmentPoint& p) {
    const TRR base = segment_to_trr(ms);
    if (!base.valid) {
        return common::SegmentPoint{INF, INF};
    }
    return from_uv(clamp_double(to_u(p.x, p.y), base.u_min, base.u_max),
                   clamp_double(to_v(p.x, p.y), base.v_min, base.v_max));
}

static bool add_unique_point(std::vector<common::SegmentPoint>& points,
                             const common::SegmentPoint& p,
                             const common::MergingSegment& ms) {
    if (!is_finite_point(p) || !point_on_ms(p, ms)) {
        return false;
    }
    for (const common::SegmentPoint& old : points) {
        if (near(old.x, p.x) && near(old.y, p.y)) {
            return false;
        }
    }
    points.push_back(p);
    return true;
}

static void add_x_projection(std::vector<common::SegmentPoint>& points,
                             double x,
                             const common::MergingSegment& ms) {
    const TRR base = segment_to_trr(ms);
    if (!base.valid) return;
    if (std::abs(base.u_max - base.u_min) <= SNAP_EPS) {
        add_unique_point(points, from_uv(base.u_min, 2.0 * x - base.u_min), ms);
    }
    if (std::abs(base.v_max - base.v_min) <= SNAP_EPS) {
        add_unique_point(points, from_uv(2.0 * x - base.v_min, base.v_min), ms);
    }
}

static void add_y_projection(std::vector<common::SegmentPoint>& points,
                             double y,
                             const common::MergingSegment& ms) {
    const TRR base = segment_to_trr(ms);
    if (!base.valid) return;
    if (std::abs(base.u_max - base.u_min) <= SNAP_EPS) {
        add_unique_point(points, from_uv(base.u_min, base.u_min - 2.0 * y), ms);
    }
    if (std::abs(base.v_max - base.v_min) <= SNAP_EPS) {
        add_unique_point(points, from_uv(base.v_min + 2.0 * y, base.v_min), ms);
    }
}

static bool segment_crosses_sink(const common::SegmentPoint& a,
                                 const common::SegmentPoint& b,
                                 const common::Problem& problem,
                                 int allowed_parent_sink,
                                 int allowed_child_sink) {
    const bool horizontal = near(a.y, b.y);
    const bool vertical = near(a.x, b.x);
    if (!horizontal && !vertical) {
        return true;
    }
    for (std::size_t i = 0; i < problem.sinks.size(); ++i) {
        const int idx = static_cast<int>(i);
        if (idx == allowed_parent_sink || idx == allowed_child_sink) {
            continue;
        }
        const common::Point& s = problem.sinks[i].loc;
        if (horizontal && near(a.y, s.y) &&
            s.x >= std::min(a.x, b.x) - SNAP_EPS &&
            s.x <= std::max(a.x, b.x) + SNAP_EPS) {
            return true;
        }
        if (vertical && near(a.x, s.x) &&
            s.y >= std::min(a.y, b.y) - SNAP_EPS &&
            s.y <= std::max(a.y, b.y) + SNAP_EPS) {
            return true;
        }
    }
    return false;
}

static int lshape_forbidden_sink_penalty(const common::SegmentPoint& a,
                                         const common::SegmentPoint& b,
                                         const common::Problem& problem,
                                         int allowed_parent_sink,
                                         int allowed_child_sink) {
    if (near(a.x, b.x) || near(a.y, b.y)) {
        return segment_crosses_sink(a, b, problem, allowed_parent_sink,
                                    allowed_child_sink) ? 1 : 0;
    }
    const common::SegmentPoint hv{b.x, a.y};
    const common::SegmentPoint vh{a.x, b.y};
    int penalty = 0;
    if (segment_crosses_sink(a, hv, problem, allowed_parent_sink, allowed_child_sink) ||
        segment_crosses_sink(hv, b, problem, allowed_parent_sink, allowed_child_sink)) {
        ++penalty;
    }
    if (segment_crosses_sink(a, vh, problem, allowed_parent_sink, allowed_child_sink) ||
        segment_crosses_sink(vh, b, problem, allowed_parent_sink, allowed_child_sink)) {
        ++penalty;
    }
    return penalty;
}

static double loc_score(const common::SegmentPoint& loc,
                        const common::SegmentPoint& parent_loc,
                        double assigned_edge,
                        const common::Problem& problem,
                        int allowed_parent_sink,
                        int allowed_child_sink) {
    const double d = manhattan(parent_loc, loc);
    return 10000.0 * std::max(0.0, d - assigned_edge) +
           100.0 * lshape_forbidden_sink_penalty(parent_loc, loc, problem,
                                                 allowed_parent_sink,
                                                 allowed_child_sink) +
           std::abs(d - assigned_edge) +
           0.001 * d;
}

static std::vector<common::SegmentPoint> generate_candidate_locs(
    const common::MergingSegment& feasible_ms,
    const common::SegmentPoint& parent_loc,
    const common::Problem& problem) {
    std::vector<common::SegmentPoint> points;
    add_unique_point(points, midpoint_of_ms(feasible_ms), feasible_ms);
    add_unique_point(points, feasible_ms.p1, feasible_ms);
    add_unique_point(points, feasible_ms.p2, feasible_ms);
    add_unique_point(points, nearest_point_on_ms_to_point(feasible_ms, parent_loc),
                     feasible_ms);
    for (const common::Sink& sink : problem.sinks) {
        add_x_projection(points, sink.loc.x - 1.0, feasible_ms);
        add_x_projection(points, sink.loc.x + 1.0, feasible_ms);
        add_y_projection(points, sink.loc.y - 1.0, feasible_ms);
        add_y_projection(points, sink.loc.y + 1.0, feasible_ms);
    }
    if (points.empty()) {
        points.push_back(midpoint_of_ms(feasible_ms));
    }
    return points;
}

static const char* dme_class_to_string(common::DmeNodeClass node_class) {
    switch (node_class) {
        case common::DmeNodeClass::Sink:
            return "SINK";
        case common::DmeNodeClass::Internal:
            return "INTERNAL";
        case common::DmeNodeClass::Access:
            return "ACCESS";
    }
    return "UNKNOWN";
}

static std::string ms_to_string(const common::MergingSegment& ms) {
    if (!ms.valid) return "INVALID";
    std::ostringstream oss;
    oss << "[(" << ms.p1.x << "," << ms.p1.y << "),("
        << ms.p2.x << "," << ms.p2.y << ")]";
    return oss.str();
}

static bool validate_bu_node(const common::BottomUpNodeResult& node,
                             int local_id,
                             std::string& err) {
    if (!node.valid) {
        err = "BU result for local node " + std::to_string(local_id) + " is invalid";
        return false;
    }
    if (node.local_id != local_id) {
        err = "BU local_id mismatch at local node " + std::to_string(local_id);
        return false;
    }
    if (!is_valid_ms_segment(node.ms)) {
        err = "BU result for local node " + std::to_string(local_id) +
              " has invalid merging segment";
        return false;
    }
    if (node.min_delay > node.max_delay + EPS || node.skew < -EPS) {
        err = "BU delay interval invalid at local node " + std::to_string(local_id);
        return false;
    }
    return true;
}

static bool root_loc_matches_config(const common::TopDownNodeResult& root,
                                    const common::TopDownConfig& config) {
    return near(root.loc.x, config.root_loc.x) &&
           near(root.loc.y, config.root_loc.y);
}

static bool place_child(int parent_id,
                        int child_id,
                        bool is_left_child,
                        const common::Problem& problem,
                        const common::ClusterDmeInput& input,
                        const common::BottomUpResult& bu_result,
                        common::TopDownResult& result,
                        std::string& err) {
    const common::BottomUpNodeResult& parent_bu =
        bu_result.node_results[static_cast<std::size_t>(parent_id)];
    const common::BottomUpNodeResult& child_bu =
        bu_result.node_results[static_cast<std::size_t>(child_id)];
    if (!validate_bu_node(parent_bu, parent_id, err) ||
        !validate_bu_node(child_bu, child_id, err)) {
        return false;
    }

    const double assigned_edge = std::max(0.0, is_left_child
        ? parent_bu.edge_to_left
        : parent_bu.edge_to_right);
    const common::SegmentPoint parent_loc =
        result.node_results[static_cast<std::size_t>(parent_id)].loc;
    const TRR parent_trr = expand_trr(segment_to_trr(point_segment(parent_loc.x,
                                                                   parent_loc.y)),
                                      assigned_edge);
    const TRR child_base = segment_to_trr(child_bu.ms);
    const TRR feasible = intersect_trr(child_base, parent_trr);
    const common::MergingSegment feasible_ms =
        feasible.valid ? trr_to_representative_ms(feasible) : invalid_segment();

    const common::ClusterDmeNode& parent_node =
        input.nodes[static_cast<std::size_t>(parent_id)];
    const common::ClusterDmeNode& child_node =
        input.nodes[static_cast<std::size_t>(child_id)];
    const int allowed_parent_sink =
        parent_node.node_class == common::DmeNodeClass::Sink ? parent_node.sink_index : -1;
    const int allowed_child_sink =
        child_node.node_class == common::DmeNodeClass::Sink ? child_node.sink_index : -1;

    std::vector<common::SegmentPoint> candidates;
    common::SegmentPoint selected;
    double best_score = INF;
    bool used_feasible = false;
    std::string mode;
    if (feasible_ms.valid) {
        candidates = generate_candidate_locs(feasible_ms, parent_loc, problem);
        std::sort(candidates.begin(), candidates.end(),
                  [&](const common::SegmentPoint& a, const common::SegmentPoint& b) {
                      return loc_score(a, parent_loc, assigned_edge, problem,
                                       allowed_parent_sink, allowed_child_sink) <
                             loc_score(b, parent_loc, assigned_edge, problem,
                                       allowed_parent_sink, allowed_child_sink);
                  });
        if (candidates.size() > MAX_CANDIDATES) {
            candidates.resize(MAX_CANDIDATES);
        }
        for (const common::SegmentPoint& candidate : candidates) {
            const double score = loc_score(candidate, parent_loc, assigned_edge,
                                           problem, allowed_parent_sink,
                                           allowed_child_sink);
            if (score < best_score) {
                best_score = score;
                selected = candidate;
            }
        }
        used_feasible = true;
        mode = "FEASIBLE_CANDIDATE";
    } else {
        selected = nearest_point_on_ms_to_point(child_bu.ms, parent_loc);
        best_score = loc_score(selected, parent_loc, assigned_edge, problem,
                               allowed_parent_sink, allowed_child_sink);
        candidates.push_back(selected);
        mode = "NEAREST_MS_FALLBACK";
    }

    if (!is_finite_point(selected) || !point_on_ms(selected, child_bu.ms)) {
        err = "TD selected invalid loc for local child " + std::to_string(child_id);
        return false;
    }

    common::TopDownNodeResult out;
    out.node_id = child_node.origin_node_id;
    out.local_id = child_id;
    out.origin_node_id = child_node.origin_node_id;
    out.valid = true;
    out.loc = selected;
    out.parent_id = parent_node.origin_node_id;
    out.parent_local_id = parent_id;
    out.parent_origin_node_id = parent_node.origin_node_id;
    out.assigned_edge_to_parent = assigned_edge;
    out.geometric_distance_to_parent = manhattan(parent_loc, selected);
    out.feasible_ms = feasible_ms.valid ? feasible_ms : point_segment(selected.x, selected.y);
    out.used_feasible_intersection = used_feasible;
    out.loc_mode = mode;
    out.candidate_count = static_cast<int>(candidates.size());
    out.loc_score = best_score;
    out.min_delay = child_bu.min_delay;
    out.max_delay = child_bu.max_delay;
    out.skew = child_bu.skew;
    result.node_results[static_cast<std::size_t>(child_id)] = out;
    return true;
}

static bool place_recursive(int local_id,
                            const common::Problem& problem,
                            const common::ClusterDmeInput& input,
                            const common::BottomUpResult& bu_result,
                            common::TopDownResult& result,
                            std::vector<int>& state,
                            std::string& err) {
    if (local_id < 0 || static_cast<std::size_t>(local_id) >= input.nodes.size()) {
        err = "TD local node id out of range";
        return false;
    }
    const std::size_t idx = static_cast<std::size_t>(local_id);
    if (state[idx] == 1) {
        err = "Cycle detected during TD";
        return false;
    }
    if (state[idx] == 2) {
        return true;
    }
    if (!result.node_results[idx].valid) {
        err = "TD local node " + std::to_string(local_id) + " was not placed";
        return false;
    }
    state[idx] = 1;
    const common::ClusterDmeNode& node = input.nodes[idx];
    if (node.node_class != common::DmeNodeClass::Sink) {
        if (node.left < 0 || node.right < 0 || node.left == node.right ||
            static_cast<std::size_t>(node.left) >= input.nodes.size() ||
            static_cast<std::size_t>(node.right) >= input.nodes.size()) {
            err = "TD internal/access node has invalid children";
            return false;
        }
        if (!place_child(local_id, node.left, true, problem, input, bu_result,
                         result, err) ||
            !place_recursive(node.left, problem, input, bu_result, result,
                             state, err) ||
            !place_child(local_id, node.right, false, problem, input, bu_result,
                         result, err) ||
            !place_recursive(node.right, problem, input, bu_result, result,
                             state, err)) {
            return false;
        }
    }
    state[idx] = 2;
    return true;
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_output(const TopDownResult& result,
                  const common::Problem&,
                  const ClusterDmeInput& input,
                  const common::BottomUpResult& bu_result,
                  const TopDownConfig& config) {
    if (!g_debug_enabled) {
        return;
    }
    std::cout << "[TD] valid=" << (result.valid ? 1 : 0)
              << " error_msg=" << result.error_msg
              << " cluster_id=" << result.cluster_id
              << " root_local_id=" << result.root_local_id
              << " root_origin_node_id=" << result.root_origin_node_id
              << " root_loc=(" << config.root_loc.x << "," << config.root_loc.y << ")"
              << " root_loc_mode=" << config.root_loc_mode
              << " num_node_results=" << result.node_results.size() << "\n";
    for (std::size_t i = 0; i < result.node_results.size(); ++i) {
        if (i >= input.nodes.size() || i >= bu_result.node_results.size()) continue;
        const common::ClusterDmeNode& node = input.nodes[i];
        const common::TopDownNodeResult& td_node = result.node_results[i];
        const common::BottomUpNodeResult& bu_node = bu_result.node_results[i];
        std::cout << "[TD] local_id=" << node.local_id
                  << " origin_node_id=" << node.origin_node_id
                  << " class=" << dme_class_to_string(node.node_class)
                  << " parent_local=" << node.parent
                  << " parent_origin=" << td_node.parent_origin_node_id
                  << " left=" << node.left
                  << " right=" << node.right
                  << " sink_index=" << node.sink_index << "\n";
        if (!td_node.valid) {
            std::cout << "[TD]   td_result=INVALID\n";
            continue;
        }
        const double over = std::max(0.0, td_node.geometric_distance_to_parent -
                                            td_node.assigned_edge_to_parent);
        std::cout << "[TD]   loc=(" << td_node.loc.x << "," << td_node.loc.y << ")"
                  << " assigned_edge=" << td_node.assigned_edge_to_parent
                  << " geometric_distance=" << td_node.geometric_distance_to_parent
                  << " over_assigned_edge=" << over << "\n";
        std::cout << "[TD]   feasible_ms=" << ms_to_string(td_node.feasible_ms)
                  << " used_feasible_intersection="
                  << (td_node.used_feasible_intersection ? 1 : 0)
                  << " loc_mode=" << td_node.loc_mode
                  << " candidate_count=" << td_node.candidate_count
                  << " loc_score=" << td_node.loc_score << "\n";
        std::cout << "[TD]   bu_ms=" << ms_to_string(bu_node.ms)
                  << " bu_min=" << bu_node.min_delay
                  << " bu_max=" << bu_node.max_delay
                  << " bu_skew=" << bu_node.skew << "\n";
    }
}

TopDownResult run(const common::Problem& problem,
                  const ClusterDmeInput& input,
                  const common::BottomUpResult& bu_result,
                  const TopDownConfig& config) {
    TopDownResult result;
    if (!problem.valid) {
        result.error_msg = "Cannot run TD on invalid problem: " + problem.error_msg;
        return result;
    }
    if (!input.valid) {
        result.error_msg = "Cannot run TD on invalid cluster DME input: " + input.error_msg;
        return result;
    }
    if (!bu_result.valid) {
        result.error_msg = "Cannot run TD on invalid BU result: " + bu_result.error_msg;
        return result;
    }
    if (!config.has_root_loc) {
        result.error_msg = "TD root loc config is missing";
        return result;
    }
    if (input.nodes.empty()) {
        result.error_msg = "Cannot run TD on empty cluster DME input";
        return result;
    }
    if (input.root_local_id < 0 ||
        static_cast<std::size_t>(input.root_local_id) >= input.nodes.size()) {
        result.error_msg = "Invalid TD cluster root";
        return result;
    }
    if (input.nodes[static_cast<std::size_t>(input.root_local_id)].node_class !=
        common::DmeNodeClass::Access) {
        result.error_msg = "TD root must be an access node";
        return result;
    }
    if (bu_result.node_results.size() != input.nodes.size() ||
        bu_result.cluster_id != input.cluster_id ||
        bu_result.root_local_id != input.root_local_id) {
        result.error_msg = "TD BU result does not match cluster DME input";
        return result;
    }

    result.cluster_id = input.cluster_id;
    result.root_local_id = input.root_local_id;
    result.root_origin_node_id = input.root_origin_node_id;
    result.root = input.root_local_id;
    result.node_results.resize(input.nodes.size());
    result.local_to_origin_node_id.resize(input.nodes.size(), -1);
    for (std::size_t i = 0; i < input.nodes.size(); ++i) {
        const common::ClusterDmeNode& node = input.nodes[i];
        if (node.local_id != static_cast<int>(i) || node.origin_node_id < 0) {
            result.error_msg = "TD cluster node ids are invalid";
            return result;
        }
        std::string err;
        if (!validate_bu_node(bu_result.node_results[i], static_cast<int>(i), err)) {
            result.error_msg = err;
            return result;
        }
        result.local_to_origin_node_id[i] = node.origin_node_id;
    }

    const common::BottomUpNodeResult& root_bu =
        bu_result.node_results[static_cast<std::size_t>(input.root_local_id)];
    if (!point_on_ms(config.root_loc, root_bu.ms)) {
        result.error_msg = "TD root loc is not on root merging segment";
        return result;
    }

    const common::ClusterDmeNode& root_node =
        input.nodes[static_cast<std::size_t>(input.root_local_id)];
    common::TopDownNodeResult root_td;
    root_td.node_id = root_node.origin_node_id;
    root_td.local_id = input.root_local_id;
    root_td.origin_node_id = root_node.origin_node_id;
    root_td.valid = true;
    root_td.loc = config.root_loc;
    root_td.parent_id = -1;
    root_td.parent_local_id = -1;
    root_td.parent_origin_node_id = -1;
    root_td.feasible_ms = root_bu.ms;
    root_td.used_feasible_intersection = true;
    root_td.loc_mode = "ROOT_FROM_CONFIG";
    root_td.candidate_count = 1;
    root_td.min_delay = root_bu.min_delay;
    root_td.max_delay = root_bu.max_delay;
    root_td.skew = root_bu.skew;
    result.node_results[static_cast<std::size_t>(input.root_local_id)] = root_td;

    std::vector<int> state(input.nodes.size(), 0);
    std::string err;
    if (!place_recursive(input.root_local_id, problem, input, bu_result, result,
                         state, err)) {
        result.error_msg = err;
        return result;
    }
    for (std::size_t i = 0; i < state.size(); ++i) {
        if (state[i] != 2) {
            result.error_msg = "TD local node " + std::to_string(i) +
                               " was not visited";
            return result;
        }
        const common::TopDownNodeResult& node = result.node_results[i];
        if (!node.valid || !is_finite_point(node.loc)) {
            result.error_msg = "TD node result invalid at local node " +
                               std::to_string(i);
            return result;
        }
        if (!point_on_ms(node.loc, bu_result.node_results[i].ms)) {
            result.error_msg = "TD loc is not on BU merging segment at local node " +
                               std::to_string(i);
            return result;
        }
        if (node.min_delay > node.max_delay + EPS || node.skew < -EPS) {
            result.error_msg = "TD delay fields invalid at local node " +
                               std::to_string(i);
            return result;
        }
    }
    if (!root_loc_matches_config(result.node_results[static_cast<std::size_t>(input.root_local_id)],
                                 config)) {
        result.error_msg = "TD root loc changed during traversal";
        return result;
    }

    result.valid = true;
    if (g_debug_enabled) {
        debug_output(result, problem, input, bu_result, config);
    }
    return result;
}

}  // namespace td
