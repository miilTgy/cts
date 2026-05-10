#include "bu.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace bu {
namespace {

bool g_debug_enabled = false;

struct TRR {
    double u_min = 0.0;
    double u_max = 0.0;
    double v_min = 0.0;
    double v_max = 0.0;
    bool valid = false;
};

struct Edge {
    bool u_fixed = true;
    double fixed = 0.0;
    double lo = 0.0;
    double hi = 0.0;
};

struct SegmentCandidate {
    common::MergingSegment ms;
    double length = 0.0;
    double score = 0.0;
};

static constexpr double EPS = 1e-9;
static constexpr double INF = std::numeric_limits<double>::infinity();

static double to_u(double x, double y) {
    return x + y;
}

static double to_v(double x, double y) {
    return x - y;
}

static common::SegmentPoint from_uv(double u, double v) {
    common::SegmentPoint p;
    p.x = (u + v) / 2.0;
    p.y = (u - v) / 2.0;
    return p;
}

static common::MergingSegment point_segment(double x, double y) {
    common::MergingSegment ms;
    ms.p1.x = x;
    ms.p1.y = y;
    ms.p2 = ms.p1;
    ms.valid = true;
    return ms;
}

static double clamp_double(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

static bool approx_le(double a, double b) {
    return a <= b + EPS;
}

static bool in_range(double x, double lo, double hi) {
    return approx_le(lo, x) && approx_le(x, hi);
}

static TRR invalid_trr() {
    return TRR{};
}

static common::MergingSegment invalid_segment() {
    return common::MergingSegment{};
}

static TRR normalize_trr(double u_min, double u_max, double v_min, double v_max) {
    TRR trr;
    trr.u_min = std::min(u_min, u_max);
    trr.u_max = std::max(u_min, u_max);
    trr.v_min = std::min(v_min, v_max);
    trr.v_max = std::max(v_min, v_max);
    trr.valid = approx_le(trr.u_min, trr.u_max) && approx_le(trr.v_min, trr.v_max);
    return trr;
}

static TRR segment_to_trr(const common::MergingSegment& ms) {
    if (!ms.valid) {
        return invalid_trr();
    }
    const double u1 = to_u(ms.p1.x, ms.p1.y);
    const double v1 = to_v(ms.p1.x, ms.p1.y);
    const double u2 = to_u(ms.p2.x, ms.p2.y);
    const double v2 = to_v(ms.p2.x, ms.p2.y);
    return normalize_trr(u1, u2, v1, v2);
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
    TRR out;
    out.u_min = std::max(a.u_min, b.u_min);
    out.u_max = std::min(a.u_max, b.u_max);
    out.v_min = std::max(a.v_min, b.v_min);
    out.v_max = std::min(a.v_max, b.v_max);
    out.valid = approx_le(out.u_min, out.u_max) && approx_le(out.v_min, out.v_max);
    return out.valid ? out : invalid_trr();
}

static bool is_valid_ms_segment(const common::MergingSegment& ms) {
    if (!ms.valid) {
        return false;
    }
    if (!std::isfinite(ms.p1.x) || !std::isfinite(ms.p1.y) ||
        !std::isfinite(ms.p2.x) || !std::isfinite(ms.p2.y)) {
        return false;
    }
    const double du = std::abs(to_u(ms.p1.x, ms.p1.y) - to_u(ms.p2.x, ms.p2.y));
    const double dv = std::abs(to_v(ms.p1.x, ms.p1.y) - to_v(ms.p2.x, ms.p2.y));
    return du <= EPS || dv <= EPS;
}

static double distance_point_to_trr_in_uv(double u, double v, const TRR& base) {
    if (!base.valid) {
        return INF;
    }
    double du = 0.0;
    if (u < base.u_min) {
        du = base.u_min - u;
    } else if (u > base.u_max) {
        du = u - base.u_max;
    }

    double dv = 0.0;
    if (v < base.v_min) {
        dv = base.v_min - v;
    } else if (v > base.v_max) {
        dv = v - base.v_max;
    }
    return std::max(du, dv);
}

static double min_distance_between_base_regions(const TRR& a, const TRR& b) {
    if (!a.valid || !b.valid) {
        return INF;
    }
    double du = 0.0;
    if (a.u_max < b.u_min) {
        du = b.u_min - a.u_max;
    } else if (b.u_max < a.u_min) {
        du = a.u_min - b.u_max;
    }

    double dv = 0.0;
    if (a.v_max < b.v_min) {
        dv = b.v_min - a.v_max;
    } else if (b.v_max < a.v_min) {
        dv = a.v_min - b.v_max;
    }
    return std::max(du, dv);
}

static double min_distance_between_ms(const common::MergingSegment& a,
                                      const common::MergingSegment& b) {
    if (!is_valid_ms_segment(a) || !is_valid_ms_segment(b)) {
        return INF;
    }
    return min_distance_between_base_regions(segment_to_trr(a), segment_to_trr(b));
}

static double min_distance_between_segment_and_base(
    const common::MergingSegment& ms,
    const TRR& base) {
    if (!is_valid_ms_segment(ms) || !base.valid) {
        return INF;
    }
    return min_distance_between_base_regions(segment_to_trr(ms), base);
}

static common::MergingSegment segment_from_uv(double u1, double v1,
                                              double u2, double v2) {
    common::MergingSegment ms;
    ms.p1 = from_uv(u1, v1);
    ms.p2 = from_uv(u2, v2);
    ms.valid = true;
    return ms;
}

static common::MergingSegment segment_from_edge(const Edge& edge) {
    if (edge.u_fixed) {
        return segment_from_uv(edge.fixed, edge.lo, edge.fixed, edge.hi);
    }
    return segment_from_uv(edge.lo, edge.fixed, edge.hi, edge.fixed);
}

static std::vector<Edge> trr_edges(const TRR& trr) {
    std::vector<Edge> edges;
    if (!trr.valid) {
        return edges;
    }
    edges.push_back(Edge{true, trr.u_min, trr.v_min, trr.v_max});
    edges.push_back(Edge{true, trr.u_max, trr.v_min, trr.v_max});
    edges.push_back(Edge{false, trr.v_min, trr.u_min, trr.u_max});
    edges.push_back(Edge{false, trr.v_max, trr.u_min, trr.u_max});
    return edges;
}

static bool segment_within_trr(const common::MergingSegment& ms, const TRR& trr) {
    if (!is_valid_ms_segment(ms) || !trr.valid) {
        return false;
    }
    const double u1 = to_u(ms.p1.x, ms.p1.y);
    const double v1 = to_v(ms.p1.x, ms.p1.y);
    const double u2 = to_u(ms.p2.x, ms.p2.y);
    const double v2 = to_v(ms.p2.x, ms.p2.y);
    return in_range(u1, trr.u_min, trr.u_max) &&
           in_range(u2, trr.u_min, trr.u_max) &&
           in_range(v1, trr.v_min, trr.v_max) &&
           in_range(v2, trr.v_min, trr.v_max);
}

static double segment_length_uv(const common::MergingSegment& ms) {
    const double du = std::abs(to_u(ms.p1.x, ms.p1.y) - to_u(ms.p2.x, ms.p2.y));
    const double dv = std::abs(to_v(ms.p1.x, ms.p1.y) - to_v(ms.p2.x, ms.p2.y));
    return std::max(du, dv);
}

static void segment_midpoint_uv(const common::MergingSegment& ms,
                                double& mid_u,
                                double& mid_v) {
    const double u1 = to_u(ms.p1.x, ms.p1.y);
    const double v1 = to_v(ms.p1.x, ms.p1.y);
    const double u2 = to_u(ms.p2.x, ms.p2.y);
    const double v2 = to_v(ms.p2.x, ms.p2.y);
    mid_u = (u1 + u2) / 2.0;
    mid_v = (v1 + v2) / 2.0;
}

static double boundary_score(const common::MergingSegment& ms,
                             const TRR& left_base,
                             const TRR& right_base,
                             double rL,
                             double rR) {
    double mid_u = 0.0;
    double mid_v = 0.0;
    segment_midpoint_uv(ms, mid_u, mid_v);
    const double dl = distance_point_to_trr_in_uv(mid_u, mid_v, left_base);
    const double dr = distance_point_to_trr_in_uv(mid_u, mid_v, right_base);
    return std::abs(dl - rL) + std::abs(dr - rR);
}

static void add_candidate(std::vector<SegmentCandidate>& candidates,
                          const common::MergingSegment& ms,
                          const TRR& inter,
                          const TRR& left_base,
                          const TRR& right_base,
                          double rL,
                          double rR) {
    if (!is_valid_ms_segment(ms) || !segment_within_trr(ms, inter)) {
        return;
    }
    SegmentCandidate cand;
    cand.ms = ms;
    cand.length = segment_length_uv(ms);
    cand.score = boundary_score(ms, left_base, right_base, rL, rR);
    candidates.push_back(cand);
}

static common::MergingSegment intersect_edges(const Edge& a, const Edge& b) {
    if (a.u_fixed == b.u_fixed) {
        if (std::abs(a.fixed - b.fixed) > EPS) {
            return invalid_segment();
        }
        const double lo = std::max(a.lo, b.lo);
        const double hi = std::min(a.hi, b.hi);
        if (!approx_le(lo, hi)) {
            return invalid_segment();
        }
        if (a.u_fixed) {
            return segment_from_uv(a.fixed, lo, a.fixed, hi);
        }
        return segment_from_uv(lo, a.fixed, hi, a.fixed);
    }

    const Edge& u_edge = a.u_fixed ? a : b;
    const Edge& v_edge = a.u_fixed ? b : a;
    const double u = u_edge.fixed;
    const double v = v_edge.fixed;
    if (!in_range(v, u_edge.lo, u_edge.hi) ||
        !in_range(u, v_edge.lo, v_edge.hi)) {
        return invalid_segment();
    }
    return segment_from_uv(u, v, u, v);
}

static common::MergingSegment choose_boundary_intersection(
    const std::vector<SegmentCandidate>& candidates) {
    if (candidates.empty()) {
        return invalid_segment();
    }
    return std::max_element(
               candidates.begin(),
               candidates.end(),
               [](const SegmentCandidate& a, const SegmentCandidate& b) {
                   if (std::abs(a.length - b.length) > EPS) {
                       return a.length < b.length;
                   }
                   return a.score > b.score;
               })
        ->ms;
}

static common::MergingSegment choose_inter_boundary(
    const std::vector<SegmentCandidate>& candidates) {
    if (candidates.empty()) {
        return invalid_segment();
    }
    return std::min_element(
               candidates.begin(),
               candidates.end(),
               [](const SegmentCandidate& a, const SegmentCandidate& b) {
                   if (std::abs(a.score - b.score) > EPS) {
                       return a.score < b.score;
                   }
                   return a.length > b.length;
               })
        ->ms;
}

static common::MergingSegment strict_dme_segment_from_intersection(
    const TRR& inter,
    const TRR& left_base,
    const TRR& right_base,
    double rL,
    double rR,
    std::string& extraction_mode) {
    if (!inter.valid) {
        return invalid_segment();
    }

    const TRR expanded_left = expand_trr(left_base, rL);
    const TRR expanded_right = expand_trr(right_base, rR);
    std::vector<SegmentCandidate> double_boundary_candidates;
    for (const Edge& le : trr_edges(expanded_left)) {
        for (const Edge& re : trr_edges(expanded_right)) {
            add_candidate(double_boundary_candidates,
                          intersect_edges(le, re),
                          inter,
                          left_base,
                          right_base,
                          rL,
                          rR);
        }
    }
    common::MergingSegment selected =
        choose_boundary_intersection(double_boundary_candidates);
    if (selected.valid) {
        extraction_mode = "BOUNDARY_INTERSECTION";
        return selected;
    }

    std::vector<SegmentCandidate> inter_boundary_candidates;
    for (const Edge& edge : trr_edges(inter)) {
        add_candidate(inter_boundary_candidates,
                      segment_from_edge(edge),
                      inter,
                      left_base,
                      right_base,
                      rL,
                      rR);
    }
    selected = choose_inter_boundary(inter_boundary_candidates);
    if (selected.valid) {
        extraction_mode = "INTER_BOUNDARY_FALLBACK";
        return selected;
    }

    if (std::abs(inter.u_max - inter.u_min) <= EPS &&
        std::abs(inter.v_max - inter.v_min) <= EPS) {
        extraction_mode = "INTER_BOUNDARY_FALLBACK";
        return segment_from_uv(inter.u_min, inter.v_min, inter.u_min, inter.v_min);
    }

    if ((inter.u_max - inter.u_min) >= (inter.v_max - inter.v_min)) {
        const double v_mid = (inter.v_min + inter.v_max) / 2.0;
        selected = segment_from_uv(inter.u_min, v_mid, inter.u_max, v_mid);
    } else {
        const double u_mid = (inter.u_min + inter.u_max) / 2.0;
        selected = segment_from_uv(u_mid, inter.v_min, u_mid, inter.v_max);
    }
    extraction_mode = "INTERIOR_FALLBACK";
    return selected;
}

static std::string trr_to_string(const TRR& trr) {
    if (!trr.valid) {
        return "INVALID";
    }
    std::ostringstream oss;
    oss << "[u=" << trr.u_min << ".." << trr.u_max
        << ",v=" << trr.v_min << ".." << trr.v_max << "]";
    return oss.str();
}

static bool calc_merge_candidate(const common::MergingSegment& left_ms,
                                 const common::MergingSegment& right_ms,
                                 double left_min_delay,
                                 double left_max_delay,
                                 double left_buffer_delay,
                                 double right_min_delay,
                                 double right_max_delay,
                                 double right_buffer_delay,
                                 common::MergingSegment& candidate_ms,
                                 double& edge_to_left,
                                 double& edge_to_right,
                                 double& detour_to_left,
                                 double& detour_to_right,
                                 std::string& extraction_mode,
                                 std::string& err) {
    if (!is_valid_ms_segment(left_ms) || !is_valid_ms_segment(right_ms)) {
        err = "Invalid child merging segment";
        return false;
    }
    const TRR base_left = segment_to_trr(left_ms);
    const TRR base_right = segment_to_trr(right_ms);
    const double l_mid = (left_min_delay + left_max_delay) / 2.0 + left_buffer_delay;
    const double r_mid = (right_min_delay + right_max_delay) / 2.0 + right_buffer_delay;
    const double d = min_distance_between_ms(left_ms, right_ms);
    if (!std::isfinite(d)) {
        err = "Cannot compute child merging segment distance";
        return false;
    }

    const double raw_left = (d + r_mid - l_mid) / 2.0;
    const double initial_left = clamp_double(raw_left, 0.0, d);
    const double initial_right = d - initial_left;
    const double repair_limit = std::max(1.0, d + std::abs(l_mid - r_mid) + 10.0);

    for (double extra = 0.0;; extra = (extra == 0.0) ? 1.0 : extra * 2.0) {
        if (extra > repair_limit + EPS) {
            break;
        }
        const double rL = initial_left + extra;
        const double rR = initial_right + extra;
        const TRR expanded_left = expand_trr(base_left, rL);
        const TRR expanded_right = expand_trr(base_right, rR);
        const TRR inter = intersect_trr(expanded_left, expanded_right);
        if (!inter.valid) {
            continue;
        }

        std::string mode;
        common::MergingSegment ms =
            strict_dme_segment_from_intersection(inter, base_left, base_right,
                                                 rL, rR, mode);
        if (!ms.valid || !is_valid_ms_segment(ms)) {
            continue;
        }

        const double geo_left = min_distance_between_segment_and_base(ms, base_left);
        const double geo_right = min_distance_between_segment_and_base(ms, base_right);
        if (!std::isfinite(geo_left) || !std::isfinite(geo_right)) {
            continue;
        }
        candidate_ms = ms;
        edge_to_left = rL;
        edge_to_right = rR;
        detour_to_left = std::max(0.0, edge_to_left - geo_left);
        detour_to_right = std::max(0.0, edge_to_right - geo_right);
        extraction_mode = mode;

        if (g_debug_enabled) {
            std::cout << "[BU_CALC] base_left_trr=" << trr_to_string(base_left)
                      << " base_right_trr=" << trr_to_string(base_right)
                      << " expanded_left_trr=" << trr_to_string(expanded_left)
                      << " expanded_right_trr=" << trr_to_string(expanded_right)
                      << " intersection_trr=" << trr_to_string(inter)
                      << " selected_segment=[(" << ms.p1.x << "," << ms.p1.y
                      << "),(" << ms.p2.x << "," << ms.p2.y << ")]"
                      << " extraction_mode=" << extraction_mode
                      << " detour_left=" << detour_to_left
                      << " detour_right=" << detour_to_right << "\n";
        }
        return true;
    }

    err = "Failed to find a valid TRR merge candidate";
    return false;
}

static int buffer_delay(const common::Problem& problem,
                        const common::BufferChoice& choice) {
    if (!choice.has_buffer) {
        return 0;
    }
    return problem.buffer_types[static_cast<std::size_t>(choice.buffer_type_index)].delay;
}

static int buffer_cost(const common::Problem& problem,
                       const common::BufferChoice& choice) {
    if (!choice.has_buffer) {
        return 0;
    }
    return problem.buffer_types[static_cast<std::size_t>(choice.buffer_type_index)].cost;
}

static std::vector<common::BufferChoice> buffer_options(
    const common::Problem& problem,
    const common::TopoNode& child) {
    std::vector<common::BufferChoice> options;
    options.push_back(common::BufferChoice{});
    if (child.is_sink) {
        return options;
    }
    for (std::size_t i = 0; i < problem.buffer_types.size(); ++i) {
        const common::BufferType& buf = problem.buffer_types[i];
        if (buf.max_fanout >= static_cast<int>(child.sink_indices.size())) {
            common::BufferChoice choice;
            choice.has_buffer = true;
            choice.buffer_type_index = static_cast<int>(i);
            options.push_back(choice);
        }
    }
    return options;
}

static bool validate_node_result(const common::BottomUpNodeResult& node,
                                 std::string& err) {
    if (!node.valid) {
        err = "Node result is invalid";
        return false;
    }
    if (!is_valid_ms_segment(node.ms)) {
        err = "Node result has invalid merging segment";
        return false;
    }
    if (node.min_delay > node.max_delay + EPS) {
        err = "Node result has min_delay greater than max_delay";
        return false;
    }
    if (node.skew < -EPS) {
        err = "Node result has negative skew";
        return false;
    }
    if (node.wire_est < -EPS) {
        err = "Node result has negative wire estimate";
        return false;
    }
    if (node.buffer_cost < 0) {
        err = "Node result has negative buffer cost";
        return false;
    }
    if (node.detour_to_left < -EPS || node.detour_to_right < -EPS) {
        err = "Node result has negative detour";
        return false;
    }
    return true;
}

static bool solve_leaf(int node_id,
                       const common::Problem& problem,
                       const common::TopoTree& tree,
                       common::BottomUpResult& result,
                       std::string& err) {
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    if (node.sink_index < 0 ||
        static_cast<std::size_t>(node.sink_index) >= problem.sinks.size()) {
        err = "Leaf node has invalid sink index";
        return false;
    }
    const common::Sink& sink = problem.sinks[static_cast<std::size_t>(node.sink_index)];

    common::BottomUpNodeResult out;
    out.node_id = node_id;
    out.valid = true;
    out.ms = point_segment(sink.loc.x, sink.loc.y);
    out.extraction_mode = "LEAF";
    result.node_results[static_cast<std::size_t>(node_id)] = out;
    return validate_node_result(out, err);
}

static bool solve_internal(int node_id,
                           const common::Problem& problem,
                           const common::TopoTree& tree,
                           common::BottomUpResult& result,
                           std::string& err) {
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    const int left_id = node.left;
    const int right_id = node.right;
    if (left_id < 0 || right_id < 0 || left_id == right_id ||
        static_cast<std::size_t>(left_id) >= tree.nodes.size() ||
        static_cast<std::size_t>(right_id) >= tree.nodes.size()) {
        err = "Internal node has invalid children";
        return false;
    }

    const common::BottomUpNodeResult& left =
        result.node_results[static_cast<std::size_t>(left_id)];
    const common::BottomUpNodeResult& right =
        result.node_results[static_cast<std::size_t>(right_id)];
    if (!left.valid || !right.valid) {
        err = "Internal node has invalid child result";
        return false;
    }

    const std::vector<common::BufferChoice> left_options =
        buffer_options(problem, tree.nodes[static_cast<std::size_t>(left_id)]);
    const std::vector<common::BufferChoice> right_options =
        buffer_options(problem, tree.nodes[static_cast<std::size_t>(right_id)]);

    double best_total_cost = INF;
    common::BottomUpNodeResult best;
    std::string last_candidate_error;

    for (const common::BufferChoice& bl : left_options) {
        for (const common::BufferChoice& br : right_options) {
            common::MergingSegment candidate_ms;
            double edge_to_left = 0.0;
            double edge_to_right = 0.0;
            double detour_to_left = 0.0;
            double detour_to_right = 0.0;
            std::string extraction_mode;
            std::string candidate_error;

            const double l_buf_delay = buffer_delay(problem, bl);
            const double r_buf_delay = buffer_delay(problem, br);
            const bool ok = calc_merge_candidate(left.ms, right.ms,
                                                 left.min_delay, left.max_delay, l_buf_delay,
                                                 right.min_delay, right.max_delay, r_buf_delay,
                                                 candidate_ms,
                                                 edge_to_left, edge_to_right,
                                                 detour_to_left, detour_to_right,
                                                 extraction_mode,
                                                 candidate_error);
            if (!ok) {
                last_candidate_error = candidate_error;
                continue;
            }

            const double l_min = left.min_delay + edge_to_left + l_buf_delay;
            const double l_max = left.max_delay + edge_to_left + l_buf_delay;
            const double r_min = right.min_delay + edge_to_right + r_buf_delay;
            const double r_max = right.max_delay + edge_to_right + r_buf_delay;

            common::BottomUpNodeResult candidate;
            candidate.node_id = node_id;
            candidate.valid = true;
            candidate.ms = candidate_ms;
            candidate.edge_to_left = edge_to_left;
            candidate.edge_to_right = edge_to_right;
            candidate.buffer_at_left_child = bl;
            candidate.buffer_at_right_child = br;
            candidate.min_delay = std::min(l_min, r_min);
            candidate.max_delay = std::max(l_max, r_max);
            candidate.skew = std::max(0.0, candidate.max_delay - candidate.min_delay);
            candidate.wire_est = left.wire_est + right.wire_est +
                                  edge_to_left + edge_to_right;
            candidate.buffer_cost = left.buffer_cost + right.buffer_cost +
                                    buffer_cost(problem, bl) + buffer_cost(problem, br);
            candidate.total_cost = 5000.0 * candidate.skew +
                                   50.0 * candidate.wire_est +
                                   200.0 * candidate.buffer_cost;
            candidate.extraction_mode = extraction_mode;
            candidate.detour_to_left = detour_to_left;
            candidate.detour_to_right = detour_to_right;

            std::string validation_error;
            if (!validate_node_result(candidate, validation_error)) {
                last_candidate_error = validation_error;
                continue;
            }
            if (candidate.total_cost < best_total_cost) {
                best_total_cost = candidate.total_cost;
                best = candidate;
            }
        }
    }

    if (!std::isfinite(best_total_cost)) {
        err = "No legal merge candidate for node " + std::to_string(node_id);
        if (!last_candidate_error.empty()) {
            err += ": " + last_candidate_error;
        }
        return false;
    }

    result.node_results[static_cast<std::size_t>(node_id)] = best;
    return true;
}

static bool solve_node(int node_id,
                       const common::Problem& problem,
                       const common::TopoTree& tree,
                       common::BottomUpResult& result,
                       std::vector<int>& state,
                       std::string& err) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) {
        err = "Tree node id out of range";
        return false;
    }
    const std::size_t idx = static_cast<std::size_t>(node_id);
    if (state[idx] == 1) {
        err = "Cycle detected in topology tree";
        return false;
    }
    if (state[idx] == 2) {
        return true;
    }

    state[idx] = 1;
    const common::TopoNode& node = tree.nodes[idx];
    bool ok = false;
    if (node.is_sink) {
        ok = solve_leaf(node_id, problem, tree, result, err);
    } else {
        if (node.left < 0 || node.right < 0 || node.left == node.right ||
            static_cast<std::size_t>(node.left) >= tree.nodes.size() ||
            static_cast<std::size_t>(node.right) >= tree.nodes.size()) {
            err = "Internal node has invalid children";
            return false;
        }
        ok = solve_node(node.left, problem, tree, result, state, err) &&
             solve_node(node.right, problem, tree, result, state, err) &&
             solve_internal(node_id, problem, tree, result, err);
    }
    if (!ok) {
        return false;
    }
    state[idx] = 2;
    return true;
}

static std::string buffer_choice_to_string(const common::BufferChoice& choice,
                                           const common::Problem& problem) {
    if (!choice.has_buffer) {
        return "NONE";
    }
    if (choice.buffer_type_index < 0 ||
        static_cast<std::size_t>(choice.buffer_type_index) >= problem.buffer_types.size()) {
        return "INVALID";
    }
    const common::BufferType& buf =
        problem.buffer_types[static_cast<std::size_t>(choice.buffer_type_index)];
    std::ostringstream oss;
    oss << buf.name << "(delay=" << buf.delay
        << ", fanout=" << buf.max_fanout
        << ", cost=" << buf.cost << ")";
    return oss.str();
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_output(const BottomUpResult& result,
                  const common::Problem& problem,
                  const common::TopoTree& tree) {
    if (!g_debug_enabled) {
        return;
    }

    std::cout << "[BU] valid=" << (result.valid ? 1 : 0)
              << " error_msg=" << result.error_msg
              << " root=" << result.root
              << " num_node_results=" << result.node_results.size() << "\n";
    for (std::size_t i = 0; i < result.node_results.size(); ++i) {
        const common::BottomUpNodeResult& node_result = result.node_results[i];
        if (i >= tree.nodes.size()) {
            continue;
        }
        const common::TopoNode& node = tree.nodes[i];
        std::cout << "[BU] node_id=" << node.id
                  << " parent=" << node.parent
                  << " left=" << node.left
                  << " right=" << node.right
                  << " is_leaf=" << (node.is_sink ? 1 : 0)
                  << " sink_index=" << node.sink_index
                  << " sink_count=" << node.sink_indices.size() << "\n";
        if (node.is_sink &&
            node.sink_index >= 0 &&
            static_cast<std::size_t>(node.sink_index) < problem.sinks.size()) {
            const common::Sink& sink =
                problem.sinks[static_cast<std::size_t>(node.sink_index)];
            std::cout << "[BU]   sink=" << sink.id
                      << " coord=(" << sink.loc.x << "," << sink.loc.y << ")\n";
        }
        std::cout << "[BU]   ms=[(" << node_result.ms.p1.x << ","
                  << node_result.ms.p1.y << "),("
                  << node_result.ms.p2.x << ","
                  << node_result.ms.p2.y << ")]"
                  << " edge_left=" << node_result.edge_to_left
                  << " edge_right=" << node_result.edge_to_right
                  << " detour_left=" << node_result.detour_to_left
                  << " detour_right=" << node_result.detour_to_right << "\n";
        std::cout << "[BU]   extraction_mode=" << node_result.extraction_mode << "\n";
        std::cout << "[BU]   buf_left="
                  << buffer_choice_to_string(node_result.buffer_at_left_child, problem)
                  << " buf_right="
                  << buffer_choice_to_string(node_result.buffer_at_right_child, problem)
                  << "\n";
        std::cout << "[BU]   min_delay=" << node_result.min_delay
                  << " max_delay=" << node_result.max_delay
                  << " skew=" << node_result.skew
                  << " wire_est=" << node_result.wire_est
                  << " buffer_cost=" << node_result.buffer_cost
                  << " total_cost=" << node_result.total_cost << "\n";
    }
}

BottomUpResult run(const common::Problem& problem,
                   const common::TopoTree& tree) {
    BottomUpResult result;
    if (!problem.valid) {
        result.error_msg = "Cannot run BU on invalid problem: " + problem.error_msg;
        return result;
    }
    if (!tree.valid) {
        result.error_msg = "Cannot run BU on invalid tree: " + tree.error_msg;
        return result;
    }
    if (tree.nodes.empty()) {
        result.error_msg = "Cannot run BU on empty tree";
        return result;
    }
    if (tree.root < 0 || static_cast<std::size_t>(tree.root) >= tree.nodes.size()) {
        result.error_msg = "Invalid tree root";
        return result;
    }

    result.node_results.resize(tree.nodes.size());
    std::vector<int> state(tree.nodes.size(), 0);
    std::string err;
    if (!solve_node(tree.root, problem, tree, result, state, err)) {
        result.valid = false;
        result.error_msg = err;
        return result;
    }
    const common::BottomUpNodeResult& root_result =
        result.node_results[static_cast<std::size_t>(tree.root)];
    if (!root_result.valid) {
        result.error_msg = "Root BU result is invalid";
        return result;
    }

    result.root = tree.root;
    result.valid = true;
    if (g_debug_enabled) {
        debug_output(result, problem, tree);
    }
    return result;
}

}  // namespace bu
