#include "td.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace td {
namespace {

bool g_debug_enabled = false;

struct TRR {
    double u_min = 0.0;
    double u_max = 0.0;
    double v_min = 0.0;
    double v_max = 0.0;
    bool valid = false;
};

struct RouteCandidate {
    bool valid = false;
    common::SegmentPoint loc;
    std::vector<common::SegmentPoint> path;
    double geo = 0.0;
    double routed_len = 0.0;
    double cost = 0.0;
};

struct BranchCandidate {
    int child_id = -1;
    bool is_left_child = false;
    int allowed_parent_sink = -1;
    int allowed_child_sink = -1;
    double assigned_edge = 0.0;
    RouteCandidate route;
    double route_excess = 0.0;
    double compensation = 0.0;
    double final_len = 0.0;
    common::MergingSegment feasible_ms;
    common::BufferChoice buffer_choice;
};

static constexpr double EPS = 1e-9;
static constexpr double SNAP_EPS = 1e-6;
static constexpr double DELAY_EPS = 1e-6;
static constexpr double INF = 1e100;
static constexpr int MAX_CANDIDATES = 32;
static constexpr double LSHAPE_EXCESS_COST = 10000.0;
static constexpr double LSHAPE_BEND_COST = 100.0;
static constexpr double LSHAPE_LEN_COST = 1.0;
static constexpr double LSHAPE_DIST_COST = 0.001;

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

static bool near(double a, double b) {
    return std::abs(a - b) <= SNAP_EPS;
}

static bool same_point(const common::SegmentPoint& a,
                       const common::SegmentPoint& b) {
    return near(a.x, b.x) && near(a.y, b.y);
}

static bool approx_le(double a, double b) {
    return a <= b + EPS;
}

static bool is_finite_point(const common::SegmentPoint& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::abs(p.x) < INF / 2.0 && std::abs(p.y) < INF / 2.0;
}

static common::MergingSegment invalid_segment() {
    return common::MergingSegment{};
}

static TRR invalid_trr() {
    return TRR{};
}

static common::SegmentPoint invalid_point() {
    return common::SegmentPoint{INF, INF};
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

static TRR segment_to_trr(const common::MergingSegment& ms) {
    if (!ms.valid) {
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

static bool is_valid_ms_segment(const common::MergingSegment& ms) {
    if (!ms.valid || !is_finite_point(ms.p1) || !is_finite_point(ms.p2)) {
        return false;
    }
    const double du = std::abs(to_u(ms.p1.x, ms.p1.y) - to_u(ms.p2.x, ms.p2.y));
    const double dv = std::abs(to_v(ms.p1.x, ms.p1.y) - to_v(ms.p2.x, ms.p2.y));
    return du <= SNAP_EPS || dv <= SNAP_EPS;
}

static bool point_on_ms(const common::SegmentPoint& p,
                        const common::MergingSegment& ms) {
    if (!is_valid_ms_segment(ms)) {
        return false;
    }
    return point_in_trr(p, segment_to_trr(ms));
}

static double manhattan(const common::SegmentPoint& a,
                        const common::SegmentPoint& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

static common::SegmentPoint midpoint_of_ms(const common::MergingSegment& ms) {
    common::SegmentPoint p;
    p.x = (ms.p1.x + ms.p2.x) / 2.0;
    p.y = (ms.p1.y + ms.p2.y) / 2.0;
    return p;
}

static common::SegmentPoint nearest_point_on_ms_to_point(
    const common::MergingSegment& ms,
    const common::SegmentPoint& p) {
    if (!is_valid_ms_segment(ms)) {
        return invalid_point();
    }
    const TRR base = segment_to_trr(ms);
    const double u = clamp_double(to_u(p.x, p.y), base.u_min, base.u_max);
    const double v = clamp_double(to_v(p.x, p.y), base.v_min, base.v_max);
    return from_uv(u, v);
}

static common::MergingSegment segment_from_uv(double u1, double v1,
                                              double u2, double v2) {
    common::MergingSegment ms;
    ms.p1 = from_uv(u1, v1);
    ms.p2 = from_uv(u2, v2);
    ms.valid = true;
    return ms;
}

static common::MergingSegment trr_to_representative_ms(const TRR& trr) {
    if (!trr.valid) {
        return invalid_segment();
    }
    if (trr.u_max - trr.u_min >= trr.v_max - trr.v_min) {
        const double v_mid = (trr.v_min + trr.v_max) / 2.0;
        return segment_from_uv(trr.u_min, v_mid, trr.u_max, v_mid);
    }
    const double u_mid = (trr.u_min + trr.u_max) / 2.0;
    return segment_from_uv(u_mid, trr.v_min, u_mid, trr.v_max);
}

static std::string ms_to_string(const common::MergingSegment& ms) {
    if (!ms.valid) {
        return "INVALID";
    }
    std::ostringstream oss;
    oss << "[(" << ms.p1.x << "," << ms.p1.y << "),("
        << ms.p2.x << "," << ms.p2.y << ")]";
    return oss.str();
}

static double polyline_length(const std::vector<common::SegmentPoint>& path) {
    double len = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        len += manhattan(path[i - 1], path[i]);
    }
    return len;
}

static int bend_count(const std::vector<common::SegmentPoint>& path) {
    int bends = 0;
    for (std::size_t i = 2; i < path.size(); ++i) {
        const bool prev_horizontal = near(path[i - 2].y, path[i - 1].y);
        const bool curr_horizontal = near(path[i - 1].y, path[i].y);
        const bool prev_vertical = near(path[i - 2].x, path[i - 1].x);
        const bool curr_vertical = near(path[i - 1].x, path[i].x);
        if ((prev_horizontal && curr_vertical) || (prev_vertical && curr_horizontal)) {
            ++bends;
        }
    }
    return bends;
}

static void append_point(std::vector<common::SegmentPoint>& path,
                         const common::SegmentPoint& p) {
    if (path.empty() || !same_point(path.back(), p)) {
        path.push_back(p);
    }
}

static std::vector<common::SegmentPoint> simplify_polyline(
    std::vector<common::SegmentPoint> path) {
    std::vector<common::SegmentPoint> no_dup;
    for (const common::SegmentPoint& p : path) {
        append_point(no_dup, p);
    }

    std::vector<common::SegmentPoint> out;
    for (const common::SegmentPoint& p : no_dup) {
        out.push_back(p);
        while (out.size() >= 3U) {
            const common::SegmentPoint& a = out[out.size() - 3U];
            const common::SegmentPoint& b = out[out.size() - 2U];
            const common::SegmentPoint& c = out[out.size() - 1U];
            if ((near(a.x, b.x) && near(b.x, c.x)) ||
                (near(a.y, b.y) && near(b.y, c.y))) {
                out.erase(out.end() - 2);
            } else {
                break;
            }
        }
    }
    return out;
}

static bool point_equals_sink(const common::SegmentPoint& p,
                              const common::Sink& sink) {
    return near(p.x, sink.loc.x) && near(p.y, sink.loc.y);
}

static bool is_allowed_sink_index(int idx, int allowed_parent_sink, int allowed_child_sink) {
    return idx >= 0 && (idx == allowed_parent_sink || idx == allowed_child_sink);
}

static bool segment_crosses_forbidden_sink(const common::SegmentPoint& a,
                                           const common::SegmentPoint& b,
                                           const std::vector<common::Sink>& sinks,
                                           int allowed_parent_sink,
                                           int allowed_child_sink) {
    const bool horizontal = near(a.y, b.y);
    const bool vertical = near(a.x, b.x);
    if (!horizontal && !vertical) {
        return true;
    }

    for (std::size_t i = 0; i < sinks.size(); ++i) {
        if (is_allowed_sink_index(static_cast<int>(i), allowed_parent_sink, allowed_child_sink)) {
            continue;
        }
        const common::Point& s = sinks[i].loc;
        if (horizontal && near(a.y, s.y)) {
            const double lo = std::min(a.x, b.x) - SNAP_EPS;
            const double hi = std::max(a.x, b.x) + SNAP_EPS;
            if (s.x >= lo && s.x <= hi) {
                return true;
            }
        }
        if (vertical && near(a.x, s.x)) {
            const double lo = std::min(a.y, b.y) - SNAP_EPS;
            const double hi = std::max(a.y, b.y) + SNAP_EPS;
            if (s.y >= lo && s.y <= hi) {
                return true;
            }
        }
    }
    return false;
}

static bool polyline_crosses_forbidden_sink(
    const std::vector<common::SegmentPoint>& path,
    const std::vector<common::Sink>& sinks,
    int allowed_parent_sink,
    int allowed_child_sink) {
    for (const common::SegmentPoint& p : path) {
        for (std::size_t i = 0; i < sinks.size(); ++i) {
            if (!is_allowed_sink_index(static_cast<int>(i), allowed_parent_sink,
                                       allowed_child_sink) &&
                point_equals_sink(p, sinks[i])) {
                return true;
            }
        }
    }
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (segment_crosses_forbidden_sink(path[i - 1], path[i], sinks,
                                           allowed_parent_sink, allowed_child_sink)) {
            return true;
        }
    }
    return false;
}

static bool add_unique_point(std::vector<common::SegmentPoint>& points,
                             const common::SegmentPoint& p) {
    if (!is_finite_point(p)) {
        return false;
    }
    for (const common::SegmentPoint& old : points) {
        if (same_point(old, p)) {
            return false;
        }
    }
    points.push_back(p);
    return true;
}

static void add_candidate_if_valid(std::vector<common::SegmentPoint>& points,
                                   const common::SegmentPoint& p,
                                   const common::MergingSegment& child_ms,
                                   const TRR& feasible) {
    if (!point_on_ms(p, child_ms)) {
        return;
    }
    if (feasible.valid && !point_in_trr(p, feasible)) {
        return;
    }
    add_unique_point(points, p);
}

static void add_x_projection(std::vector<common::SegmentPoint>& points,
                             double x,
                             const common::MergingSegment& child_ms,
                             const TRR& feasible) {
    if (!is_valid_ms_segment(child_ms)) {
        return;
    }
    const TRR base = segment_to_trr(child_ms);
    std::vector<common::SegmentPoint> candidates;
    if (std::abs(base.u_max - base.u_min) <= SNAP_EPS) {
        candidates.push_back(from_uv(base.u_min, 2.0 * x - base.u_min));
    }
    if (std::abs(base.v_max - base.v_min) <= SNAP_EPS) {
        candidates.push_back(from_uv(2.0 * x - base.v_min, base.v_min));
    }
    for (const common::SegmentPoint& p : candidates) {
        add_candidate_if_valid(points, p, child_ms, feasible);
    }
}

static void add_y_projection(std::vector<common::SegmentPoint>& points,
                             double y,
                             const common::MergingSegment& child_ms,
                             const TRR& feasible) {
    if (!is_valid_ms_segment(child_ms)) {
        return;
    }
    const TRR base = segment_to_trr(child_ms);
    std::vector<common::SegmentPoint> candidates;
    if (std::abs(base.u_max - base.u_min) <= SNAP_EPS) {
        candidates.push_back(from_uv(base.u_min, base.u_min - 2.0 * y));
    }
    if (std::abs(base.v_max - base.v_min) <= SNAP_EPS) {
        candidates.push_back(from_uv(base.v_min + 2.0 * y, base.v_min));
    }
    for (const common::SegmentPoint& p : candidates) {
        add_candidate_if_valid(points, p, child_ms, feasible);
    }
}

static std::vector<common::SegmentPoint> generate_candidate_locs(
    const common::MergingSegment& child_ms,
    const TRR& feasible,
    const common::SegmentPoint& parent_loc,
    const std::vector<common::Sink>& sinks) {
    std::vector<common::SegmentPoint> points;
    if (feasible.valid) {
        const common::SegmentPoint mid = midpoint_of_ms(child_ms);
        const common::SegmentPoint clamped = from_uv(
            clamp_double(to_u(mid.x, mid.y), feasible.u_min, feasible.u_max),
            clamp_double(to_v(mid.x, mid.y), feasible.v_min, feasible.v_max));
        add_candidate_if_valid(points, clamped, child_ms, feasible);

        add_candidate_if_valid(points, from_uv(feasible.u_min, feasible.v_min),
                               child_ms, feasible);
        add_candidate_if_valid(points, from_uv(feasible.u_min, feasible.v_max),
                               child_ms, feasible);
        add_candidate_if_valid(points, from_uv(feasible.u_max, feasible.v_min),
                               child_ms, feasible);
        add_candidate_if_valid(points, from_uv(feasible.u_max, feasible.v_max),
                               child_ms, feasible);
        add_candidate_if_valid(points, from_uv((feasible.u_min + feasible.u_max) / 2.0,
                                               feasible.v_min),
                               child_ms, feasible);
        add_candidate_if_valid(points, from_uv((feasible.u_min + feasible.u_max) / 2.0,
                                               feasible.v_max),
                               child_ms, feasible);
        add_candidate_if_valid(points, from_uv(feasible.u_min,
                                               (feasible.v_min + feasible.v_max) / 2.0),
                               child_ms, feasible);
        add_candidate_if_valid(points, from_uv(feasible.u_max,
                                               (feasible.v_min + feasible.v_max) / 2.0),
                               child_ms, feasible);
    } else {
        add_unique_point(points, nearest_point_on_ms_to_point(child_ms, parent_loc));
    }

    for (const common::Sink& sink : sinks) {
        add_x_projection(points, sink.loc.x - 1.0, child_ms, feasible);
        add_x_projection(points, sink.loc.x + 1.0, child_ms, feasible);
        add_y_projection(points, sink.loc.y - 1.0, child_ms, feasible);
        add_y_projection(points, sink.loc.y + 1.0, child_ms, feasible);
    }

    std::sort(points.begin(), points.end(),
              [&](const common::SegmentPoint& a, const common::SegmentPoint& b) {
                  const double da = manhattan(parent_loc, a);
                  const double db = manhattan(parent_loc, b);
                  if (std::abs(da - db) > SNAP_EPS) {
                      return da < db;
                  }
                  if (std::abs(a.x - b.x) > SNAP_EPS) {
                      return a.x < b.x;
                  }
                  return a.y < b.y;
              });
    if (points.size() > MAX_CANDIDATES) {
        points.resize(MAX_CANDIDATES);
    }
    return points;
}

static RouteCandidate route_to_best_candidate_loc_dag(
    const common::SegmentPoint& parent_loc,
    const std::vector<common::SegmentPoint>& candidate_locs,
    const std::vector<common::Sink>& sinks,
    int allowed_parent_sink,
    int allowed_child_sink,
    double assigned_edge,
    std::string& err) {
    RouteCandidate best;
    if (candidate_locs.empty()) {
        err = "TD has no candidate locations for L-shape route";
        return best;
    }

    for (const common::SegmentPoint& T : candidate_locs) {
        std::vector<std::vector<common::SegmentPoint>> paths;

        if (near(parent_loc.x, T.x) || near(parent_loc.y, T.y)) {
            paths.push_back({parent_loc, T});
        } else {
            paths.push_back({parent_loc,
                             common::SegmentPoint{T.x, parent_loc.y},
                             T});
            paths.push_back({parent_loc,
                             common::SegmentPoint{parent_loc.x, T.y},
                             T});
            for (const common::Sink& sink : sinks) {
                if (is_allowed_sink_index(static_cast<int>(&sink - sinks.data()),
                                           allowed_parent_sink, allowed_child_sink))
                    continue;
                paths.push_back({parent_loc,
                                 common::SegmentPoint{sink.loc.x - 1.0, parent_loc.y},
                                 T});
                paths.push_back({parent_loc,
                                 common::SegmentPoint{sink.loc.x + 1.0, parent_loc.y},
                                 T});
                paths.push_back({parent_loc,
                                 common::SegmentPoint{parent_loc.x, sink.loc.y - 1.0},
                                 T});
                paths.push_back({parent_loc,
                                 common::SegmentPoint{parent_loc.x, sink.loc.y + 1.0},
                                 T});
            }
        }

        for (auto& path : paths) {
            path = simplify_polyline(path);
            if (path.size() < 2U) continue;
            if (polyline_crosses_forbidden_sink(path, sinks,
                                                 allowed_parent_sink,
                                                 allowed_child_sink))
                continue;
            const double len = polyline_length(path);
            const double cost =
                LSHAPE_EXCESS_COST * std::max(0.0, len - assigned_edge) +
                LSHAPE_BEND_COST * bend_count(path) +
                LSHAPE_LEN_COST * len +
                LSHAPE_DIST_COST * manhattan(parent_loc, T);
            if (!best.valid || cost < best.cost - SNAP_EPS) {
                best.valid = true;
                best.loc = T;
                best.path = path;
                best.geo = manhattan(parent_loc, T);
                best.routed_len = len;
                best.cost = cost;
            }
        }
    }

    if (!best.valid) {
        // Fallback: undirected visibility graph + Dijkstra on candidate grid
        std::vector<common::SegmentPoint> vertices;
        std::unordered_map<std::string, int> id_by_key;

        auto fallback_add_vertex = [&](const common::SegmentPoint& p) -> int {
            if (!is_finite_point(p)) return -1;
            for (std::size_t i = 0; i < sinks.size(); ++i) {
                if (!is_allowed_sink_index(static_cast<int>(i), allowed_parent_sink,
                                            allowed_child_sink) &&
                    point_equals_sink(p, sinks[i]))
                    return -1;
            }
            auto key = std::to_string(static_cast<long long>(std::llround(p.x * 1e6))) +
                       "," + std::to_string(static_cast<long long>(std::llround(p.y * 1e6)));
            auto it = id_by_key.find(key);
            if (it != id_by_key.end()) return it->second;
            int id = static_cast<int>(vertices.size());
            vertices.push_back(p);
            id_by_key[key] = id;
            return id;
        };

        // Build x/y coords from parent, candidates, and sink ±1 corridors
        std::vector<double> x_coords, y_coords;
        x_coords.push_back(parent_loc.x);
        y_coords.push_back(parent_loc.y);
        for (const auto& loc : candidate_locs) {
            x_coords.push_back(loc.x);
            y_coords.push_back(loc.y);
        }
        for (const common::Sink& sink : sinks) {
            if (is_allowed_sink_index(static_cast<int>(&sink - sinks.data()),
                                       allowed_parent_sink, allowed_child_sink))
                continue;
            x_coords.push_back(sink.loc.x - 1.0);
            x_coords.push_back(sink.loc.x + 1.0);
            y_coords.push_back(sink.loc.y - 1.0);
            y_coords.push_back(sink.loc.y + 1.0);
        }
        // Deduplicate
        auto sort_unique = [](std::vector<double>& values) {
            std::sort(values.begin(), values.end());
            std::vector<double> out;
            for (double v : values) {
                if (out.empty() || !near(out.back(), v)) out.push_back(v);
            }
            values.swap(out);
        };
        sort_unique(x_coords);
        sort_unique(y_coords);

        int start_id = fallback_add_vertex(parent_loc);
        std::vector<int> target_ids;
        for (const auto& loc : candidate_locs) {
            int id = fallback_add_vertex(loc);
            if (id >= 0) target_ids.push_back(id);
        }
        if (start_id >= 0 && !target_ids.empty()) {
            for (double x : x_coords)
                for (double y : y_coords)
                    fallback_add_vertex(common::SegmentPoint{x, y});

            std::vector<std::vector<std::pair<int, double>>> adj(vertices.size());
            long long key_scale = 1000000;
            std::map<long long, std::vector<int>> rows, cols;
            for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
                rows[static_cast<long long>(std::llround(vertices[i].y * key_scale))].push_back(i);
                cols[static_cast<long long>(std::llround(vertices[i].x * key_scale))].push_back(i);
            }
            auto connect_fb = [&](std::vector<int> ids, bool sort_by_x) {
                std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                    return sort_by_x ? vertices[a].x < vertices[b].x
                                     : vertices[a].y < vertices[b].y;
                });
                for (std::size_t i = 1; i < ids.size(); ++i) {
                    int a = ids[i - 1], b = ids[i];
                    if (same_point(vertices[a], vertices[b])) continue;
                    if (segment_crosses_forbidden_sink(vertices[a], vertices[b], sinks,
                                                        allowed_parent_sink, allowed_child_sink))
                        continue;
                    double len = manhattan(vertices[a], vertices[b]);
                    adj[a].push_back({b, len});
                    adj[b].push_back({a, len});
                }
            };
            for (auto& e : rows) connect_fb(e.second, true);
            for (auto& e : cols) connect_fb(e.second, false);

            std::vector<double> fb_dist(vertices.size(), INF);
            std::vector<int> fb_prev(vertices.size(), -1);
            fb_dist[start_id] = 0.0;
            using QI = std::pair<double, int>;
            std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
            pq.push({0.0, start_id});
            while (!pq.empty()) {
                auto item = pq.top(); pq.pop();
                if (item.first > fb_dist[item.second] + SNAP_EPS) continue;
                for (auto& edge : adj[item.second]) {
                    double nd = item.first + edge.second;
                    if (nd + SNAP_EPS < fb_dist[edge.first]) {
                        fb_dist[edge.first] = nd;
                        fb_prev[edge.first] = item.second;
                        pq.push({nd, edge.first});
                    }
                }
            }
            for (int tid : target_ids) {
                if (fb_dist[tid] >= INF / 2.0) continue;
                std::vector<common::SegmentPoint> path;
                for (int cur = tid; cur >= 0; cur = fb_prev[cur]) {
                    path.push_back(vertices[cur]);
                    if (cur == start_id) break;
                }
                if (path.empty() || !same_point(path.back(), parent_loc)) continue;
                std::reverse(path.begin(), path.end());
                path = simplify_polyline(path);
                if (path.size() == 1U && same_point(path.front(), vertices[tid]))
                    path.push_back(vertices[tid]);
                if (polyline_crosses_forbidden_sink(path, sinks, allowed_parent_sink,
                                                     allowed_child_sink))
                    continue;
                double len = polyline_length(path);
                double cost = LSHAPE_EXCESS_COST * std::max(0.0, len - assigned_edge) +
                              LSHAPE_BEND_COST * bend_count(path) +
                              LSHAPE_LEN_COST * len +
                              LSHAPE_DIST_COST * manhattan(parent_loc, vertices[tid]);
                if (!best.valid || cost < best.cost - SNAP_EPS) {
                    best.valid = true;
                    best.loc = vertices[tid];
                    best.path = path;
                    best.geo = manhattan(parent_loc, best.loc);
                    best.routed_len = len;
                    best.cost = cost;
                }
            }
        }

        if (!best.valid) {
            err = "no legal L-shape route from parent to any child candidate";
            return best;
        }
    }
    return best;
}

static std::vector<common::SegmentPoint> make_compensation_snake(
    const std::vector<common::SegmentPoint>& base_route,
    double required_extra,
    const std::vector<common::Sink>& sinks,
    int allowed_parent_sink,
    int allowed_child_sink,
    std::string& err) {
    std::vector<common::SegmentPoint> snake;
    if (required_extra <= SNAP_EPS) {
        return snake;
    }
    const double d = required_extra / 2.0;
    const double offsets[] = {d, -d};
    for (const common::SegmentPoint& anchor : base_route) {
        for (double offset : offsets) {
            {
                std::vector<common::SegmentPoint> trial;
                trial.push_back(anchor);
                trial.push_back(common::SegmentPoint{anchor.x + offset, anchor.y});
                trial.push_back(anchor);
                if (!polyline_crosses_forbidden_sink(trial, sinks, allowed_parent_sink,
                                                      allowed_child_sink)) {
                    return trial;
                }
            }
            {
                std::vector<common::SegmentPoint> trial;
                trial.push_back(anchor);
                trial.push_back(common::SegmentPoint{anchor.x, anchor.y + offset});
                trial.push_back(anchor);
                if (!polyline_crosses_forbidden_sink(trial, sinks, allowed_parent_sink,
                                                      allowed_child_sink)) {
                    return trial;
                }
            }
        }
    }
    err = "TD could not construct a sink-safe compensation snake";
    return snake;
}

static std::vector<common::SegmentPoint> merge_base_route_and_compensation(
    const std::vector<common::SegmentPoint>& base_route,
    const std::vector<common::SegmentPoint>& compensation_snake) {
    if (compensation_snake.empty()) {
        return base_route;
    }

    std::vector<common::SegmentPoint> merged;
    const common::SegmentPoint anchor = compensation_snake.front();
    bool inserted = false;
    if (!base_route.empty()) {
        append_point(merged, base_route.front());
    }
    for (std::size_t i = 1; i < base_route.size(); ++i) {
        const common::SegmentPoint& a = base_route[i - 1U];
        const common::SegmentPoint& b = base_route[i];
        if (!inserted && same_point(a, anchor)) {
            for (std::size_t j = 1; j < compensation_snake.size(); ++j) {
                append_point(merged, compensation_snake[j]);
            }
            inserted = true;
            append_point(merged, b);
        } else {
            append_point(merged, b);
        }
    }
    return inserted ? merged : base_route;
}

static bool is_valid_buffer_choice(const common::BufferChoice& choice,
                                   const common::Problem& problem,
                                   const common::TopoNode& child,
                                   std::string& err) {
    if (!choice.has_buffer) {
        if (choice.buffer_type_index != -1) {
            err = "Buffer choice without buffer has non--1 type index";
            return false;
        }
        return true;
    }
    if (child.is_sink) {
        err = "Leaf node has a buffer marker from BU";
        return false;
    }
    if (choice.buffer_type_index < 0 ||
        static_cast<std::size_t>(choice.buffer_type_index) >= problem.buffer_types.size()) {
        err = "Buffer choice has invalid buffer type index";
        return false;
    }
    return true;
}

static double buffer_delay(const common::Problem& problem,
                           const common::BufferChoice& choice) {
    if (!choice.has_buffer) {
        return 0.0;
    }
    return problem.buffer_types[static_cast<std::size_t>(choice.buffer_type_index)].delay;
}

static std::string buffer_marker_to_string(bool has_buffer,
                                           int buffer_type_index,
                                           const common::Problem& problem) {
    if (!has_buffer) {
        return "NONE";
    }
    if (buffer_type_index < 0 ||
        static_cast<std::size_t>(buffer_type_index) >= problem.buffer_types.size()) {
        return "INVALID";
    }
    const common::BufferType& buf =
        problem.buffer_types[static_cast<std::size_t>(buffer_type_index)];
    std::ostringstream oss;
    oss << buf.name << "(delay=" << buf.delay
        << ", fanout=" << buf.max_fanout
        << ", cost=" << buf.cost << ")";
    return oss.str();
}

static bool validate_bu_node(const common::BottomUpNodeResult& node_result,
                             int expected_node_id,
                             std::string& err) {
    if (!node_result.valid) {
        err = "BU result for node " + std::to_string(expected_node_id) + " is invalid";
        return false;
    }
    if (node_result.node_id != expected_node_id) {
        err = "BU result node_id mismatch for node " + std::to_string(expected_node_id);
        return false;
    }
    if (!is_valid_ms_segment(node_result.ms)) {
        err = "BU result for node " + std::to_string(expected_node_id) +
              " has invalid merging segment";
        return false;
    }
    if (node_result.min_delay > node_result.max_delay + EPS) {
        err = "BU result for node " + std::to_string(expected_node_id) +
              " has min_delay greater than max_delay";
        return false;
    }
    if (node_result.skew < -EPS) {
        err = "BU result for node " + std::to_string(expected_node_id) +
              " has negative skew";
        return false;
    }
    return true;
}

static bool validate_tree_node_shape(int node_id,
                                     const common::Problem& problem,
                                     const common::TopoTree& tree,
                                     std::string& err) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) {
        err = "Tree node id out of range";
        return false;
    }
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    if (node.id != node_id) {
        err = "Tree node id does not match vector index";
        return false;
    }
    if (node.is_sink) {
        if (node.sink_index < 0 ||
            static_cast<std::size_t>(node.sink_index) >= problem.sinks.size()) {
            err = "Leaf node " + std::to_string(node_id) + " has invalid sink index";
            return false;
        }
        return true;
    }
    if (node.left < 0 || node.right < 0 || node.left == node.right ||
        static_cast<std::size_t>(node.left) >= tree.nodes.size() ||
        static_cast<std::size_t>(node.right) >= tree.nodes.size()) {
        err = "Internal node " + std::to_string(node_id) + " has invalid children";
        return false;
    }
    return true;
}

static BranchCandidate build_branch_candidate(
    int parent_id,
    int child_id,
    bool is_left_child,
    const common::Problem& problem,
    const common::TopoTree& tree,
    const common::BottomUpResult& bu_result,
    const common::TopDownResult& result,
    std::string& err) {
    BranchCandidate branch;
    branch.child_id = child_id;
    branch.is_left_child = is_left_child;

    const common::BottomUpNodeResult& parent_bu =
        bu_result.node_results[static_cast<std::size_t>(parent_id)];
    const common::BottomUpNodeResult& child_bu =
        bu_result.node_results[static_cast<std::size_t>(child_id)];
    const common::TopoNode& child_node = tree.nodes[static_cast<std::size_t>(child_id)];

    if (!validate_bu_node(parent_bu, parent_id, err) ||
        !validate_bu_node(child_bu, child_id, err)) {
        return branch;
    }

    branch.assigned_edge = is_left_child ? parent_bu.edge_to_left
                                         : parent_bu.edge_to_right;
    branch.buffer_choice = is_left_child ? parent_bu.buffer_at_left_child
                                         : parent_bu.buffer_at_right_child;
    if (branch.assigned_edge < -EPS) {
        err = "TD assigned edge is negative for child " + std::to_string(child_id);
        return branch;
    }
    branch.assigned_edge = std::max(0.0, branch.assigned_edge);
    if (!is_valid_buffer_choice(branch.buffer_choice, problem, child_node, err)) {
        err = "Invalid buffer choice for child " + std::to_string(child_id) +
              ": " + err;
        return branch;
    }

    const common::SegmentPoint parent_loc =
        result.node_results[static_cast<std::size_t>(parent_id)].loc;
    const TRR parent_trr = expand_trr(segment_to_trr(point_segment(parent_loc.x,
                                                                   parent_loc.y)),
                                      branch.assigned_edge);
    const TRR child_base = segment_to_trr(child_bu.ms);
    const TRR feasible = intersect_trr(child_base, parent_trr);
    branch.feasible_ms = feasible.valid ? trr_to_representative_ms(feasible)
                                        : point_segment(0.0, 0.0);

    std::vector<common::SegmentPoint> candidates =
        generate_candidate_locs(child_bu.ms, feasible, parent_loc, problem.sinks);
    if (candidates.empty()) {
        err = "TD generated no candidate locations for child " + std::to_string(child_id);
        return branch;
    }
    branch.allowed_parent_sink = tree.nodes[static_cast<std::size_t>(parent_id)].is_sink
                                     ? tree.nodes[static_cast<std::size_t>(parent_id)].sink_index
                                     : -1;
    branch.allowed_child_sink = child_node.is_sink ? child_node.sink_index : -1;
    branch.route = route_to_best_candidate_loc_dag(parent_loc, candidates, problem.sinks,
                                                   branch.allowed_parent_sink,
                                                   branch.allowed_child_sink,
                                                   branch.assigned_edge,
                                                   err);
    if (!branch.route.valid) {
        err = "TD cannot route child " + std::to_string(child_id) + ": " + err;
        return branch;
    }
    branch.route_excess = std::max(0.0, branch.route.routed_len - branch.assigned_edge);
    branch.final_len = branch.route.routed_len;
    return branch;
}

static bool write_branch_result(int parent_id,
                                const BranchCandidate& branch,
                                double common_extra,
                                const common::Problem& problem,
                                const common::TopoTree& tree,
                                const common::BottomUpResult& bu_result,
                                common::TopDownResult& result,
                                std::string& err) {
    const common::BottomUpNodeResult& child_bu =
        bu_result.node_results[static_cast<std::size_t>(branch.child_id)];
    const common::TopoNode& child_node =
        tree.nodes[static_cast<std::size_t>(branch.child_id)];
    const double child_extra = result.node_results[static_cast<std::size_t>(branch.child_id)].valid
                                   ? result.node_results[static_cast<std::size_t>(branch.child_id)].td_common_extra_delay
                                   : 0.0;

    common::TopDownNodeResult out;
    out.node_id = branch.child_id;
    out.valid = true;
    out.loc = branch.route.loc;
    out.parent_id = parent_id;
    out.assigned_edge_to_parent = branch.assigned_edge;
    out.geometric_distance_to_parent =
        manhattan(result.node_results[static_cast<std::size_t>(parent_id)].loc, out.loc);
    out.routed_length_to_parent = branch.route.routed_len;
    out.compensation_detour_to_parent =
        std::max(0.0, branch.assigned_edge + (common_extra - child_extra) -
                          branch.route.routed_len);
    out.final_length_to_parent = out.routed_length_to_parent +
                                 out.compensation_detour_to_parent;
    if (out.compensation_detour_to_parent > SNAP_EPS) {
        std::string snake_err;
        out.compensation_snake_to_parent = make_compensation_snake(
            branch.route.path, out.compensation_detour_to_parent, problem.sinks,
            branch.allowed_parent_sink, branch.allowed_child_sink, snake_err);
        if (out.compensation_snake_to_parent.empty()) {
            err = snake_err.empty() ? "TD could not construct compensation snake"
                                    : snake_err;
            return false;
        }
    }
    out.route_to_parent = branch.route.path;
    out.final_route_to_parent = merge_base_route_and_compensation(
        out.route_to_parent, out.compensation_snake_to_parent);
    out.feasible_ms = branch.feasible_ms.valid ? branch.feasible_ms
                                               : point_segment(out.loc.x, out.loc.y);
    out.has_buffer = branch.buffer_choice.has_buffer;
    out.buffer_type_index = branch.buffer_choice.has_buffer
                                ? branch.buffer_choice.buffer_type_index
                                : -1;
    out.min_delay = child_bu.min_delay;
    out.max_delay = child_bu.max_delay;
    out.skew = child_bu.skew;
    out.td_common_extra_delay = child_extra;

    if (out.route_to_parent.size() < 2U ||
        !same_point(out.route_to_parent.front(),
                    result.node_results[static_cast<std::size_t>(parent_id)].loc) ||
        !same_point(out.route_to_parent.back(), out.loc)) {
        err = "TD route_to_parent endpoint mismatch for child " +
              std::to_string(branch.child_id);
        return false;
    }
    if (polyline_crosses_forbidden_sink(out.route_to_parent, problem.sinks,
                                        branch.allowed_parent_sink,
                                        branch.allowed_child_sink)) {
        err = "TD route_to_parent crosses forbidden sink for child " +
              std::to_string(branch.child_id);
        return false;
    }
    if (out.final_route_to_parent.size() < 2U ||
        !same_point(out.final_route_to_parent.front(),
                    result.node_results[static_cast<std::size_t>(parent_id)].loc) ||
        !same_point(out.final_route_to_parent.back(), out.loc)) {
        err = "TD final_route_to_parent endpoint mismatch for child " +
              std::to_string(branch.child_id);
        return false;
    }
    if (polyline_crosses_forbidden_sink(out.final_route_to_parent, problem.sinks,
                                        branch.allowed_parent_sink,
                                        branch.allowed_child_sink)) {
        err = "TD final_route_to_parent crosses forbidden sink for child " +
              std::to_string(branch.child_id);
        return false;
    }
    if (std::abs(out.routed_length_to_parent - polyline_length(out.route_to_parent)) >
        DELAY_EPS) {
        err = "TD routed length mismatch for child " +
              std::to_string(branch.child_id);
        return false;
    }
    if (std::abs(out.final_length_to_parent -
                 (out.routed_length_to_parent + out.compensation_detour_to_parent)) >
        DELAY_EPS) {
        err = "TD final branch length mismatch for child " +
              std::to_string(branch.child_id);
        return false;
    }
    if (std::abs(out.final_length_to_parent -
                 polyline_length(out.final_route_to_parent)) > DELAY_EPS) {
        err = "TD final route polyline length mismatch for child " +
              std::to_string(branch.child_id);
        return false;
    }
    if (child_node.is_sink && out.has_buffer) {
        err = "TD cannot place a buffer on leaf child " +
              std::to_string(branch.child_id);
        return false;
    }

    result.node_results[static_cast<std::size_t>(branch.child_id)] = out;
    return true;
}

static bool place_sibling_pair_with_balancing(
    int parent_id,
    int left_id,
    int right_id,
    const common::Problem& problem,
    const common::TopoTree& tree,
    const common::BottomUpResult& bu_result,
    common::TopDownResult& result,
    std::string& err) {
    BranchCandidate left = build_branch_candidate(parent_id, left_id, true,
                                                  problem, tree, bu_result,
                                                  result, err);
    if (!left.route.valid) {
        return false;
    }
    BranchCandidate right = build_branch_candidate(parent_id, right_id, false,
                                                   problem, tree, bu_result,
                                                   result, err);
    if (!right.route.valid) {
        return false;
    }

    const double left_child_extra =
        result.node_results[static_cast<std::size_t>(left_id)].valid
            ? result.node_results[static_cast<std::size_t>(left_id)].td_common_extra_delay
            : 0.0;
    const double right_child_extra =
        result.node_results[static_cast<std::size_t>(right_id)].valid
            ? result.node_results[static_cast<std::size_t>(right_id)].td_common_extra_delay
            : 0.0;
    const double common_extra = std::max(left_child_extra + left.route_excess,
                                         right_child_extra + right.route_excess);

    if (!write_branch_result(parent_id, left, common_extra, problem, tree, bu_result,
                             result, err) ||
        !write_branch_result(parent_id, right, common_extra, problem, tree, bu_result,
                             result, err)) {
        return false;
    }
    const common::TopDownNodeResult& left_td =
        result.node_results[static_cast<std::size_t>(left_id)];
    const common::TopDownNodeResult& right_td =
        result.node_results[static_cast<std::size_t>(right_id)];
    const double left_extra = left_td.td_common_extra_delay +
                              left_td.final_length_to_parent -
                              left.assigned_edge;
    const double right_extra = right_td.td_common_extra_delay +
                               right_td.final_length_to_parent -
                               right.assigned_edge;
    if (std::abs(left_extra - right_extra) > DELAY_EPS ||
        std::abs(left_extra - common_extra) > DELAY_EPS) {
        err = "TD sibling balancing invariant failed at parent " +
              std::to_string(parent_id);
        return false;
    }
    result.node_results[static_cast<std::size_t>(parent_id)].td_common_extra_delay =
        common_extra;

    if (g_debug_enabled) {
        std::cout << "[TD_PAIR] parent=" << parent_id
                  << " left=" << left_id
                  << " right=" << right_id << "\n";
        std::cout << "[TD_PAIR]   left selected=(" << left.route.loc.x << ","
                  << left.route.loc.y << ") assigned=" << left.assigned_edge
                  << " routed=" << left.route.routed_len
                  << " excess=" << left.route_excess
                  << " child_extra=" << left_child_extra
                  << " compensation=" << left_td.compensation_detour_to_parent
                  << " final_len=" << left_td.final_length_to_parent
                  << " cost=" << left.route.cost << "\n";
        std::cout << "[TD_PAIR]   right selected=(" << right.route.loc.x << ","
                  << right.route.loc.y << ") assigned=" << right.assigned_edge
                  << " routed=" << right.route.routed_len
                  << " excess=" << right.route_excess
                  << " child_extra=" << right_child_extra
                  << " compensation=" << right_td.compensation_detour_to_parent
                  << " final_len=" << right_td.final_length_to_parent
                  << " cost=" << right.route.cost << "\n";
        std::cout << "[TD_PAIR]   common_extra=" << common_extra << "\n";
    }
    return true;
}

static bool check_bu_delay_consistency(int parent_id,
                                       const common::Problem& problem,
                                       const common::TopoTree& tree,
                                       const common::BottomUpResult& bu_result,
                                       std::string& err) {
    const common::TopoNode& parent = tree.nodes[static_cast<std::size_t>(parent_id)];
    if (parent.is_sink) {
        return true;
    }
    const common::BottomUpNodeResult& parent_bu =
        bu_result.node_results[static_cast<std::size_t>(parent_id)];
    const common::BottomUpNodeResult& left_bu =
        bu_result.node_results[static_cast<std::size_t>(parent.left)];
    const common::BottomUpNodeResult& right_bu =
        bu_result.node_results[static_cast<std::size_t>(parent.right)];

    const double left_min = left_bu.min_delay + parent_bu.edge_to_left +
                            buffer_delay(problem, parent_bu.buffer_at_left_child);
    const double left_max = left_bu.max_delay + parent_bu.edge_to_left +
                            buffer_delay(problem, parent_bu.buffer_at_left_child);
    const double right_min = right_bu.min_delay + parent_bu.edge_to_right +
                             buffer_delay(problem, parent_bu.buffer_at_right_child);
    const double right_max = right_bu.max_delay + parent_bu.edge_to_right +
                             buffer_delay(problem, parent_bu.buffer_at_right_child);
    const double expected_min = std::min(left_min, right_min);
    const double expected_max = std::max(left_max, right_max);
    const double expected_skew = expected_max - expected_min;
    const double worst = std::max(std::abs(expected_min - parent_bu.min_delay),
                                  std::max(std::abs(expected_max - parent_bu.max_delay),
                                           std::abs(expected_skew - parent_bu.skew)));
    if (worst > 1e-5) {
        std::ostringstream oss;
        oss << "BU delay consistency failed at node " << parent_id;
        err = oss.str();
        return false;
    }
    return true;
}

static bool place_node_recursive(int node_id,
                                 const common::Problem& problem,
                                 const common::TopoTree& tree,
                                 const common::BottomUpResult& bu_result,
                                 common::TopDownResult& result,
                                 std::vector<int>& state,
                                 std::string& err) {
    if (!validate_tree_node_shape(node_id, problem, tree, err)) {
        return false;
    }
    const std::size_t idx = static_cast<std::size_t>(node_id);
    if (state[idx] == 1) {
        err = "Cycle detected in topology tree during TD";
        return false;
    }
    if (state[idx] == 2) {
        return true;
    }
    if (!result.node_results[idx].valid) {
        err = "TD node " + std::to_string(node_id) + " was not placed";
        return false;
    }

    state[idx] = 1;
    if (!check_bu_delay_consistency(node_id, problem, tree, bu_result, err)) {
        return false;
    }
    const common::TopoNode& node = tree.nodes[idx];
    if (!node.is_sink) {
        if (!place_sibling_pair_with_balancing(node_id, node.left, node.right, problem, tree, bu_result,
                                               result, err) ||
            !place_node_recursive(node.left, problem, tree, bu_result, result,
                                  state, err) ||
            !place_node_recursive(node.right, problem, tree, bu_result, result,
                                  state, err) ||
            !place_sibling_pair_with_balancing(node_id, node.left, node.right, problem, tree, bu_result,
                                               result, err)) {
            return false;
        }
    }
    state[idx] = 2;
    return true;
}
static bool validate_all_nodes_visited(const std::vector<int>& state,
                                       std::string& err) {
    for (std::size_t i = 0; i < state.size(); ++i) {
        if (state[i] != 2) {
            err = "Tree node " + std::to_string(i) +
                  " was not visited by TD traversal";
            return false;
        }
    }
    return true;
}

static bool validate_td_node(const common::TopDownNodeResult& node_result,
                             int expected_node_id,
                             const common::Problem& problem,
                             const common::TopoTree& tree,
                             std::string& err) {
    if (!node_result.valid) {
        err = "TD result for node " + std::to_string(expected_node_id) + " is invalid";
        return false;
    }
    if (node_result.node_id != expected_node_id) {
        err = "TD result node_id mismatch for node " + std::to_string(expected_node_id);
        return false;
    }
    if (!is_finite_point(node_result.loc)) {
        err = "TD node " + std::to_string(expected_node_id) + " has non-finite loc";
        return false;
    }
    if (node_result.assigned_edge_to_parent < -EPS ||
        node_result.geometric_distance_to_parent < -EPS ||
        node_result.routed_length_to_parent < -EPS ||
        node_result.compensation_detour_to_parent < -EPS ||
        node_result.final_length_to_parent < -EPS) {
        err = "TD node " + std::to_string(expected_node_id) +
              " has negative branch geometry";
        return false;
    }
    if (node_result.min_delay > node_result.max_delay + EPS ||
        node_result.skew < -EPS) {
        err = "TD node " + std::to_string(expected_node_id) +
              " has invalid delay interval";
        return false;
    }
    if (expected_node_id != tree.root) {
        if (node_result.route_to_parent.size() < 2U) {
            err = "TD non-root node " + std::to_string(expected_node_id) +
                  " has empty route_to_parent";
            return false;
        }
        if (node_result.final_route_to_parent.size() < 2U) {
            err = "TD non-root node " + std::to_string(expected_node_id) +
                  " has empty final_route_to_parent";
            return false;
        }
        const int parent_id = tree.nodes[static_cast<std::size_t>(expected_node_id)].parent;
        const common::SegmentPoint& parent_loc =
            node_result.final_route_to_parent.front();
        (void)parent_loc;
        const int allowed_parent_sink = tree.nodes[static_cast<std::size_t>(parent_id)].is_sink
                                            ? tree.nodes[static_cast<std::size_t>(parent_id)].sink_index
                                            : -1;
        const int allowed_child_sink = tree.nodes[static_cast<std::size_t>(expected_node_id)].is_sink
                                           ? tree.nodes[static_cast<std::size_t>(expected_node_id)].sink_index
                                           : -1;
        const common::SegmentPoint expected_parent_loc =
            node_result.route_to_parent.front();
        if (!same_point(node_result.final_route_to_parent.front(), expected_parent_loc) ||
            !same_point(node_result.final_route_to_parent.back(), node_result.loc)) {
            err = "TD final_route_to_parent endpoint mismatch at node " +
                  std::to_string(expected_node_id);
            return false;
        }
        if (polyline_crosses_forbidden_sink(node_result.route_to_parent,
                                            problem.sinks,
                                            allowed_parent_sink,
                                            allowed_child_sink)) {
            err = "TD route crosses forbidden sink at node " +
                  std::to_string(expected_node_id);
            return false;
        }
        if (polyline_crosses_forbidden_sink(node_result.final_route_to_parent,
                                            problem.sinks,
                                            allowed_parent_sink,
                                            allowed_child_sink)) {
            err = "TD final route crosses forbidden sink at node " +
                  std::to_string(expected_node_id);
            return false;
        }
        if (std::abs(node_result.routed_length_to_parent -
                     polyline_length(node_result.route_to_parent)) > DELAY_EPS) {
            err = "TD route_to_parent length mismatch at node " +
                  std::to_string(expected_node_id);
            return false;
        }
        if (std::abs(node_result.final_length_to_parent -
                     (node_result.routed_length_to_parent +
                      node_result.compensation_detour_to_parent)) > DELAY_EPS) {
            err = "TD final length fields mismatch at node " +
                  std::to_string(expected_node_id);
            return false;
        }
        if (std::abs(node_result.final_length_to_parent -
                     polyline_length(node_result.final_route_to_parent)) > DELAY_EPS) {
            err = "TD final_route_to_parent length mismatch at node " +
                  std::to_string(expected_node_id);
            return false;
        }
    } else if (!node_result.route_to_parent.empty() ||
               !node_result.compensation_snake_to_parent.empty() ||
               !node_result.final_route_to_parent.empty()) {
        err = "TD root node unexpectedly has parent route geometry";
        return false;
    }
    return true;
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_output(const TopDownResult& result,
                  const common::Problem& problem,
                  const common::TopoTree& tree,
                  const common::BottomUpResult& bu_result) {
    if (!g_debug_enabled) {
        return;
    }

    std::cout << "[TD] valid=" << (result.valid ? 1 : 0)
              << " error_msg=" << result.error_msg
              << " root=" << result.root
              << " num_node_results=" << result.node_results.size() << "\n";
    for (std::size_t i = 0; i < result.node_results.size(); ++i) {
        if (i >= tree.nodes.size() || i >= bu_result.node_results.size()) {
            continue;
        }
        const common::TopoNode& tree_node = tree.nodes[i];
        const common::TopDownNodeResult& td_node = result.node_results[i];
        const common::BottomUpNodeResult& bu_node = bu_result.node_results[i];

        std::cout << "[TD] node_id=" << tree_node.id
                  << " parent=" << tree_node.parent
                  << " left=" << tree_node.left
                  << " right=" << tree_node.right
                  << " is_leaf=" << (tree_node.is_sink ? 1 : 0)
                  << " sink_index=" << tree_node.sink_index
                  << " sink_count=" << tree_node.sink_indices.size() << "\n";
        if (!td_node.valid) {
            std::cout << "[TD]   td_result=INVALID\n";
            continue;
        }
        std::cout << "[TD]   loc=(" << td_node.loc.x << "," << td_node.loc.y << ")\n";
        std::cout << "[TD]   assigned_edge=" << td_node.assigned_edge_to_parent
                  << " geo=" << td_node.geometric_distance_to_parent
                  << " routed_len=" << td_node.routed_length_to_parent
                  << " compensation=" << td_node.compensation_detour_to_parent
                  << " final_len=" << td_node.final_length_to_parent
                  << " td_common_extra_delay=" << td_node.td_common_extra_delay << "\n";
        std::cout << "[TD]   feasible_ms=" << ms_to_string(td_node.feasible_ms) << "\n";
        std::cout << "[TD]   route_to_parent";
        for (const common::SegmentPoint& p : td_node.route_to_parent) {
            std::cout << " (" << p.x << "," << p.y << ")";
        }
        std::cout << "\n";
        std::cout << "[TD]   compensation_snake_to_parent";
        for (const common::SegmentPoint& p : td_node.compensation_snake_to_parent) {
            std::cout << " (" << p.x << "," << p.y << ")";
        }
        std::cout << "\n";
        std::cout << "[TD]   final_route_to_parent";
        for (const common::SegmentPoint& p : td_node.final_route_to_parent) {
            std::cout << " (" << p.x << "," << p.y << ")";
        }
        std::cout << "\n";
        std::cout << "[TD]   buffer_at_node="
                  << buffer_marker_to_string(td_node.has_buffer,
                                             td_node.buffer_type_index,
                                             problem)
                  << "\n";
        std::cout << "[TD]   bu_ms=" << ms_to_string(bu_node.ms)
                  << " bu_min_delay=" << bu_node.min_delay
                  << " bu_max_delay=" << bu_node.max_delay
                  << " bu_skew=" << bu_node.skew << "\n";
    }
}

TopDownResult run(const common::Problem& problem,
                  const common::TopoTree& tree,
                  const common::BottomUpResult& bu_result) {
    TopDownResult result;
    if (!problem.valid) {
        result.error_msg = "Cannot run TD on invalid problem: " + problem.error_msg;
        return result;
    }
    if (!tree.valid) {
        result.error_msg = "Cannot run TD on invalid tree: " + tree.error_msg;
        return result;
    }
    if (!bu_result.valid) {
        result.error_msg = "Cannot run TD on invalid BU result: " + bu_result.error_msg;
        return result;
    }
    if (tree.nodes.empty()) {
        result.error_msg = "Cannot run TD on empty tree";
        return result;
    }
    if (tree.root < 0 || static_cast<std::size_t>(tree.root) >= tree.nodes.size()) {
        result.error_msg = "Invalid tree root";
        return result;
    }
    if (bu_result.node_results.size() != tree.nodes.size()) {
        result.error_msg = "BU result node count does not match tree node count";
        return result;
    }

    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        std::string err;
        if (!validate_tree_node_shape(static_cast<int>(i), problem, tree, err) ||
            !validate_bu_node(bu_result.node_results[i], static_cast<int>(i), err)) {
            result.error_msg = err;
            return result;
        }
    }

    const int root = tree.root;
    const common::BottomUpNodeResult& root_bu =
        bu_result.node_results[static_cast<std::size_t>(root)];
    if (!is_valid_ms_segment(root_bu.ms)) {
        result.error_msg = "Root BU result has invalid merging segment";
        return result;
    }

    result.node_results.resize(tree.nodes.size());
    common::TopDownNodeResult root_td;
    root_td.node_id = root;
    root_td.valid = true;
    root_td.loc = midpoint_of_ms(root_bu.ms);
    root_td.parent_id = -1;
    root_td.feasible_ms = root_bu.ms;
    root_td.min_delay = root_bu.min_delay;
    root_td.max_delay = root_bu.max_delay;
    root_td.skew = root_bu.skew;
    result.node_results[static_cast<std::size_t>(root)] = root_td;

    std::string err;
    std::vector<int> state(tree.nodes.size(), 0);
    if (!place_node_recursive(root, problem, tree, bu_result, result, state, err) ||
        !validate_all_nodes_visited(state, err)) {
        result.error_msg = err;
        return result;
    }

    for (std::size_t i = 0; i < result.node_results.size(); ++i) {
        if (!validate_td_node(result.node_results[i], static_cast<int>(i),
                              problem, tree, err)) {
            result.error_msg = err;
            return result;
        }
    }

    result.root = root;
    result.valid = true;
    if (g_debug_enabled) {
        debug_output(result, problem, tree, bu_result);
    }
    return result;
}

}  // namespace td
