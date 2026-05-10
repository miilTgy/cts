#include "partreer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <string>
#include <vector>

namespace partreer {
namespace {

static bool g_debug_enabled = false;

constexpr int kTargetAccessPoints = 3;
constexpr int kNearest = 3;
constexpr int kMaxPairLevels = 8;
constexpr double kEps = 1e-9;

struct Segment {
    int a = -1;
    int b = -1;
};

struct Candidate {
    int i = -1;
    int j = -1;
    double cost = 0.0;
};

static const char* kind_to_string(common::NodeKind kind) {
    switch (kind) {
        case common::NodeKind::Sink:
            return "SINK";
        case common::NodeKind::ClusterInternal:
            return "CLUSTER_INTERNAL";
        case common::NodeKind::ClusterAccess:
            return "CLUSTER_ACCESS";
        case common::NodeKind::Global:
            return "GLOBAL";
    }
    return "UNKNOWN";
}

static int manhattan(const common::Point& a, const common::Point& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

static int clamp_int(int value, int lo, int hi) {
    return std::max(lo, std::min(value, hi));
}

static common::BBox union_bbox(const common::BBox& a, const common::BBox& b) {
    return {std::min(a.lx, b.lx),
            std::min(a.ly, b.ly),
            std::max(a.ux, b.ux),
            std::max(a.uy, b.uy)};
}

static bool same_point(const common::Point& a, const common::Point& b) {
    return a.x == b.x && a.y == b.y;
}

static long long cross(const common::Point& a,
                       const common::Point& b,
                       const common::Point& c) {
    return static_cast<long long>(b.x - a.x) * static_cast<long long>(c.y - a.y) -
           static_cast<long long>(b.y - a.y) * static_cast<long long>(c.x - a.x);
}

static bool between_int(int a, int b, int c) {
    return std::min(a, c) <= b && b <= std::max(a, c);
}

static bool on_segment(const common::Point& a,
                       const common::Point& p,
                       const common::Point& b) {
    return cross(a, b, p) == 0 &&
           between_int(a.x, p.x, b.x) &&
           between_int(a.y, p.y, b.y);
}

static bool collinear_overlap(const common::Point& a,
                              const common::Point& b,
                              const common::Point& c,
                              const common::Point& d) {
    if (cross(a, b, c) != 0 || cross(a, b, d) != 0) {
        return false;
    }
    const bool x_overlap = std::max(std::min(a.x, b.x), std::min(c.x, d.x)) <=
                           std::min(std::max(a.x, b.x), std::max(c.x, d.x));
    const bool y_overlap = std::max(std::min(a.y, b.y), std::min(c.y, d.y)) <=
                           std::min(std::max(a.y, b.y), std::max(c.y, d.y));
    return x_overlap && y_overlap;
}

static bool segments_intersect(const common::Point& a,
                               const common::Point& b,
                               const common::Point& c,
                               const common::Point& d) {
    const long long c1 = cross(a, b, c);
    const long long c2 = cross(a, b, d);
    const long long c3 = cross(c, d, a);
    const long long c4 = cross(c, d, b);

    if (((c1 > 0 && c2 < 0) || (c1 < 0 && c2 > 0)) &&
        ((c3 > 0 && c4 < 0) || (c3 < 0 && c4 > 0))) {
        return true;
    }
    if (c1 == 0 && on_segment(a, c, b)) return true;
    if (c2 == 0 && on_segment(a, d, b)) return true;
    if (c3 == 0 && on_segment(c, a, d)) return true;
    if (c4 == 0 && on_segment(c, b, d)) return true;
    return false;
}

static bool only_allowed_shared_endpoint(const common::TopoNode& ca,
                                         const common::TopoNode& cb,
                                         const common::TopoNode& ea,
                                         const common::TopoNode& eb) {
    const bool share_ca_ea = ca.id == ea.id && same_point(ca.loc, ea.loc);
    const bool share_ca_eb = ca.id == eb.id && same_point(ca.loc, eb.loc);
    const bool share_cb_ea = cb.id == ea.id && same_point(cb.loc, ea.loc);
    const bool share_cb_eb = cb.id == eb.id && same_point(cb.loc, eb.loc);
    const int shared_count = static_cast<int>(share_ca_ea) +
                             static_cast<int>(share_ca_eb) +
                             static_cast<int>(share_cb_ea) +
                             static_cast<int>(share_cb_eb);
    if (shared_count != 1) return false;
    if (collinear_overlap(ca.loc, cb.loc, ea.loc, eb.loc)) return false;
    return true;
}

static bool connectable_segment(const common::TopoTree& tree,
                                const std::vector<Segment>& segments,
                                int left,
                                int right) {
    const common::TopoNode& a = tree.nodes[static_cast<std::size_t>(left)];
    const common::TopoNode& b = tree.nodes[static_cast<std::size_t>(right)];
    if (same_point(a.loc, b.loc)) {
        return true;
    }

    for (const Segment& seg : segments) {
        const common::TopoNode& c = tree.nodes[static_cast<std::size_t>(seg.a)];
        const common::TopoNode& d = tree.nodes[static_cast<std::size_t>(seg.b)];
        if (!segments_intersect(a.loc, b.loc, c.loc, d.loc)) {
            continue;
        }
        if (only_allowed_shared_endpoint(a, b, c, d)) {
            continue;
        }
        return false;
    }
    return true;
}

static bool connectable_segment_to_point(const common::TopoTree& tree,
                                         const std::vector<Segment>& segments,
                                         int endpoint_id,
                                         common::Point other_point) {
    const common::TopoNode& endpoint =
        tree.nodes[static_cast<std::size_t>(endpoint_id)];
    if (same_point(endpoint.loc, other_point)) {
        return true;
    }

    for (const Segment& seg : segments) {
        const common::TopoNode& c = tree.nodes[static_cast<std::size_t>(seg.a)];
        const common::TopoNode& d = tree.nodes[static_cast<std::size_t>(seg.b)];
        if (!segments_intersect(endpoint.loc, other_point, c.loc, d.loc)) {
            continue;
        }

        const bool shared_c = endpoint.id == c.id && same_point(endpoint.loc, c.loc);
        const bool shared_d = endpoint.id == d.id && same_point(endpoint.loc, d.loc);
        if ((shared_c || shared_d) && !collinear_overlap(endpoint.loc, other_point, c.loc, d.loc)) {
            continue;
        }
        return false;
    }
    return true;
}

static common::TopoNode make_sink_node(int id,
                                       int sink_index,
                                       const common::Problem& problem) {
    const common::Sink& sink = problem.sinks[static_cast<std::size_t>(sink_index)];
    common::TopoNode node;
    node.id = id;
    node.loc = sink.loc;
    node.is_sink = true;
    node.sink_index = sink_index;
    node.kind = common::NodeKind::Sink;
    node.sink_indices = {sink_index};
    node.bbox = {sink.loc.x, sink.loc.y, sink.loc.x, sink.loc.y};
    node.region_lx = 0;
    node.region_ly = 0;
    node.region_ux = problem.die_width;
    node.region_uy = problem.die_height;
    return node;
}

static void update_delay_fields(common::TopoNode& parent,
                                const common::TopoNode& left,
                                const common::TopoNode& right) {
    const int left_edge = manhattan(parent.loc, left.loc);
    const int right_edge = manhattan(parent.loc, right.loc);

    parent.left_min_delay_to_node = left.subtree_min_delay_to_node + left_edge;
    parent.left_max_delay_to_node = left.subtree_max_delay_to_node + left_edge;
    parent.left_skew_to_node = parent.left_max_delay_to_node -
                               parent.left_min_delay_to_node;

    parent.right_min_delay_to_node = right.subtree_min_delay_to_node + right_edge;
    parent.right_max_delay_to_node = right.subtree_max_delay_to_node + right_edge;
    parent.right_skew_to_node = parent.right_max_delay_to_node -
                                parent.right_min_delay_to_node;

    parent.subtree_min_delay_to_node =
        std::min(parent.left_min_delay_to_node, parent.right_min_delay_to_node);
    parent.subtree_max_delay_to_node =
        std::max(parent.left_max_delay_to_node, parent.right_max_delay_to_node);
    parent.subtree_skew_to_node = parent.subtree_max_delay_to_node -
                                  parent.subtree_min_delay_to_node;
}

static common::Point midpoint_with_bias(const common::TopoNode& left,
                                        const common::TopoNode& right,
                                        common::Point external_target,
                                        const common::Problem& problem) {
    common::Point p;
    p.x = (left.loc.x + right.loc.x + 1) / 2;
    p.y = (left.loc.y + right.loc.y + 1) / 2;

    common::Point biased = p;
    if (external_target.x > biased.x) ++biased.x;
    if (external_target.x < biased.x) --biased.x;
    if (external_target.y > biased.y) ++biased.y;
    if (external_target.y < biased.y) --biased.y;
    biased.x = clamp_int(biased.x, 0, problem.die_width);
    biased.y = clamp_int(biased.y, 0, problem.die_height);
    return biased;
}

static common::Point raw_midpoint(const common::TopoNode& left,
                                  const common::TopoNode& right,
                                  const common::Problem& problem) {
    common::Point p;
    p.x = clamp_int((left.loc.x + right.loc.x + 1) / 2, 0, problem.die_width);
    p.y = clamp_int((left.loc.y + right.loc.y + 1) / 2, 0, problem.die_height);
    return p;
}

static bool actual_parent_segments_connectable(const common::TopoTree& tree,
                                               const std::vector<Segment>& segments,
                                               int left_id,
                                               int right_id,
                                               common::Point parent_loc) {
    return connectable_segment_to_point(tree, segments, left_id, parent_loc) &&
           connectable_segment_to_point(tree, segments, right_id, parent_loc);
}

static int create_parent(common::TopoTree& tree,
                         std::vector<Segment>& segments,
                         int left_id,
                         int right_id,
                         common::Point external_target,
                         const common::Problem& problem,
                         common::NodeKind kind) {
    const common::TopoNode left = tree.nodes[static_cast<std::size_t>(left_id)];
    const common::TopoNode right = tree.nodes[static_cast<std::size_t>(right_id)];

    common::TopoNode parent;
    parent.id = static_cast<int>(tree.nodes.size());
    parent.left = left_id;
    parent.right = right_id;
    parent.kind = kind;
    parent.loc = midpoint_with_bias(left, right, external_target, problem);
    if (!actual_parent_segments_connectable(tree, segments, left_id, right_id, parent.loc)) {
        parent.loc = raw_midpoint(left, right, problem);
    }
    parent.bbox = union_bbox(left.bbox, right.bbox);
    parent.sink_indices = left.sink_indices;
    parent.sink_indices.insert(parent.sink_indices.end(),
                               right.sink_indices.begin(),
                               right.sink_indices.end());
    std::sort(parent.sink_indices.begin(), parent.sink_indices.end());
    parent.region_lx = 0;
    parent.region_ly = 0;
    parent.region_ux = problem.die_width;
    parent.region_uy = problem.die_height;
    update_delay_fields(parent, left, right);

    const int parent_id = parent.id;
    tree.nodes.push_back(parent);
    tree.nodes[static_cast<std::size_t>(left_id)].parent = parent_id;
    tree.nodes[static_cast<std::size_t>(right_id)].parent = parent_id;
    segments.push_back({parent_id, left_id});
    segments.push_back({parent_id, right_id});
    return parent_id;
}

static int create_access_wrapper(common::TopoTree& tree,
                                 std::vector<Segment>& segments,
                                 int child_id,
                                 const common::Problem& problem) {
    const common::TopoNode child = tree.nodes[static_cast<std::size_t>(child_id)];
    common::TopoNode parent;
    parent.id = static_cast<int>(tree.nodes.size());
    parent.left = child_id;
    parent.right = -1;
    parent.loc = child.loc;
    parent.bbox = child.bbox;
    parent.sink_indices = child.sink_indices;
    parent.region_lx = 0;
    parent.region_ly = 0;
    parent.region_ux = problem.die_width;
    parent.region_uy = problem.die_height;
    parent.kind = common::NodeKind::ClusterAccess;
    parent.left_min_delay_to_node = child.subtree_min_delay_to_node;
    parent.left_max_delay_to_node = child.subtree_max_delay_to_node;
    parent.left_skew_to_node = child.subtree_skew_to_node;
    parent.subtree_min_delay_to_node = child.subtree_min_delay_to_node;
    parent.subtree_max_delay_to_node = child.subtree_max_delay_to_node;
    parent.subtree_skew_to_node = child.subtree_skew_to_node;

    const int parent_id = parent.id;
    tree.nodes.push_back(parent);
    tree.nodes[static_cast<std::size_t>(child_id)].parent = parent_id;
    segments.push_back({parent_id, child_id});
    return parent_id;
}

static double pair_cost(const common::TopoNode& a,
                        const common::TopoNode& b,
                        const std::vector<int>& active,
                        const common::TopoTree& tree) {
    const int dist = manhattan(a.loc, b.loc);
    const common::BBox bbox = union_bbox(a.bbox, b.bbox);
    const int bbox_penalty = (bbox.ux - bbox.lx) + (bbox.uy - bbox.ly);
    const int skew_penalty =
        std::abs(a.subtree_skew_to_node - b.subtree_skew_to_node);

    int interleave = 0;
    const int min_x = std::min(a.loc.x, b.loc.x);
    const int max_x = std::max(a.loc.x, b.loc.x);
    const int min_y = std::min(a.loc.y, b.loc.y);
    const int max_y = std::max(a.loc.y, b.loc.y);
    for (int id : active) {
        if (id == a.id || id == b.id) continue;
        const common::Point p = tree.nodes[static_cast<std::size_t>(id)].loc;
        if (min_x <= p.x && p.x <= max_x && min_y <= p.y && p.y <= max_y) {
            ++interleave;
        }
    }

    return static_cast<double>(dist) +
           0.10 * static_cast<double>(bbox_penalty) +
           0.50 * static_cast<double>(interleave) +
           0.05 * static_cast<double>(skew_penalty);
}

static void add_candidate(std::set<std::pair<int, int>>& pairs, int a, int b) {
    if (a == b) return;
    if (a > b) std::swap(a, b);
    pairs.insert({a, b});
}

static std::vector<Candidate> generate_candidates(const std::vector<int>& active,
                                                  const common::TopoTree& tree) {
    std::set<std::pair<int, int>> pairs;

    std::vector<int> by_x = active;
    std::stable_sort(by_x.begin(), by_x.end(), [&](int a, int b) {
        const common::Point& pa = tree.nodes[static_cast<std::size_t>(a)].loc;
        const common::Point& pb = tree.nodes[static_cast<std::size_t>(b)].loc;
        if (pa.x != pb.x) return pa.x < pb.x;
        if (pa.y != pb.y) return pa.y < pb.y;
        return a < b;
    });
    std::vector<int> by_y = active;
    std::stable_sort(by_y.begin(), by_y.end(), [&](int a, int b) {
        const common::Point& pa = tree.nodes[static_cast<std::size_t>(a)].loc;
        const common::Point& pb = tree.nodes[static_cast<std::size_t>(b)].loc;
        if (pa.y != pb.y) return pa.y < pb.y;
        if (pa.x != pb.x) return pa.x < pb.x;
        return a < b;
    });

    for (std::size_t i = 0; i < active.size(); ++i) {
        for (int d = 1; d <= kNearest; ++d) {
            if (i + static_cast<std::size_t>(d) < active.size()) {
                add_candidate(pairs, by_x[i], by_x[i + static_cast<std::size_t>(d)]);
                add_candidate(pairs, by_y[i], by_y[i + static_cast<std::size_t>(d)]);
            }
        }
    }

    for (int a : active) {
        std::vector<std::pair<int, int>> nearest;
        for (int b : active) {
            if (a == b) continue;
            nearest.push_back({manhattan(tree.nodes[static_cast<std::size_t>(a)].loc,
                                         tree.nodes[static_cast<std::size_t>(b)].loc),
                               b});
        }
        std::sort(nearest.begin(), nearest.end());
        for (int k = 0; k < kNearest && k < static_cast<int>(nearest.size()); ++k) {
            add_candidate(pairs, a, nearest[static_cast<std::size_t>(k)].second);
        }
    }

    std::vector<Candidate> candidates;
    for (const auto& p : pairs) {
        const common::TopoNode& a = tree.nodes[static_cast<std::size_t>(p.first)];
        const common::TopoNode& b = tree.nodes[static_cast<std::size_t>(p.second)];
        candidates.push_back({p.first, p.second, pair_cost(a, b, active, tree)});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (std::abs(a.cost - b.cost) > kEps) return a.cost < b.cost;
        if (a.i != b.i) return a.i < b.i;
        return a.j < b.j;
    });
    return candidates;
}

static std::vector<int> pairing_level(const std::vector<int>& active,
                                      common::TopoTree& tree,
                                      std::vector<Segment>& segments,
                                      common::Point external_target,
                                      const common::Problem& problem,
                                      bool require_connectable) {
    std::vector<Candidate> candidates = generate_candidates(active, tree);
    std::vector<int> next;
    std::set<int> used;

    for (const Candidate& cand : candidates) {
        if (used.count(cand.i) || used.count(cand.j)) continue;
        if (require_connectable &&
            !connectable_segment(tree, segments, cand.i, cand.j)) {
            continue;
        }
        const common::TopoNode& left = tree.nodes[static_cast<std::size_t>(cand.i)];
        const common::TopoNode& right = tree.nodes[static_cast<std::size_t>(cand.j)];
        const common::Point biased = midpoint_with_bias(left, right, external_target, problem);
        const common::Point midpoint = raw_midpoint(left, right, problem);
        if (require_connectable &&
            !actual_parent_segments_connectable(tree, segments, cand.i, cand.j, biased) &&
            !actual_parent_segments_connectable(tree, segments, cand.i, cand.j, midpoint)) {
            continue;
        }
        const int parent = create_parent(tree, segments, cand.i, cand.j,
                                         external_target, problem,
                                         common::NodeKind::ClusterInternal);
        used.insert(cand.i);
        used.insert(cand.j);
        next.push_back(parent);
        if (g_debug_enabled) {
            std::cout << "[PARTREER] pair " << cand.i << " " << cand.j
                      << " -> " << parent << " cost=" << cand.cost << "\n";
        }
    }

    for (int id : active) {
        if (!used.count(id)) next.push_back(id);
    }
    return next;
}

static int build_balanced_access(std::vector<int> roots,
                                 common::TopoTree& tree,
                                 std::vector<Segment>& segments,
                                 common::Point external_target,
                                 const common::Problem& problem) {
    if (roots.empty()) return -1;
    if (roots.size() == 1) {
        if (tree.nodes[static_cast<std::size_t>(roots[0])].kind ==
            common::NodeKind::ClusterAccess) {
            return roots[0];
        }
        return create_access_wrapper(tree, segments, roots[0], problem);
    }

    std::stable_sort(roots.begin(), roots.end(), [&](int a, int b) {
        const common::Point& pa = tree.nodes[static_cast<std::size_t>(a)].loc;
        const common::Point& pb = tree.nodes[static_cast<std::size_t>(b)].loc;
        const int da = std::abs(pa.x - external_target.x) +
                       std::abs(pa.y - external_target.y);
        const int db = std::abs(pb.x - external_target.x) +
                       std::abs(pb.y - external_target.y);
        if (da != db) return da < db;
        if (pa.x != pb.x) return pa.x < pb.x;
        if (pa.y != pb.y) return pa.y < pb.y;
        return a < b;
    });

    while (roots.size() > 1) {
        std::vector<int> next;
        for (std::size_t i = 0; i < roots.size(); i += 2) {
            if (i + 1 >= roots.size()) {
                next.push_back(roots[i]);
                continue;
            }
            int a = roots[i];
            int b = roots[i + 1];
            auto can_pair = [&](int x, int y) {
                const common::TopoNode& left = tree.nodes[static_cast<std::size_t>(x)];
                const common::TopoNode& right = tree.nodes[static_cast<std::size_t>(y)];
                const common::Point biased = midpoint_with_bias(left, right, external_target, problem);
                const common::Point midpoint = raw_midpoint(left, right, problem);
                return connectable_segment(tree, segments, x, y) &&
                       (actual_parent_segments_connectable(tree, segments, x, y, biased) ||
                        actual_parent_segments_connectable(tree, segments, x, y, midpoint));
            };
            if (!can_pair(a, b)) {
                // The prompt allows changing pairing order for access-tree
                // fallback. Try a later partner before accepting an abstract
                // edge that preserves binary topology.
                for (std::size_t j = i + 2; j < roots.size(); ++j) {
                    if (can_pair(a, roots[j])) {
                        std::swap(roots[i + 1], roots[j]);
                        b = roots[i + 1];
                        break;
                    }
                }
            }
            next.push_back(create_parent(tree, segments, a, b, external_target,
                                         problem, common::NodeKind::ClusterAccess));
        }
        roots = std::move(next);
    }
    return roots[0];
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_output(const common::TopoTree& tree) {
    if (!g_debug_enabled) return;
    std::cout << "[PARTREER] valid=" << tree.valid
              << " root=" << tree.root
              << " cluster_root=" << tree.cluster_root
              << " nodes=" << tree.nodes.size()
              << " error=" << tree.error_msg << "\n";
    int sinks = 0;
    int cluster_internal = 0;
    int cluster_access = 0;
    int global = 0;
    for (const common::TopoNode& node : tree.nodes) {
        if (node.kind == common::NodeKind::Sink) ++sinks;
        if (node.kind == common::NodeKind::ClusterInternal) ++cluster_internal;
        if (node.kind == common::NodeKind::ClusterAccess) ++cluster_access;
        if (node.kind == common::NodeKind::Global) ++global;
        std::cout << "[PARTREER] node=" << node.id
                  << " kind=" << kind_to_string(node.kind)
                  << " skew=" << node.subtree_skew_to_node << "\n";
    }
    std::cout << "[PARTREER] kind_stats SINK=" << sinks
              << " CLUSTER_INTERNAL=" << cluster_internal
              << " CLUSTER_ACCESS=" << cluster_access
              << " GLOBAL=" << global << "\n";
}

common::TopoTree build(const common::Problem& problem,
                       const std::vector<int>& sink_indices,
                       common::Point external_target) {
    common::TopoTree tree;
    if (!problem.valid) {
        tree.error_msg = "Cannot build local topology from invalid problem";
        return tree;
    }
    if (sink_indices.empty()) {
        tree.error_msg = "Cannot build local topology with zero sinks";
        return tree;
    }

    std::vector<int> sorted_sinks = sink_indices;
    std::sort(sorted_sinks.begin(), sorted_sinks.end());
    tree.nodes.reserve(sorted_sinks.size() * 2U);

    std::vector<int> active;
    for (int sink_index : sorted_sinks) {
        if (sink_index < 0 ||
            static_cast<std::size_t>(sink_index) >= problem.sinks.size()) {
            tree.error_msg = "Local topology sink index out of range";
            return tree;
        }
        const int id = static_cast<int>(tree.nodes.size());
        tree.nodes.push_back(make_sink_node(id, sink_index, problem));
        active.push_back(id);
    }

    std::vector<Segment> segments;
    for (int level = 0;
         level < kMaxPairLevels && active.size() > static_cast<std::size_t>(kTargetAccessPoints);
         ++level) {
        std::vector<int> next = pairing_level(active, tree, segments,
                                              external_target, problem, true);
        if (next.size() >= active.size()) break;
        active = std::move(next);
    }

    while (active.size() > static_cast<std::size_t>(kTargetAccessPoints)) {
        std::vector<int> next = pairing_level(active, tree, segments,
                                              external_target, problem, true);
        if (next.size() >= active.size()) break;
        active = std::move(next);
    }

    const int root = build_balanced_access(active, tree, segments,
                                           external_target, problem);
    if (root < 0) {
        tree.error_msg = "Failed to create local cluster root";
        return tree;
    }

    tree.root = root;
    tree.cluster_root = root;
    tree.nodes[static_cast<std::size_t>(root)].parent = -1;
    tree.valid = true;
    debug_output(tree);
    return tree;
}

}  // namespace partreer
