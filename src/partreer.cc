#include "partreer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <map>
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

struct MatchingPlan {
    std::vector<Candidate> pairs;
    std::vector<int> carry;
    double cost = 0.0;
};

static double pair_cost(const common::TopoNode& a,
                        const common::TopoNode& b,
                        const std::vector<int>& active,
                        const common::TopoTree& tree);

static const char* kind_to_string(common::NodeKind kind) {
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

static bool subtree_contains(const common::TopoTree& tree, int root_id, int node_id) {
    if (root_id < 0 || node_id < 0 ||
        static_cast<std::size_t>(root_id) >= tree.nodes.size()) {
        return false;
    }
    if (root_id == node_id) return true;
    const common::TopoNode& root = tree.nodes[static_cast<std::size_t>(root_id)];
    return subtree_contains(tree, root.left, node_id) ||
           subtree_contains(tree, root.right, node_id);
}

static bool segment_inside_subtree(const common::TopoTree& tree,
                                   const Segment& segment,
                                   int root_id) {
    return subtree_contains(tree, root_id, segment.a) &&
           subtree_contains(tree, root_id, segment.b);
}

static bool connectable_segment_to_point_for_access(
    const common::TopoTree& tree,
    const std::vector<Segment>& segments,
    int endpoint_id,
    int other_endpoint_id,
    common::Point other_point) {
    const common::TopoNode& endpoint =
        tree.nodes[static_cast<std::size_t>(endpoint_id)];
    if (same_point(endpoint.loc, other_point)) {
        return true;
    }

    for (const Segment& seg : segments) {
        if (segment_inside_subtree(tree, seg, endpoint_id) ||
            segment_inside_subtree(tree, seg, other_endpoint_id)) {
            continue;
        }
        const common::TopoNode& c = tree.nodes[static_cast<std::size_t>(seg.a)];
        const common::TopoNode& d = tree.nodes[static_cast<std::size_t>(seg.b)];
        if (!segments_intersect(endpoint.loc, other_point, c.loc, d.loc)) {
            continue;
        }

        const bool shared_c = endpoint.id == c.id && same_point(endpoint.loc, c.loc);
        const bool shared_d = endpoint.id == d.id && same_point(endpoint.loc, d.loc);
        if ((shared_c || shared_d) &&
            !collinear_overlap(endpoint.loc, other_point, c.loc, d.loc)) {
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

static void add_unique_point(std::vector<common::Point>& points,
                             common::Point point,
                             const common::Problem& problem) {
    point.x = clamp_int(point.x, 0, problem.die_width);
    point.y = clamp_int(point.y, 0, problem.die_height);
    for (const common::Point& existing : points) {
        if (same_point(existing, point)) return;
    }
    points.push_back(point);
}

static bool actual_parent_segments_connectable(const common::TopoTree& tree,
                                               const std::vector<Segment>& segments,
                                               int left_id,
                                               int right_id,
                                               common::Point parent_loc,
                                               bool access_connection) {
    if (access_connection) {
        return connectable_segment_to_point_for_access(tree, segments, left_id,
                                                       right_id, parent_loc) &&
               connectable_segment_to_point_for_access(tree, segments, right_id,
                                                       left_id, parent_loc);
    }
    return connectable_segment_to_point(tree, segments, left_id, parent_loc) &&
           connectable_segment_to_point(tree, segments, right_id, parent_loc);
}

static bool choose_legal_parent_loc(const common::TopoTree& tree,
                                    const std::vector<Segment>& segments,
                                    int left_id,
                                    int right_id,
                                    common::Point external_target,
                                    const common::Problem& problem,
                                    bool require_candidate_segment,
                                    common::Point& loc) {
    if (require_candidate_segment &&
        !connectable_segment(tree, segments, left_id, right_id)) {
        if (g_debug_enabled) {
            std::cout << "[PARTREER] reject candidate segment "
                      << left_id << " " << right_id << "\n";
        }
        return false;
    }

    const common::TopoNode& left = tree.nodes[static_cast<std::size_t>(left_id)];
    const common::TopoNode& right = tree.nodes[static_cast<std::size_t>(right_id)];
    std::vector<common::Point> candidates;
    const common::Point biased = midpoint_with_bias(left, right, external_target, problem);
    const common::Point midpoint = raw_midpoint(left, right, problem);
    add_unique_point(candidates, biased, problem);
    add_unique_point(candidates, midpoint, problem);
    add_unique_point(candidates, left.loc, problem);
    add_unique_point(candidates, right.loc, problem);
    add_unique_point(candidates, external_target, problem);
    add_unique_point(candidates, {left.loc.x, right.loc.y}, problem);
    add_unique_point(candidates, {right.loc.x, left.loc.y}, problem);
    add_unique_point(candidates, {external_target.x, left.loc.y}, problem);
    add_unique_point(candidates, {external_target.x, right.loc.y}, problem);
    add_unique_point(candidates, {left.loc.x, external_target.y}, problem);
    add_unique_point(candidates, {right.loc.x, external_target.y}, problem);

    const common::BBox merged_bbox = union_bbox(left.bbox, right.bbox);
    add_unique_point(candidates, {merged_bbox.lx, merged_bbox.ly}, problem);
    add_unique_point(candidates, {merged_bbox.lx, merged_bbox.uy}, problem);
    add_unique_point(candidates, {merged_bbox.ux, merged_bbox.ly}, problem);
    add_unique_point(candidates, {merged_bbox.ux, merged_bbox.uy}, problem);

    const int outside_x = external_target.x < midpoint.x ? merged_bbox.lx - 1 :
                          external_target.x > midpoint.x ? merged_bbox.ux + 1 :
                          midpoint.x;
    const int outside_y = external_target.y < midpoint.y ? merged_bbox.ly - 1 :
                          external_target.y > midpoint.y ? merged_bbox.uy + 1 :
                          midpoint.y;
    add_unique_point(candidates, {outside_x, midpoint.y}, problem);
    add_unique_point(candidates, {midpoint.x, outside_y}, problem);
    add_unique_point(candidates, {outside_x, outside_y}, problem);
    add_unique_point(candidates, {outside_x, left.loc.y}, problem);
    add_unique_point(candidates, {outside_x, right.loc.y}, problem);
    add_unique_point(candidates, {left.loc.x, outside_y}, problem);
    add_unique_point(candidates, {right.loc.x, outside_y}, problem);

    add_unique_point(candidates, {0, 0}, problem);
    add_unique_point(candidates, {0, problem.die_height}, problem);
    add_unique_point(candidates, {problem.die_width, 0}, problem);
    add_unique_point(candidates, {problem.die_width, problem.die_height}, problem);

    for (const common::Point& candidate : candidates) {
        if (actual_parent_segments_connectable(tree, segments, left_id, right_id,
                                               candidate,
                                               !require_candidate_segment)) {
            loc = candidate;
            return true;
        }
    }
    if (g_debug_enabled) {
        std::cout << "[PARTREER] reject parent segments "
                  << left_id << " " << right_id
                  << " biased=(" << biased.x << "," << biased.y << ")"
                  << " midpoint=(" << midpoint.x << "," << midpoint.y << ")\n";
    }
    return false;
}

static int create_parent(common::TopoTree& tree,
                         std::vector<Segment>& segments,
                         int left_id,
                         int right_id,
                         common::Point external_target,
                         const common::Problem& problem,
                         common::NodeKind kind,
                         bool require_candidate_segment) {
    const common::TopoNode left = tree.nodes[static_cast<std::size_t>(left_id)];
    const common::TopoNode right = tree.nodes[static_cast<std::size_t>(right_id)];
    common::Point parent_loc;
    if (!choose_legal_parent_loc(tree, segments, left_id, right_id,
                                 external_target, problem,
                                 require_candidate_segment, parent_loc)) {
        return -1;
    }

    common::TopoNode parent;
    parent.id = static_cast<int>(tree.nodes.size());
    parent.left = left_id;
    parent.right = right_id;
    parent.kind = kind;
    parent.loc = parent_loc;
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

static int create_unary_wrapper(common::TopoTree& tree,
                                int child_id,
                                const common::Problem& problem,
                                common::NodeKind kind) {
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
    parent.kind = kind;
    parent.left_min_delay_to_node = child.subtree_min_delay_to_node;
    parent.left_max_delay_to_node = child.subtree_max_delay_to_node;
    parent.left_skew_to_node = child.subtree_skew_to_node;
    parent.subtree_min_delay_to_node = child.subtree_min_delay_to_node;
    parent.subtree_max_delay_to_node = child.subtree_max_delay_to_node;
    parent.subtree_skew_to_node = child.subtree_skew_to_node;

    const int parent_id = parent.id;
    tree.nodes.push_back(parent);
    tree.nodes[static_cast<std::size_t>(child_id)].parent = parent_id;
    return parent_id;
}

static int ensure_access_root(common::TopoTree& tree,
                              int root,
                              const common::Problem& problem) {
    common::TopoNode& node = tree.nodes[static_cast<std::size_t>(root)];
    if (node.kind == common::NodeKind::ClusterAccess) {
        return root;
    }
    if (!node.is_sink) {
        node.kind = common::NodeKind::ClusterAccess;
        return root;
    }
    return create_unary_wrapper(tree, root, problem,
                                common::NodeKind::ClusterAccess);
}

static int build_access_tree_recursive(const std::vector<int>& roots,
                                       common::TopoTree& tree,
                                       std::vector<Segment>& segments,
                                       common::Point external_target,
                                       const common::Problem& problem) {
    if (roots.empty()) return -1;
    if (roots.size() == 1) {
        int exposed_root = roots[0];
        const common::NodeKind kind =
            tree.nodes[static_cast<std::size_t>(exposed_root)].kind;
        if (kind != common::NodeKind::ClusterAccess &&
            kind != common::NodeKind::ClusterBridge) {
            exposed_root = ensure_access_root(tree, exposed_root, problem);
        }
        return create_unary_wrapper(tree, exposed_root, problem,
                                    common::NodeKind::ClusterTop);
    }
    if (roots.size() == 2) {
        return create_parent(tree, segments, roots[0], roots[1], external_target,
                             problem, common::NodeKind::ClusterTop, false);
    }

    std::vector<Candidate> candidates;
    for (std::size_t i = 0; i < roots.size(); ++i) {
        for (std::size_t j = i + 1; j < roots.size(); ++j) {
            common::Point parent_loc;
            if (!choose_legal_parent_loc(tree, segments, roots[i], roots[j],
                                         external_target, problem, false,
                                         parent_loc)) {
                continue;
            }
            candidates.push_back({roots[i],
                                  roots[j],
                                  pair_cost(tree.nodes[static_cast<std::size_t>(roots[i])],
                                            tree.nodes[static_cast<std::size_t>(roots[j])],
                                            roots,
                                            tree)});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a,
                                                       const Candidate& b) {
        if (std::abs(a.cost - b.cost) > kEps) return a.cost < b.cost;
        if (a.i != b.i) return a.i < b.i;
        return a.j < b.j;
    });

    for (const Candidate& candidate : candidates) {
        const std::vector<common::TopoNode> saved_nodes = tree.nodes;
        const std::vector<Segment> saved_segments = segments;
        const int bridge = create_parent(tree, segments, candidate.i, candidate.j,
                                         external_target, problem,
                                         common::NodeKind::ClusterBridge, false);
        if (bridge < 0) {
            tree.nodes = saved_nodes;
            segments = saved_segments;
            continue;
        }

        std::vector<int> next;
        next.reserve(roots.size() - 1);
        next.push_back(bridge);
        for (int root : roots) {
            if (root != candidate.i && root != candidate.j) {
                next.push_back(root);
            }
        }
        std::stable_sort(next.begin() + 1, next.end(), [&](int a, int b) {
            const common::Point& pa = tree.nodes[static_cast<std::size_t>(a)].loc;
            const common::Point& pb = tree.nodes[static_cast<std::size_t>(b)].loc;
            if (pa.x != pb.x) return pa.x < pb.x;
            if (pa.y != pb.y) return pa.y < pb.y;
            return a < b;
        });

        const int root = build_access_tree_recursive(next, tree, segments,
                                                     external_target, problem);
        if (root >= 0) return root;
        tree.nodes = saved_nodes;
        segments = saved_segments;
    }
    return -1;
}

static void swallow_access_internal_wrappers(common::TopoTree& tree) {
    for (common::TopoNode& node : tree.nodes) {
        const bool can_swallow =
            (node.kind == common::NodeKind::ClusterAccess ||
             node.kind == common::NodeKind::ClusterTop) &&
            node.left >= 0 &&
            node.right == -1 &&
            static_cast<std::size_t>(node.left) < tree.nodes.size();
        if (!can_swallow) continue;

        common::TopoNode& child = tree.nodes[static_cast<std::size_t>(node.left)];
        if (child.kind != common::NodeKind::ClusterInternal ||
            child.left < 0 ||
            child.right < 0) {
            continue;
        }

        node.left = child.left;
        node.right = child.right;
        tree.nodes[static_cast<std::size_t>(node.left)].parent = node.id;
        tree.nodes[static_cast<std::size_t>(node.right)].parent = node.id;
        child.parent = -1;
    }
}

static bool parent_absorbs_child(common::NodeKind parent,
                                 common::NodeKind child) {
    if (parent == common::NodeKind::ClusterTop) {
        return child == common::NodeKind::ClusterBridge ||
               child == common::NodeKind::ClusterAccess ||
               child == common::NodeKind::ClusterInternal;
    }
    if (parent == common::NodeKind::ClusterBridge) {
        return child == common::NodeKind::ClusterAccess ||
               child == common::NodeKind::ClusterInternal;
    }
    if (parent == common::NodeKind::ClusterAccess) {
        return child == common::NodeKind::ClusterInternal;
    }
    return false;
}

static bool child_replaces_parent(common::NodeKind parent,
                                  common::NodeKind child) {
    return child == common::NodeKind::Sink &&
           (parent == common::NodeKind::ClusterAccess ||
            parent == common::NodeKind::ClusterInternal);
}

static void replace_child_link(common::TopoTree& tree,
                               int parent_id,
                               int old_child,
                               int new_child) {
    if (parent_id < 0) return;
    common::TopoNode& parent = tree.nodes[static_cast<std::size_t>(parent_id)];
    if (parent.left == old_child) parent.left = new_child;
    if (parent.right == old_child) parent.right = new_child;
}

static bool canonicalize_one_unary(common::TopoTree& tree, int& root) {
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        common::TopoNode& parent = tree.nodes[i];
        if (parent.left < 0 || parent.right >= 0 ||
            static_cast<std::size_t>(parent.left) >= tree.nodes.size()) {
            continue;
        }

        common::TopoNode& child = tree.nodes[static_cast<std::size_t>(parent.left)];
        if (parent_absorbs_child(parent.kind, child.kind)) {
            const int old_child = child.id;
            parent.left = child.left;
            parent.right = child.right;
            if (parent.left >= 0) {
                tree.nodes[static_cast<std::size_t>(parent.left)].parent = parent.id;
            }
            if (parent.right >= 0) {
                tree.nodes[static_cast<std::size_t>(parent.right)].parent = parent.id;
            }
            child.parent = -1;
            child.left = -1;
            child.right = -1;
            if (g_debug_enabled) {
                std::cout << "[PARTREER] canonicalize absorb parent="
                          << parent.id << " child=" << old_child << "\n";
            }
            return true;
        }

        if (child_replaces_parent(parent.kind, child.kind)) {
            const int parent_id = parent.id;
            const int grandparent = parent.parent;
            child.parent = grandparent;
            if (grandparent >= 0) {
                replace_child_link(tree, grandparent, parent_id, child.id);
            } else {
                root = child.id;
            }
            parent.parent = -1;
            parent.left = -1;
            parent.right = -1;
            if (g_debug_enabled) {
                std::cout << "[PARTREER] canonicalize replace parent="
                          << parent_id << " child=" << child.id << "\n";
            }
            return true;
        }
    }
    return false;
}

static void canonicalize_unary_nodes(common::TopoTree& tree, int& root) {
    while (canonicalize_one_unary(tree, root)) {
    }

    if (!g_debug_enabled) return;
    for (const common::TopoNode& node : tree.nodes) {
        if (node.left >= 0 && node.right == -1) {
            const common::TopoNode& child =
                tree.nodes[static_cast<std::size_t>(node.left)];
            if (!parent_absorbs_child(node.kind, child.kind) &&
                !child_replaces_parent(node.kind, child.kind)) {
                std::cout << "[PARTREER] canonicalize keep unary parent="
                          << node.id << " kind=" << kind_to_string(node.kind)
                          << " child=" << child.id
                          << " child_kind=" << kind_to_string(child.kind) << "\n";
            }
        }
    }
}

static void collect_reachable_nodes(const common::TopoTree& tree,
                                    int node_id,
                                    std::vector<int>& order,
                                    std::vector<int>& visited) {
    if (node_id < 0 ||
        static_cast<std::size_t>(node_id) >= tree.nodes.size() ||
        visited[static_cast<std::size_t>(node_id)] != 0) {
        return;
    }

    visited[static_cast<std::size_t>(node_id)] = 1;
    order.push_back(node_id);
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    collect_reachable_nodes(tree, node.left, order, visited);
    collect_reachable_nodes(tree, node.right, order, visited);
}

static void compact_to_reachable(common::TopoTree& tree, int root) {
    if (root < 0 || static_cast<std::size_t>(root) >= tree.nodes.size()) return;

    std::vector<int> order;
    std::vector<int> visited(tree.nodes.size(), 0);
    collect_reachable_nodes(tree, root, order, visited);

    std::vector<int> remap(tree.nodes.size(), -1);
    for (std::size_t i = 0; i < order.size(); ++i) {
        remap[static_cast<std::size_t>(order[i])] = static_cast<int>(i);
    }

    std::vector<common::TopoNode> compacted;
    compacted.reserve(order.size());
    for (int old_id : order) {
        common::TopoNode node = tree.nodes[static_cast<std::size_t>(old_id)];
        node.id = remap[static_cast<std::size_t>(old_id)];
        node.parent = node.parent >= 0 ? remap[static_cast<std::size_t>(node.parent)] : -1;
        node.left = node.left >= 0 ? remap[static_cast<std::size_t>(node.left)] : -1;
        node.right = node.right >= 0 ? remap[static_cast<std::size_t>(node.right)] : -1;
        compacted.push_back(std::move(node));
    }

    tree.nodes = std::move(compacted);
    tree.root = remap[static_cast<std::size_t>(root)];
    tree.cluster_root = tree.root;
}

static void recompute_subtree(common::TopoTree& tree, int node_id) {
    common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    if (node.is_sink) {
        node.left = -1;
        node.right = -1;
        node.sink_indices = {node.sink_index};
        node.subtree_min_delay_to_node = 0;
        node.subtree_max_delay_to_node = 0;
        node.subtree_skew_to_node = 0;
        node.left_min_delay_to_node = 0;
        node.left_max_delay_to_node = 0;
        node.left_skew_to_node = 0;
        node.right_min_delay_to_node = 0;
        node.right_max_delay_to_node = 0;
        node.right_skew_to_node = 0;
        return;
    }

    if (node.left >= 0) {
        tree.nodes[static_cast<std::size_t>(node.left)].parent = node_id;
        recompute_subtree(tree, node.left);
    }
    if (node.right >= 0) {
        tree.nodes[static_cast<std::size_t>(node.right)].parent = node_id;
        recompute_subtree(tree, node.right);
    }

    const common::TopoNode left =
        tree.nodes[static_cast<std::size_t>(node.left)];
    node.bbox = left.bbox;
    node.sink_indices = left.sink_indices;
    const int left_edge = manhattan(node.loc, left.loc);
    node.left_min_delay_to_node = left.subtree_min_delay_to_node + left_edge;
    node.left_max_delay_to_node = left.subtree_max_delay_to_node + left_edge;
    node.left_skew_to_node = node.left_max_delay_to_node - node.left_min_delay_to_node;

    if (node.right >= 0) {
        const common::TopoNode right =
            tree.nodes[static_cast<std::size_t>(node.right)];
        node.bbox = union_bbox(left.bbox, right.bbox);
        node.sink_indices.insert(node.sink_indices.end(),
                                 right.sink_indices.begin(),
                                 right.sink_indices.end());
        std::sort(node.sink_indices.begin(), node.sink_indices.end());
        update_delay_fields(node, left, right);
    } else {
        node.right_min_delay_to_node = 0;
        node.right_max_delay_to_node = 0;
        node.right_skew_to_node = 0;
        node.subtree_min_delay_to_node = node.left_min_delay_to_node;
        node.subtree_max_delay_to_node = node.left_max_delay_to_node;
        node.subtree_skew_to_node = node.left_skew_to_node;
    }
}

static int canonicalize_topology(common::TopoTree& tree, int root) {
    swallow_access_internal_wrappers(tree);
    canonicalize_unary_nodes(tree, root);
    compact_to_reachable(tree, root);
    if (tree.root >= 0) {
        tree.nodes[static_cast<std::size_t>(tree.root)].parent = -1;
        recompute_subtree(tree, tree.root);
        tree.nodes[static_cast<std::size_t>(tree.root)].parent = -1;
    }
    return tree.root;
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

static bool better_matching(const MatchingPlan& a, const MatchingPlan& b) {
    if (a.pairs.size() != b.pairs.size()) return a.pairs.size() > b.pairs.size();
    if (std::abs(a.cost - b.cost) > kEps) return a.cost < b.cost;
    if (a.carry != b.carry) return a.carry < b.carry;
    for (std::size_t i = 0; i < a.pairs.size() && i < b.pairs.size(); ++i) {
        if (a.pairs[i].i != b.pairs[i].i) return a.pairs[i].i < b.pairs[i].i;
        if (a.pairs[i].j != b.pairs[i].j) return a.pairs[i].j < b.pairs[i].j;
    }
    return false;
}

static MatchingPlan solve_min_cost_matching_rec(
    unsigned long long mask,
    const std::vector<int>& active,
    const std::map<std::pair<int, int>, Candidate>& candidate_by_pair,
    std::map<unsigned long long, MatchingPlan>& memo) {
    if (mask == 0ULL) return MatchingPlan{};
    auto found = memo.find(mask);
    if (found != memo.end()) return found->second;

    int first_pos = 0;
    while (first_pos < static_cast<int>(active.size()) &&
           ((mask & (1ULL << first_pos)) == 0ULL)) {
        ++first_pos;
    }
    const int first = active[static_cast<std::size_t>(first_pos)];

    MatchingPlan best;
    best.cost = std::numeric_limits<double>::infinity();

    // Carry is allowed so odd levels keep exactly one unpaired node, and
    // disconnected candidate graphs can still advance without forcing bad pairs.
    MatchingPlan carry_plan = solve_min_cost_matching_rec(mask & ~(1ULL << first_pos),
                                                          active,
                                                          candidate_by_pair,
                                                          memo);
    carry_plan.carry.push_back(first);
    std::sort(carry_plan.carry.begin(), carry_plan.carry.end());
    best = carry_plan;

    for (int second_pos = first_pos + 1;
         second_pos < static_cast<int>(active.size());
         ++second_pos) {
        if ((mask & (1ULL << second_pos)) == 0ULL) continue;
        const int second = active[static_cast<std::size_t>(second_pos)];
        const auto key = std::minmax(first, second);
        const auto candidate_it = candidate_by_pair.find(key);
        if (candidate_it == candidate_by_pair.end()) continue;

        MatchingPlan plan = solve_min_cost_matching_rec(
            mask & ~(1ULL << first_pos) & ~(1ULL << second_pos),
            active,
            candidate_by_pair,
            memo);
        plan.pairs.push_back(candidate_it->second);
        std::sort(plan.pairs.begin(), plan.pairs.end(), [](const Candidate& a,
                                                           const Candidate& b) {
            if (a.i != b.i) return a.i < b.i;
            return a.j < b.j;
        });
        plan.cost += candidate_it->second.cost;
        if (better_matching(plan, best)) {
            best = plan;
        }
    }

    memo[mask] = best;
    return best;
}

static MatchingPlan solve_min_cost_matching(
    const std::vector<int>& active,
    const std::vector<Candidate>& candidates) {
    std::map<std::pair<int, int>, Candidate> candidate_by_pair;
    for (Candidate candidate : candidates) {
        if (candidate.i > candidate.j) std::swap(candidate.i, candidate.j);
        candidate_by_pair[{candidate.i, candidate.j}] = candidate;
    }

    std::map<unsigned long long, MatchingPlan> memo;
    const unsigned long long full_mask =
        active.size() >= 64U ? 0ULL : ((1ULL << active.size()) - 1ULL);
    return solve_min_cost_matching_rec(full_mask, active, candidate_by_pair, memo);
}

static std::vector<int> pairing_level(const std::vector<int>& active,
                                      common::TopoTree& tree,
                                      std::vector<Segment>& segments,
                                      common::Point external_target,
                                      const common::Problem& problem,
                                      bool require_connectable) {
    std::vector<Candidate> candidates = generate_candidates(active, tree);
    std::vector<Candidate> legal_candidates;
    legal_candidates.reserve(candidates.size());
    for (const Candidate& cand : candidates) {
        common::Point parent_loc;
        if (require_connectable &&
            !choose_legal_parent_loc(tree, segments, cand.i, cand.j,
                                     external_target, problem, true,
                                     parent_loc)) {
            continue;
        }
        legal_candidates.push_back(cand);
    }

    if (active.size() <= 22U) {
        MatchingPlan plan = solve_min_cost_matching(active, legal_candidates);
        std::vector<int> next;
        for (const Candidate& cand : plan.pairs) {
            const int parent = create_parent(tree, segments, cand.i, cand.j,
                                             external_target, problem,
                                             common::NodeKind::ClusterInternal,
                                             true);
            if (parent < 0) {
                next.push_back(cand.i);
                next.push_back(cand.j);
                continue;
            }
            next.push_back(parent);
            if (g_debug_enabled) {
                std::cout << "[PARTREER] pair " << cand.i << " " << cand.j
                          << " -> " << parent << " cost=" << cand.cost << "\n";
            }
        }
        next.insert(next.end(), plan.carry.begin(), plan.carry.end());
        std::sort(next.begin(), next.end(), [&](int a, int b) {
            const common::Point& pa = tree.nodes[static_cast<std::size_t>(a)].loc;
            const common::Point& pb = tree.nodes[static_cast<std::size_t>(b)].loc;
            if (pa.x != pb.x) return pa.x < pb.x;
            if (pa.y != pb.y) return pa.y < pb.y;
            return a < b;
        });
        return next;
    }

    std::vector<int> next;
    std::set<int> used;

    for (const Candidate& cand : legal_candidates) {
        if (used.count(cand.i) || used.count(cand.j)) continue;
        const int parent = create_parent(tree, segments, cand.i, cand.j,
                                         external_target, problem,
                                         common::NodeKind::ClusterInternal,
                                         true);
        if (parent < 0) continue;
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
        return build_access_tree_recursive(roots, tree, segments,
                                           external_target, problem);
    }

    std::vector<int> access_roots;
    access_roots.reserve(roots.size());
    for (int root : roots) {
        access_roots.push_back(ensure_access_root(tree, root, problem));
    }
    roots = std::move(access_roots);

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

    return build_access_tree_recursive(roots, tree, segments, external_target, problem);
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
    int cluster_bridge = 0;
    int cluster_top = 0;
    int global = 0;
    for (const common::TopoNode& node : tree.nodes) {
        if (node.kind == common::NodeKind::Sink) ++sinks;
        if (node.kind == common::NodeKind::ClusterInternal) ++cluster_internal;
        if (node.kind == common::NodeKind::ClusterAccess) ++cluster_access;
        if (node.kind == common::NodeKind::ClusterBridge) ++cluster_bridge;
        if (node.kind == common::NodeKind::ClusterTop) ++cluster_top;
        if (node.kind == common::NodeKind::Global) ++global;
        std::cout << "[PARTREER] node=" << node.id
                  << " kind=" << kind_to_string(node.kind)
                  << " skew=" << node.subtree_skew_to_node << "\n";
    }
    std::cout << "[PARTREER] kind_stats SINK=" << sinks
              << " CLUSTER_INTERNAL=" << cluster_internal
              << " CLUSTER_ACCESS=" << cluster_access
              << " CLUSTER_BRIDGE=" << cluster_bridge
              << " CLUSTER_TOP=" << cluster_top
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
        tree.error_msg = "Failed to create legal local ClusterTop root";
        return tree;
    }
    if (tree.nodes[static_cast<std::size_t>(root)].kind != common::NodeKind::ClusterTop) {
        tree.nodes[static_cast<std::size_t>(root)].kind = common::NodeKind::ClusterTop;
    }

    tree.root = root;
    tree.cluster_root = root;
    canonicalize_topology(tree, root);
    tree.valid = true;
    debug_output(tree);
    return tree;
}

}  // namespace partreer
