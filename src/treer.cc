#include "treer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <system_error>
#include <vector>

namespace treer {
namespace {

static bool g_debug_enabled = false;

static double manhattan(double x1, double y1, double x2, double y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

struct BBox {
    int lx = 0;
    int ly = 0;
    int ux = 0;
    int uy = 0;
};

struct Center {
    double x = 0.0;
    double y = 0.0;
};

static double clamp_double(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

static BBox union_node_bbox(const common::TreeNode& a, const common::TreeNode& b) {
    BBox result;
    result.lx = std::min(a.bbox_lx, b.bbox_lx);
    result.ly = std::min(a.bbox_ly, b.bbox_ly);
    result.ux = std::max(a.bbox_ux, b.bbox_ux);
    result.uy = std::max(a.bbox_uy, b.bbox_uy);
    return result;
}

static double bbox_hpwl(const BBox& bbox) {
    return static_cast<double>(bbox.ux - bbox.lx) +
           static_cast<double>(bbox.uy - bbox.ly);
}

static common::TreeNode make_leaf(int id,
                                  int sink_index,
                                  const common::Sink& sink,
                                  const common::Problem& problem) {
    common::TreeNode node;
    node.id = id;
    node.is_leaf = true;
    node.sink_index = sink_index;
    node.sink_count = 1;
    node.cx = static_cast<double>(sink.loc.x);
    node.cy = static_cast<double>(sink.loc.y);
    node.bbox_lx = sink.loc.x;
    node.bbox_ly = sink.loc.y;
    node.bbox_ux = sink.loc.x;
    node.bbox_uy = sink.loc.y;
    node.region_lx = 0;
    node.region_ly = 0;
    node.region_ux = problem.die_width;
    node.region_uy = problem.die_height;
    node.est_delay = 0.0;
    return node;
}

static Center compute_tap_point(const common::TreeNode& left,
                                const common::TreeNode& right,
                                double& parent_est_delay) {
    const double ax = left.cx;
    const double ay = left.cy;
    const double bx = right.cx;
    const double by = right.cy;
    const double D = manhattan(ax, ay, bx, by);
    const double dl = left.est_delay;
    const double dr = right.est_delay;

    Center tap;
    if (D == 0.0) {
        tap.x = ax;
        tap.y = ay;
        parent_est_delay = std::max(dl, dr);
        return tap;
    }

    double t = (D + dr - dl) / 2.0;
    t = clamp_double(t, 0.0, D);

    const double sx = (bx > ax) ? 1.0 : ((bx < ax) ? -1.0 : 0.0);
    const double sy = (by > ay) ? 1.0 : ((by < ay) ? -1.0 : 0.0);
    const double dx = std::abs(bx - ax);

    double remaining = t;
    if (remaining <= dx) {
        tap.x = ax + sx * remaining;
        tap.y = ay;
    } else {
        tap.x = bx;
        tap.y = ay + sy * (remaining - dx);
    }

    parent_est_delay = std::max(dl + t, dr + (D - t));
    return tap;
}

static common::TreeNode make_internal(int id,
                                      const common::TreeNode& left,
                                      const common::TreeNode& right,
                                      const common::Problem& problem) {
    common::TreeNode parent;
    parent.id = id;
    parent.is_leaf = false;
    parent.sink_index = -1;
    parent.left = left.id;
    parent.right = right.id;
    parent.sink_count = left.sink_count + right.sink_count;

    double parent_est_delay = 0.0;
    const Center tap = compute_tap_point(left, right, parent_est_delay);
    parent.cx = tap.x;
    parent.cy = tap.y;
    parent.est_delay = parent_est_delay;

    const BBox bbox = union_node_bbox(left, right);
    parent.bbox_lx = bbox.lx;
    parent.bbox_ly = bbox.ly;
    parent.bbox_ux = bbox.ux;
    parent.bbox_uy = bbox.uy;

    parent.region_lx = 0;
    parent.region_ly = 0;
    parent.region_ux = problem.die_width;
    parent.region_uy = problem.die_height;

    return parent;
}

static double pair_cost(const common::TreeNode& a,
                        const common::TreeNode& b,
                        const common::Problem& problem) {
    const double l1 = manhattan(a.cx, a.cy, b.cx, b.cy);
    const BBox ub = union_node_bbox(a, b);
    const double hpwl = bbox_hpwl(ub);
    const double delay_diff = std::abs(a.est_delay - b.est_delay);
    const double mid_x = (a.cx + b.cx) / 2.0;
    const double mid_y = (a.cy + b.cy) / 2.0;
    const double src_x = static_cast<double>(problem.source.loc.x);
    const double src_y = static_cast<double>(problem.source.loc.y);
    const double source_tie = manhattan(src_x, src_y, mid_x, mid_y);
    const double sink_diff = std::abs(static_cast<double>(a.sink_count) -
                                      static_cast<double>(b.sink_count));

    return 1.0 * l1 +
           0.15 * hpwl +
           0.05 * delay_diff +
           0.001 * source_tie +
           0.0001 * sink_diff;
}

static int orientation(double px, double py,
                       double qx, double qy,
                       double rx, double ry) {
    const double val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy);
    if (val == 0.0) return 0;
    return (val > 0.0) ? 1 : 2;
}

static bool on_segment(double px, double py,
                       double qx, double qy,
                       double rx, double ry) {
    return qx <= std::max(px, rx) && qx >= std::min(px, rx) &&
           qy <= std::max(py, ry) && qy >= std::min(py, ry);
}

static bool segments_properly_intersect(double p1x, double p1y,
                                        double q1x, double q1y,
                                        double p2x, double p2y,
                                        double q2x, double q2y) {
    const int o1 = orientation(p1x, p1y, q1x, q1y, p2x, p2y);
    const int o2 = orientation(p1x, p1y, q1x, q1y, q2x, q2y);
    const int o3 = orientation(p2x, p2y, q2x, q2y, p1x, p1y);
    const int o4 = orientation(p2x, p2y, q2x, q2y, q1x, q1y);

    if (o1 != o2 && o3 != o4) return true;

    if (o1 == 0 && on_segment(p1x, p1y, p2x, p2y, q1x, q1y)) return false;
    if (o2 == 0 && on_segment(p1x, p1y, q2x, q2y, q1x, q1y)) return false;
    if (o3 == 0 && on_segment(p2x, p2y, p1x, p1y, q2x, q2y)) return false;
    if (o4 == 0 && on_segment(p2x, p2y, q1x, q1y, q2x, q2y)) return false;

    return false;
}

static double point_to_segment_dist_sq(double px, double py,
                                       double ax, double ay,
                                       double bx, double by) {
    const double abx = bx - ax;
    const double aby = by - ay;
    const double apx = px - ax;
    const double apy = py - ay;
    const double len_sq = abx * abx + aby * aby;

    if (len_sq == 0.0) {
        return apx * apx + apy * apy;
    }

    double t = (apx * abx + apy * aby) / len_sq;
    t = clamp_double(t, 0.0, 1.0);

    const double proj_x = ax + t * abx;
    const double proj_y = ay + t * aby;
    const double dx = px - proj_x;
    const double dy = py - proj_y;
    return dx * dx + dy * dy;
}

static bool segment_passes_near_center(double cx, double cy,
                                       double ax, double ay,
                                       double bx, double by) {
    constexpr double kEps = 1e-9;
    const double dist_sq = point_to_segment_dist_sq(cx, cy, ax, ay, bx, by);
    if (dist_sq > kEps * kEps) return false;

    const double apx = cx - ax;
    const double apy = cy - ay;
    const double abx = bx - ax;
    const double aby = by - ay;
    const double len_sq = abx * abx + aby * aby;
    if (len_sq == 0.0) return false;

    const double dot = apx * abx + apy * aby;
    if (dot < -kEps || dot > len_sq + kEps) return false;

    return true;
}

struct RgmCandidate {
    int i;
    int j;
    double cost;
};

struct RgmSegment {
    double x1;
    double y1;
    double x2;
    double y2;
};

static std::vector<int> rgm_round(const std::vector<int>& active,
                                  const common::Problem& problem,
                                  common::TopologyTree& tree) {
    const int n = static_cast<int>(active.size());

    std::vector<RgmCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(n - 1) / 2);

    for (int i = 0; i < n; ++i) {
        const common::TreeNode& node_i = tree.nodes[static_cast<std::size_t>(active[i])];
        for (int j = i + 1; j < n; ++j) {
            const common::TreeNode& node_j = tree.nodes[static_cast<std::size_t>(active[j])];
            const double cost = pair_cost(node_i, node_j, problem);
            candidates.push_back({i, j, cost});
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [&](const RgmCandidate& a, const RgmCandidate& b) {
                  if (a.cost != b.cost) return a.cost < b.cost;
                  const int a_min = std::min(active[a.i], active[a.j]);
                  const int a_max = std::max(active[a.i], active[a.j]);
                  const int b_min = std::min(active[b.i], active[b.j]);
                  const int b_max = std::max(active[b.i], active[b.j]);
                  if (a_min != b_min) return a_min < b_min;
                  return a_max < b_max;
              });

    std::vector<bool> used(static_cast<std::size_t>(n), false);
    std::vector<RgmCandidate> selected;
    std::vector<RgmSegment> selected_segs;

    for (const RgmCandidate& cand : candidates) {
        if (used[cand.i] || used[cand.j]) continue;

        const common::TreeNode& node_i =
            tree.nodes[static_cast<std::size_t>(active[cand.i])];
        const common::TreeNode& node_j =
            tree.nodes[static_cast<std::size_t>(active[cand.j])];
        const double ax = node_i.cx;
        const double ay = node_i.cy;
        const double bx = node_j.cx;
        const double by = node_j.cy;

        bool crosses = false;
        for (const RgmSegment& seg : selected_segs) {
            if (segments_properly_intersect(
                    ax, ay, bx, by, seg.x1, seg.y1, seg.x2, seg.y2)) {
                crosses = true;
                break;
            }
        }
        if (crosses) continue;

        bool passes = false;
        for (int k = 0; k < n; ++k) {
            if (k == cand.i || k == cand.j) continue;
            const common::TreeNode& node_k =
                tree.nodes[static_cast<std::size_t>(active[k])];
            if (segment_passes_near_center(node_k.cx, node_k.cy, ax, ay, bx, by)) {
                passes = true;
                break;
            }
        }
        if (passes) continue;

        used[cand.i] = true;
        used[cand.j] = true;
        selected.push_back(cand);
        selected_segs.push_back({ax, ay, bx, by});
    }

    if (selected.empty()) {
        const RgmCandidate& fallback = candidates[0];
        used[fallback.i] = true;
        used[fallback.j] = true;
        selected.push_back(fallback);
    }

    std::vector<int> new_active;
    for (const RgmCandidate& cand : selected) {
        int id_i = active[cand.i];
        int id_j = active[cand.j];
        const common::TreeNode& node_i =
            tree.nodes[static_cast<std::size_t>(id_i)];
        const common::TreeNode& node_j =
            tree.nodes[static_cast<std::size_t>(id_j)];

        int left_id = id_i;
        int right_id = id_j;
        if (node_i.cx < node_j.cx) {
            left_id = id_i;
            right_id = id_j;
        } else if (node_i.cx > node_j.cx) {
            left_id = id_j;
            right_id = id_i;
        } else if (node_i.cy < node_j.cy) {
            left_id = id_i;
            right_id = id_j;
        } else if (node_i.cy > node_j.cy) {
            left_id = id_j;
            right_id = id_i;
        } else if (id_i < id_j) {
            left_id = id_i;
            right_id = id_j;
        } else {
            left_id = id_j;
            right_id = id_i;
        }

        const int parent_id = static_cast<int>(tree.nodes.size());
        common::TreeNode parent = make_internal(
            parent_id,
            tree.nodes[static_cast<std::size_t>(left_id)],
            tree.nodes[static_cast<std::size_t>(right_id)],
            problem);
        tree.nodes.push_back(parent);
        tree.nodes[static_cast<std::size_t>(left_id)].parent = parent_id;
        tree.nodes[static_cast<std::size_t>(right_id)].parent = parent_id;
        new_active.push_back(parent_id);
    }

    for (int i = 0; i < n; ++i) {
        if (!used[i]) {
            new_active.push_back(active[i]);
        }
    }

    return new_active;
}

static std::string get_basename(const std::string& input_path) {
    if (input_path.empty()) {
        return "tree_debug.txt";
    }
    const std::size_t pos = input_path.find_last_of("/\\");
    if (pos == std::string::npos) {
        return input_path;
    }
    if (pos + 1 >= input_path.size()) {
        return "tree_debug.txt";
    }
    return input_path.substr(pos + 1);
}

static bool ensure_tree_dir() {
    std::error_code ec;
    std::filesystem::create_directories("tree", ec);
    if (ec) {
        std::cerr << "Warning: cannot create tree directory: " << ec.message() << "\n";
        return false;
    }
    return true;
}

static void write_node_line(std::ostream& out, const common::TreeNode& node) {
    out << "NODE " << node.id << " " << node.parent << " "
        << node.left << " " << node.right << " "
        << (node.is_leaf ? 1 : 0) << " " << node.sink_index << " "
        << node.sink_count << " " << node.cx << " " << node.cy << " "
        << node.bbox_lx << " " << node.bbox_ly << " "
        << node.bbox_ux << " " << node.bbox_uy << " "
        << node.region_lx << " " << node.region_ly << " "
        << node.region_ux << " " << node.region_uy << " "
        << node.est_delay << "\n";
}

static bool write_tree_file(const common::TopologyTree& tree,
                            const common::Problem& problem,
                            const std::string& output_path) {
    std::ofstream fout(output_path);
    if (!fout) {
        std::cerr << "Warning: cannot open tree debug file: " << output_path << "\n";
        return false;
    }

    fout << "TREE_VALID " << (tree.valid ? 1 : 0) << "\n";
    fout << "ROOT " << tree.root << "\n";
    fout << "NUM_NODES " << tree.nodes.size() << "\n";
    fout << "NUM_SINKS " << problem.sinks.size() << "\n\n";

    for (const common::TreeNode& node : tree.nodes) {
        write_node_line(fout, node);
    }
    fout << "\n";

    for (const common::TreeNode& node : tree.nodes) {
        if (node.is_leaf &&
            node.sink_index >= 0 &&
            static_cast<std::size_t>(node.sink_index) < problem.sinks.size()) {
            const common::Sink& sink = problem.sinks[static_cast<std::size_t>(node.sink_index)];
            fout << "LEAF " << node.id << " " << node.sink_index << " "
                 << sink.id << " " << sink.loc.x << " " << sink.loc.y << "\n";
        }
    }
    fout << "\n";

    for (const common::TreeNode& node : tree.nodes) {
        if (!node.is_leaf) {
            fout << "EDGE " << node.id << " " << node.left << "\n";
            fout << "EDGE " << node.id << " " << node.right << "\n";
        }
    }
    return true;
}

static bool collect_subtree_sinks(int node_id,
                                  const common::TopologyTree& tree,
                                  const common::Problem& problem,
                                  std::vector<int>& node_state,
                                  std::vector<int>& sink_seen,
                                  int& reachable_nodes,
                                  std::vector<int>& sink_indices,
                                  std::string& err) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) {
        err = "Subtree references invalid node id";
        return false;
    }

    const std::size_t idx = static_cast<std::size_t>(node_id);
    if (node_state[idx] == 1) {
        err = "Cycle detected in topology tree";
        return false;
    }
    if (node_state[idx] == 2) {
        err = "Topology tree reuses a node in multiple subtrees";
        return false;
    }

    node_state[idx] = 1;
    ++reachable_nodes;
    const common::TreeNode& node = tree.nodes[idx];
    if (node.is_leaf) {
        if (node.sink_index < 0 ||
            static_cast<std::size_t>(node.sink_index) >= problem.sinks.size()) {
            err = "Leaf node has invalid sink index";
            return false;
        }
        const std::size_t sink_idx = static_cast<std::size_t>(node.sink_index);
        if (sink_seen[sink_idx] != 0) {
            err = "Duplicate sink appears in topology tree";
            return false;
        }
        sink_seen[sink_idx] = 1;
        sink_indices.push_back(node.sink_index);
    } else {
        std::vector<int> left_sinks;
        std::vector<int> right_sinks;
        if (!collect_subtree_sinks(node.left,
                                   tree,
                                   problem,
                                   node_state,
                                   sink_seen,
                                   reachable_nodes,
                                   left_sinks,
                                   err)) {
            return false;
        }
        if (!collect_subtree_sinks(node.right,
                                   tree,
                                   problem,
                                   node_state,
                                   sink_seen,
                                   reachable_nodes,
                                   right_sinks,
                                   err)) {
            return false;
        }

        sink_indices.reserve(left_sinks.size() + right_sinks.size());
        sink_indices.insert(sink_indices.end(), left_sinks.begin(), left_sinks.end());
        sink_indices.insert(sink_indices.end(), right_sinks.begin(), right_sinks.end());
        if (node.sink_count != static_cast<int>(sink_indices.size())) {
            err = "Subtree sink_count does not match descendant leaves";
            return false;
        }
    }

    node_state[idx] = 2;
    return true;
}

static bool validate_tree(const common::TopologyTree& tree,
                          const common::Problem& problem,
                          std::string& err) {
    if (tree.root < 0 || static_cast<std::size_t>(tree.root) >= tree.nodes.size()) {
        err = "Invalid tree root";
        return false;
    }
    if (tree.nodes[static_cast<std::size_t>(tree.root)].parent != -1) {
        err = "Tree root must not have a parent";
        return false;
    }

    int leaf_count = 0;
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const common::TreeNode& node = tree.nodes[i];
        if (node.id != static_cast<int>(i)) {
            err = "Tree node id does not match vector index";
            return false;
        }
        if (node.region_lx > node.region_ux || node.region_ly > node.region_uy) {
            err = "Tree node has invalid region";
            return false;
        }
        if (node.bbox_lx > node.bbox_ux || node.bbox_ly > node.bbox_uy) {
            err = "Tree node has invalid bbox";
            return false;
        }
        if (node.is_leaf) {
            ++leaf_count;
            if (node.sink_index < 0 ||
                static_cast<std::size_t>(node.sink_index) >= problem.sinks.size()) {
                err = "Leaf node has invalid sink index";
                return false;
            }
            if (node.left != -1 || node.right != -1 || node.sink_count != 1) {
                err = "Leaf node has invalid child or sink_count fields";
                return false;
            }
        } else {
            if (node.left < 0 || node.right < 0 || node.left == node.right ||
                static_cast<std::size_t>(node.left) >= tree.nodes.size() ||
                static_cast<std::size_t>(node.right) >= tree.nodes.size()) {
                err = "Internal node has invalid children";
                return false;
            }
            const common::TreeNode& left =
                tree.nodes[static_cast<std::size_t>(node.left)];
            const common::TreeNode& right =
                tree.nodes[static_cast<std::size_t>(node.right)];
            if (left.parent != node.id || right.parent != node.id) {
                err = "Child parent pointer mismatch";
                return false;
            }
            if (node.sink_count != left.sink_count + right.sink_count) {
                err = "Internal node sink_count mismatch";
                return false;
            }
            const BBox child_union = union_node_bbox(left, right);
            if (node.bbox_lx != child_union.lx ||
                node.bbox_ly != child_union.ly ||
                node.bbox_ux != child_union.ux ||
                node.bbox_uy != child_union.uy) {
                err = "Internal node bbox does not match union of children bboxes";
                return false;
            }
        }

        if (node.id != tree.root) {
            if (node.parent < 0 ||
                static_cast<std::size_t>(node.parent) >= tree.nodes.size()) {
                err = "Non-root node has invalid parent";
                return false;
            }
        }
    }

    if (leaf_count != static_cast<int>(problem.sinks.size())) {
        err = "Leaf count does not match sink count";
        return false;
    }

    std::vector<int> node_state(tree.nodes.size(), 0);
    std::vector<int> sink_seen(problem.sinks.size(), 0);
    std::vector<int> root_sinks;
    int reachable_nodes = 0;
    if (!collect_subtree_sinks(tree.root,
                               tree,
                               problem,
                               node_state,
                               sink_seen,
                               reachable_nodes,
                               root_sinks,
                               err)) {
        return false;
    }
    if (reachable_nodes != static_cast<int>(tree.nodes.size())) {
        err = "Topology tree contains nodes unreachable from root";
        return false;
    }
    if (root_sinks.size() != problem.sinks.size()) {
        err = "Root subtree does not contain every sink";
        return false;
    }
    for (int seen : sink_seen) {
        if (seen != 1) {
            err = "Topology tree is missing at least one sink";
            return false;
        }
    }
    return true;
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_output(const TopologyTree& tree, const common::Problem& problem) {
    if (!g_debug_enabled) {
        return;
    }

    std::cout << "TREER_DEBUG\n";
    std::cout << "VALID " << (tree.valid ? 1 : 0) << "\n";
    std::cout << "ERROR_MSG " << tree.error_msg << "\n";
    std::cout << "ROOT " << tree.root << "\n";
    std::cout << "NUM_NODES " << tree.nodes.size() << "\n";
    for (const common::TreeNode& node : tree.nodes) {
        write_node_line(std::cout, node);
        if (node.is_leaf &&
            node.sink_index >= 0 &&
            static_cast<std::size_t>(node.sink_index) < problem.sinks.size()) {
            const common::Sink& sink = problem.sinks[static_cast<std::size_t>(node.sink_index)];
            std::cout << "  LEAF_SINK " << sink.id << " "
                      << sink.loc.x << " " << sink.loc.y << "\n";
        }
    }
    std::cout << "END_TREER_DEBUG\n";
}

void debug_output_file(const TopologyTree& tree,
                       const common::Problem& problem,
                       const std::string& input_path) {
    if (!g_debug_enabled) {
        return;
    }
    if (!ensure_tree_dir()) {
        return;
    }

    const std::string output_path = "tree/" + get_basename(input_path);
    write_tree_file(tree, problem, output_path);
}

TopologyTree build(const common::Problem& problem, const std::string& input_path) {
    TopologyTree tree;
    if (!problem.valid) {
        tree.error_msg = "Cannot build topology from invalid problem: " + problem.error_msg;
        return tree;
    }
    if (problem.sinks.empty()) {
        tree.error_msg = "Cannot build topology with zero sinks";
        return tree;
    }

    const std::size_t num_sinks = problem.sinks.size();

    if (num_sinks == 1) {
        const int leaf_id = 0;
        tree.nodes.push_back(make_leaf(leaf_id, 0, problem.sinks[0], problem));
        tree.root = leaf_id;
        tree.valid = true;

        if (g_debug_enabled) {
            debug_output(tree, problem);
            debug_output_file(tree, problem, input_path);
        }
        return tree;
    }

    tree.nodes.reserve(num_sinks * 2U - 1U);

    std::vector<int> active;
    active.reserve(num_sinks);
    for (std::size_t i = 0; i < num_sinks; ++i) {
        const int leaf_id = static_cast<int>(tree.nodes.size());
        tree.nodes.push_back(make_leaf(
            leaf_id,
            static_cast<int>(i),
            problem.sinks[i],
            problem));
        active.push_back(leaf_id);
    }

    while (active.size() > 1) {
        std::vector<int> new_active = rgm_round(active, problem, tree);
        if (new_active.size() >= active.size()) {
            tree.error_msg = "RGM round failed to reduce active cluster count";
            return tree;
        }
        active = std::move(new_active);
    }

    tree.root = active[0];
    tree.nodes[static_cast<std::size_t>(tree.root)].parent = -1;
    tree.valid = true;

    std::string err;
    if (!validate_tree(tree, problem, err)) {
        tree.valid = false;
        tree.error_msg = err;
        return tree;
    }

    if (g_debug_enabled) {
        debug_output(tree, problem);
        debug_output_file(tree, problem, input_path);
    }
    return tree;
}

}  // namespace treer
