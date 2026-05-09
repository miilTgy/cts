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

enum class Axis {
    kX,
    kY,
};

struct BBox {
    int lx = 0;
    int ly = 0;
    int ux = 0;
    int uy = 0;
};

using Region = BBox;

struct Center {
    double x = 0.0;
    double y = 0.0;
};

static int clamp_int(int value, int lo, int hi) {
    return std::max(lo, std::min(value, hi));
}

static double clamp_double(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

static bool is_valid_region(const Region& region) {
    return region.lx <= region.ux && region.ly <= region.uy;
}

static Region root_region_from_problem(const common::Problem& problem) {
    Region region;
    region.lx = 0;
    region.ly = 0;
    region.ux = problem.die_width;
    region.uy = problem.die_height;
    return region;
}

static common::TreeNode make_leaf(int id,
                                  int sink_index,
                                  const common::Sink& sink,
                                  const Region& region) {
    common::TreeNode node;
    node.id = id;
    node.is_leaf = true;
    node.sink_index = sink_index;
    node.sink_count = 1;
    node.cx = sink.loc.x;
    node.cy = sink.loc.y;
    node.bbox_lx = sink.loc.x;
    node.bbox_ly = sink.loc.y;
    node.bbox_ux = sink.loc.x;
    node.bbox_uy = sink.loc.y;
    node.region_lx = region.lx;
    node.region_ly = region.ly;
    node.region_ux = region.ux;
    node.region_uy = region.uy;
    node.est_delay = 0.0;
    return node;
}

static common::TreeNode make_internal(int id,
                                      const common::TreeNode& left,
                                      const common::TreeNode& right,
                                      const BBox& bbox,
                                      const Region& region,
                                      const Center& abstract_center) {
    common::TreeNode parent;
    parent.id = id;
    parent.is_leaf = false;
    parent.sink_index = -1;
    parent.left = left.id;
    parent.right = right.id;
    parent.sink_count = left.sink_count + right.sink_count;

    parent.cx = abstract_center.x;
    parent.cy = abstract_center.y;
    parent.bbox_lx = bbox.lx;
    parent.bbox_ly = bbox.ly;
    parent.bbox_ux = bbox.ux;
    parent.bbox_uy = bbox.uy;
    parent.region_lx = region.lx;
    parent.region_ly = region.ly;
    parent.region_ux = region.ux;
    parent.region_uy = region.uy;

    const double d = manhattan(left.cx, left.cy, right.cx, right.cy);
    const double dl = left.est_delay;
    const double dr = right.est_delay;
    parent.est_delay = std::max(dl, dr) + std::max(0.0, (d - std::abs(dl - dr)) / 2.0);
    return parent;
}

static BBox compute_bbox(const std::vector<int>& indices, const common::Problem& problem) {
    BBox bbox;
    if (indices.empty()) {
        return bbox;
    }

    bbox.lx = std::numeric_limits<int>::max();
    bbox.ly = std::numeric_limits<int>::max();
    bbox.ux = std::numeric_limits<int>::min();
    bbox.uy = std::numeric_limits<int>::min();
    for (int sink_index : indices) {
        const common::Point& loc = problem.sinks[static_cast<std::size_t>(sink_index)].loc;
        bbox.lx = std::min(bbox.lx, loc.x);
        bbox.ly = std::min(bbox.ly, loc.y);
        bbox.ux = std::max(bbox.ux, loc.x);
        bbox.uy = std::max(bbox.uy, loc.y);
    }
    return bbox;
}

static Center compute_cog(const std::vector<int>& indices, const common::Problem& problem) {
    Center cog;
    if (indices.empty()) {
        return cog;
    }

    double sx = 0.0;
    double sy = 0.0;
    for (int sink_index : indices) {
        const common::Point& loc = problem.sinks[static_cast<std::size_t>(sink_index)].loc;
        sx += loc.x;
        sy += loc.y;
    }
    const double count = static_cast<double>(indices.size());
    cog.x = sx / count;
    cog.y = sy / count;
    return cog;
}

static Axis choose_split_axis(const Region& region, const BBox& bbox) {
    const int region_width = region.ux - region.lx;
    const int region_height = region.uy - region.ly;
    if (region_width > 0 || region_height > 0) {
        return region_width >= region_height ? Axis::kX : Axis::kY;
    }

    const int width = bbox.ux - bbox.lx;
    const int height = bbox.uy - bbox.ly;
    return width >= height ? Axis::kX : Axis::kY;
}

static std::vector<int> sort_indices(std::vector<int> indices,
                                     Axis axis,
                                     const common::Problem& problem) {
    std::sort(indices.begin(), indices.end(), [&](int lhs, int rhs) {
        const common::Point& a = problem.sinks[static_cast<std::size_t>(lhs)].loc;
        const common::Point& b = problem.sinks[static_cast<std::size_t>(rhs)].loc;
        if (axis == Axis::kX) {
            if (a.x != b.x) {
                return a.x < b.x;
            }
            if (a.y != b.y) {
                return a.y < b.y;
            }
            return lhs < rhs;
        }

        if (a.y != b.y) {
            return a.y < b.y;
        }
        if (a.x != b.x) {
            return a.x < b.x;
        }
        return lhs < rhs;
    });
    return indices;
}

static double bbox_hpwl(const BBox& bbox) {
    return static_cast<double>(bbox.ux - bbox.lx) +
           static_cast<double>(bbox.uy - bbox.ly);
}

static double split_cost(const std::vector<int>& left,
                         const std::vector<int>& right,
                         const common::Problem& problem) {
    constexpr double kBalanceWeight = 1000000.0;
    constexpr double kHpwlWeight = 1.0;
    constexpr double kSourceBiasWeight = 0.05;
    constexpr double kDelayBiasWeight = 0.01;

    const BBox left_bbox = compute_bbox(left, problem);
    const BBox right_bbox = compute_bbox(right, problem);
    const Center left_cog = compute_cog(left, problem);
    const Center right_cog = compute_cog(right, problem);

    const double balance =
        std::abs(static_cast<double>(left.size()) - static_cast<double>(right.size()));
    const double hpwl = bbox_hpwl(left_bbox) + bbox_hpwl(right_bbox);
    const double source_left =
        manhattan(problem.source.loc.x, problem.source.loc.y, left_cog.x, left_cog.y);
    const double source_right =
        manhattan(problem.source.loc.x, problem.source.loc.y, right_cog.x, right_cog.y);
    const double source_bias = std::abs(source_left - source_right);
    const double est_delay_bias = 0.0;

    return kBalanceWeight * balance +
           kHpwlWeight * hpwl +
           kSourceBiasWeight * source_bias +
           kDelayBiasWeight * est_delay_bias;
}

static std::size_t choose_best_median_split(const std::vector<int>& sorted,
                                            const common::Problem& problem) {
    const std::size_t n = sorted.size();
    const std::size_t mid = n / 2U;
    const std::size_t first = mid > 2U ? mid - 2U : 1U;
    const std::size_t last = std::min(n - 1U, mid + 2U);

    double best_score = std::numeric_limits<double>::infinity();
    std::size_t best_k = 0U;
    for (std::size_t k = first; k <= last; ++k) {
        std::vector<int> left(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(k));
        std::vector<int> right(sorted.begin() + static_cast<std::ptrdiff_t>(k), sorted.end());
        const double score = split_cost(left, right, problem);
        const std::size_t distance = k > mid ? k - mid : mid - k;
        const std::size_t best_distance = best_k > mid ? best_k - mid : mid - best_k;

        constexpr double kEps = 1e-9;
        if (score < best_score - kEps ||
            (std::abs(score - best_score) <= kEps &&
             (best_k == 0U || distance < best_distance ||
              (distance == best_distance && k < best_k)))) {
            best_score = score;
            best_k = k;
        }
    }
    return best_k;
}

static int axis_coord(int sink_index, Axis axis, const common::Problem& problem) {
    const common::Point& loc = problem.sinks[static_cast<std::size_t>(sink_index)].loc;
    return axis == Axis::kX ? loc.x : loc.y;
}

static int choose_split_coordinate(const std::vector<int>& sorted,
                                   std::size_t split,
                                   Axis axis,
                                   const Region& region,
                                   const common::Problem& problem) {
    const int left_coord = axis_coord(sorted[split - 1U], axis, problem);
    const int right_coord = axis_coord(sorted[split], axis, problem);
    const int raw = static_cast<int>(std::floor((left_coord + right_coord) / 2.0));
    if (axis == Axis::kX) {
        return clamp_int(raw, region.lx, region.ux);
    }
    return clamp_int(raw, region.ly, region.uy);
}

static void split_region(const Region& region,
                         Axis axis,
                         int split_coord,
                         Region& left_region,
                         Region& right_region) {
    left_region = region;
    right_region = region;
    if (axis == Axis::kX) {
        left_region.ux = split_coord;
        right_region.lx = split_coord;
    } else {
        left_region.uy = split_coord;
        right_region.ly = split_coord;
    }
}

static Center choose_separator_center(const Region& region,
                                      Axis axis,
                                      int split_coord,
                                      const Center& cog) {
    Center center;
    if (axis == Axis::kX) {
        center.x = split_coord;
        center.y = clamp_double(cog.y, region.ly, region.uy);
    } else {
        center.x = clamp_double(cog.x, region.lx, region.ux);
        center.y = split_coord;
    }
    return center;
}

static int build_subtree(const std::vector<int>& indices,
                         const Region& region,
                         const common::Problem& problem,
                         TopologyTree& tree,
                         std::string& err) {
    if (indices.empty()) {
        err = "Cannot build subtree from empty sink set";
        return -1;
    }
    if (!is_valid_region(region)) {
        err = "Cannot build subtree from invalid region";
        return -1;
    }

    if (indices.size() == 1U) {
        const int sink_index = indices[0];
        if (sink_index < 0 ||
            static_cast<std::size_t>(sink_index) >= problem.sinks.size()) {
            err = "Subtree contains invalid sink index";
            return -1;
        }

        const int id = static_cast<int>(tree.nodes.size());
        tree.nodes.push_back(make_leaf(
            id,
            sink_index,
            problem.sinks[static_cast<std::size_t>(sink_index)],
            region));
        return id;
    }

    const BBox bbox = compute_bbox(indices, problem);
    const Center cog = compute_cog(indices, problem);
    const Axis axis = choose_split_axis(region, bbox);
    const std::vector<int> sorted = sort_indices(indices, axis, problem);
    const std::size_t split = choose_best_median_split(sorted, problem);
    if (split == 0U || split >= sorted.size()) {
        err = "Failed to choose non-empty median split";
        return -1;
    }

    const std::vector<int> left_indices(sorted.begin(),
                                        sorted.begin() + static_cast<std::ptrdiff_t>(split));
    const std::vector<int> right_indices(sorted.begin() + static_cast<std::ptrdiff_t>(split),
                                         sorted.end());

    const int split_coord = choose_split_coordinate(sorted, split, axis, region, problem);
    Region left_region;
    Region right_region;
    split_region(region, axis, split_coord, left_region, right_region);

    const int left_id = build_subtree(left_indices, left_region, problem, tree, err);
    if (left_id < 0) {
        return -1;
    }
    const int right_id = build_subtree(right_indices, right_region, problem, tree, err);
    if (right_id < 0) {
        return -1;
    }

    const Center abstract_center = choose_separator_center(region, axis, split_coord, cog);
    const int parent_id = static_cast<int>(tree.nodes.size());
    common::TreeNode parent = make_internal(
        parent_id,
        tree.nodes[static_cast<std::size_t>(left_id)],
        tree.nodes[static_cast<std::size_t>(right_id)],
        bbox,
        region,
        abstract_center);
    tree.nodes.push_back(parent);
    tree.nodes[static_cast<std::size_t>(left_id)].parent = parent_id;
    tree.nodes[static_cast<std::size_t>(right_id)].parent = parent_id;
    return parent_id;
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

static Region node_region(const common::TreeNode& node) {
    Region region;
    region.lx = node.region_lx;
    region.ly = node.region_ly;
    region.ux = node.region_ux;
    region.uy = node.region_uy;
    return region;
}

static bool same_region(const Region& a, const Region& b) {
    return a.lx == b.lx && a.ly == b.ly && a.ux == b.ux && a.uy == b.uy;
}

static bool children_match_x_split(const Region& parent,
                                   const Region& left,
                                   const Region& right) {
    return left.lx == parent.lx &&
           left.ly == parent.ly &&
           left.uy == parent.uy &&
           right.ux == parent.ux &&
           right.ly == parent.ly &&
           right.uy == parent.uy &&
           left.ux == right.lx &&
           left.ux >= parent.lx &&
           left.ux <= parent.ux;
}

static bool children_match_y_split(const Region& parent,
                                   const Region& left,
                                   const Region& right) {
    return left.lx == parent.lx &&
           left.ly == parent.ly &&
           left.ux == parent.ux &&
           right.lx == parent.lx &&
           right.ux == parent.ux &&
           right.uy == parent.uy &&
           left.uy == right.ly &&
           left.uy >= parent.ly &&
           left.uy <= parent.uy;
}

static bool child_regions_match_slicing(const common::TreeNode& parent,
                                        const common::TreeNode& left,
                                        const common::TreeNode& right) {
    const Region parent_region = node_region(parent);
    const Region left_region = node_region(left);
    const Region right_region = node_region(right);
    if (!is_valid_region(parent_region) ||
        !is_valid_region(left_region) ||
        !is_valid_region(right_region)) {
        return false;
    }
    if (same_region(parent_region, left_region) &&
        same_region(parent_region, right_region)) {
        return parent_region.lx == parent_region.ux ||
               parent_region.ly == parent_region.uy;
    }
    return children_match_x_split(parent_region, left_region, right_region) ||
           children_match_y_split(parent_region, left_region, right_region);
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
    if (!same_region(node_region(tree.nodes[static_cast<std::size_t>(tree.root)]),
                     root_region_from_problem(problem))) {
        err = "Tree root region does not match die region";
        return false;
    }

    int leaf_count = 0;
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const common::TreeNode& node = tree.nodes[i];
        if (node.id != static_cast<int>(i)) {
            err = "Tree node id does not match vector index";
            return false;
        }
        if (!is_valid_region(node_region(node))) {
            err = "Tree node has invalid region";
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
            const common::TreeNode& left = tree.nodes[static_cast<std::size_t>(node.left)];
            const common::TreeNode& right = tree.nodes[static_cast<std::size_t>(node.right)];
            if (left.parent != node.id || right.parent != node.id) {
                err = "Child parent pointer mismatch";
                return false;
            }
            if (node.sink_count != left.sink_count + right.sink_count) {
                err = "Internal node sink_count mismatch";
                return false;
            }
            if (!child_regions_match_slicing(node, left, right)) {
                err = "Internal node child regions do not match a slicing split";
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

    std::vector<int> indices;
    indices.reserve(problem.sinks.size());
    tree.nodes.reserve(problem.sinks.size() * 2U - 1U);
    for (std::size_t i = 0; i < problem.sinks.size(); ++i) {
        indices.push_back(static_cast<int>(i));
    }

    std::string build_err;
    const Region root_region = root_region_from_problem(problem);
    tree.root = build_subtree(indices, root_region, problem, tree, build_err);
    if (tree.root < 0) {
        tree.error_msg = build_err.empty() ? "Failed to build topology tree" : build_err;
        return tree;
    }
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
