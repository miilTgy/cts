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

bool g_debug_enabled = false;

static double manhattan(double x1, double y1, double x2, double y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

static double pair_cost(const common::TreeNode& a,
                        const common::TreeNode& b,
                        const common::Point& source) {
    constexpr double alpha = 1.0;
    constexpr double beta = 5.0;
    constexpr double gamma = 1.0;
    constexpr double delta = 0.05;

    const double center_dist = manhattan(a.cx, a.cy, b.cx, b.cy);
    const double count_delta = std::abs(a.sink_count - b.sink_count);
    const double delay_delta = std::abs(a.est_delay - b.est_delay);
    const double source_a = manhattan(source.x, source.y, a.cx, a.cy);
    const double source_b = manhattan(source.x, source.y, b.cx, b.cy);
    const double source_bias = std::abs(source_a - source_b);
    return alpha * center_dist + beta * count_delta +
           gamma * delay_delta + delta * source_bias;
}

static common::TreeNode make_leaf(int id, int sink_index, const common::Sink& sink) {
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
    node.est_delay = 0.0;
    return node;
}

static common::TreeNode make_parent(int id,
                                    const common::TreeNode& left,
                                    const common::TreeNode& right) {
    common::TreeNode parent;
    parent.id = id;
    parent.is_leaf = false;
    parent.sink_index = -1;
    parent.left = left.id;
    parent.right = right.id;
    parent.sink_count = left.sink_count + right.sink_count;

    const double total = static_cast<double>(parent.sink_count);
    parent.cx = (left.cx * left.sink_count + right.cx * right.sink_count) / total;
    parent.cy = (left.cy * left.sink_count + right.cy * right.sink_count) / total;

    parent.bbox_lx = std::min(left.bbox_lx, right.bbox_lx);
    parent.bbox_ly = std::min(left.bbox_ly, right.bbox_ly);
    parent.bbox_ux = std::max(left.bbox_ux, right.bbox_ux);
    parent.bbox_uy = std::max(left.bbox_uy, right.bbox_uy);

    const double d = manhattan(left.cx, left.cy, right.cx, right.cy);
    const double dl = left.est_delay;
    const double dr = right.est_delay;
    parent.est_delay = std::max(dl, dr) + std::max(0.0, (d - std::abs(dl - dr)) / 2.0);
    return parent;
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

    std::vector<int> active_clusters;
    active_clusters.reserve(problem.sinks.size());
    tree.nodes.reserve(problem.sinks.size() * 2U - 1U);
    for (std::size_t i = 0; i < problem.sinks.size(); ++i) {
        const int id = static_cast<int>(tree.nodes.size());
        tree.nodes.push_back(make_leaf(id, static_cast<int>(i), problem.sinks[i]));
        active_clusters.push_back(id);
    }

    while (active_clusters.size() > 1U) {
        double best_cost = std::numeric_limits<double>::infinity();
        int best_i = -1;
        int best_j = -1;

        for (std::size_t i = 0; i < active_clusters.size(); ++i) {
            for (std::size_t j = i + 1U; j < active_clusters.size(); ++j) {
                const common::TreeNode& a = tree.nodes[static_cast<std::size_t>(active_clusters[i])];
                const common::TreeNode& b = tree.nodes[static_cast<std::size_t>(active_clusters[j])];
                const double cost = pair_cost(a, b, problem.source.loc);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_i = static_cast<int>(i);
                    best_j = static_cast<int>(j);
                }
            }
        }

        if (best_i < 0 || best_j < 0) {
            tree.error_msg = "Failed to select cluster pair";
            return tree;
        }

        const int left_id = active_clusters[static_cast<std::size_t>(best_i)];
        const int right_id = active_clusters[static_cast<std::size_t>(best_j)];
        const int parent_id = static_cast<int>(tree.nodes.size());

        common::TreeNode parent = make_parent(
            parent_id,
            tree.nodes[static_cast<std::size_t>(left_id)],
            tree.nodes[static_cast<std::size_t>(right_id)]);
        tree.nodes[static_cast<std::size_t>(left_id)].parent = parent_id;
        tree.nodes[static_cast<std::size_t>(right_id)].parent = parent_id;
        tree.nodes.push_back(parent);

        const int erase_hi = std::max(best_i, best_j);
        const int erase_lo = std::min(best_i, best_j);
        active_clusters.erase(active_clusters.begin() + erase_hi);
        active_clusters.erase(active_clusters.begin() + erase_lo);
        active_clusters.push_back(parent_id);
    }

    tree.root = active_clusters[0];
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
