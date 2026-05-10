#include "treer.h"

#include "partreer.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <system_error>
#include <vector>

namespace treer {
namespace {

static bool g_debug_enabled = false;

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

static common::Point merge_point(const common::TopoNode& left,
                                 const common::TopoNode& right,
                                 common::Point external_target,
                                 const common::Problem& problem) {
    common::Point p;
    p.x = (left.loc.x + right.loc.x + 1) / 2;
    p.y = (left.loc.y + right.loc.y + 1) / 2;
    if (external_target.x > p.x) ++p.x;
    if (external_target.x < p.x) --p.x;
    if (external_target.y > p.y) ++p.y;
    if (external_target.y < p.y) --p.y;
    p.x = clamp_int(p.x, 0, problem.die_width);
    p.y = clamp_int(p.y, 0, problem.die_height);
    return p;
}

static int create_parent(common::TopoTree& tree,
                         int left_id,
                         int right_id,
                         common::Point external_target,
                         const common::Problem& problem) {
    const common::TopoNode left = tree.nodes[static_cast<std::size_t>(left_id)];
    const common::TopoNode right = tree.nodes[static_cast<std::size_t>(right_id)];

    common::TopoNode parent;
    parent.id = static_cast<int>(tree.nodes.size());
    parent.left = left_id;
    parent.right = right_id;
    parent.kind = common::NodeKind::Global;
    parent.loc = merge_point(left, right, external_target, problem);
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

    const int id = parent.id;
    tree.nodes.push_back(parent);
    tree.nodes[static_cast<std::size_t>(left_id)].parent = id;
    tree.nodes[static_cast<std::size_t>(right_id)].parent = id;
    if (g_debug_enabled) {
        std::cout << "[TREER] merge left=" << left_id
                  << " right=" << right_id
                  << " root=" << id
                  << " kind=" << kind_to_string(parent.kind)
                  << " loc=(" << parent.loc.x << "," << parent.loc.y << ")"
                  << " skew=" << parent.subtree_skew_to_node << "\n";
    }
    return id;
}

static std::string basename_no_ext(const std::string& input_path) {
    if (input_path.empty()) return "sample";
    const std::size_t pos = input_path.find_last_of("/\\");
    std::string base = (pos == std::string::npos) ? input_path : input_path.substr(pos + 1);
    const std::size_t dot = base.find_last_of('.');
    if (dot != std::string::npos) base = base.substr(0, dot);
    if (base.empty()) return "sample";
    return base;
}

static std::string output_path_for(const std::string& input_path) {
    return "tree/" + basename_no_ext(input_path) + "_vtree.txt";
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

static int append_local_tree(common::TopoTree& global,
                             const common::TopoTree& local) {
    const int offset = static_cast<int>(global.nodes.size());
    for (const common::TopoNode& old_node : local.nodes) {
        common::TopoNode node = old_node;
        node.id += offset;
        if (node.parent != -1) node.parent += offset;
        if (node.left != -1) node.left += offset;
        if (node.right != -1) node.right += offset;
        global.nodes.push_back(node);
    }
    return local.cluster_root + offset;
}

static int merge_child_roots(std::vector<int> roots,
                             common::Point external_target,
                             common::TopoTree& tree,
                             const common::Problem& problem) {
    if (roots.empty()) return -1;
    if (roots.size() == 1) return roots[0];

    std::stable_sort(roots.begin(), roots.end(), [&](int a, int b) {
        const common::Point& pa = tree.nodes[static_cast<std::size_t>(a)].loc;
        const common::Point& pb = tree.nodes[static_cast<std::size_t>(b)].loc;
        const int da = manhattan(pa, external_target);
        const int db = manhattan(pb, external_target);
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
            next.push_back(create_parent(tree, roots[i], roots[i + 1],
                                         external_target, problem));
        }
        roots = std::move(next);
    }
    return roots[0];
}

static void write_node_line(std::ostream& out, const common::TopoNode& node) {
    out << "NODE " << node.id << " " << node.parent << " "
        << node.left << " " << node.right << " "
        << (node.is_sink ? 1 : 0) << " " << node.sink_index << " "
        << node.sink_indices.size() << " " << node.loc.x << " " << node.loc.y << " "
        << node.bbox.lx << " " << node.bbox.ly << " "
        << node.bbox.ux << " " << node.bbox.uy << " "
        << node.region_lx << " " << node.region_ly << " "
        << node.region_ux << " " << node.region_uy << " "
        << node.subtree_skew_to_node << " "
        << kind_to_string(node.kind) << "\n";
}

static bool write_tree_file(const common::TopoTree& tree,
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

    for (const common::TopoNode& node : tree.nodes) {
        write_node_line(fout, node);
    }
    fout << "\n";

    for (const common::TopoNode& node : tree.nodes) {
        if (node.is_sink &&
            node.sink_index >= 0 &&
            static_cast<std::size_t>(node.sink_index) < problem.sinks.size()) {
            const common::Sink& sink =
                problem.sinks[static_cast<std::size_t>(node.sink_index)];
            fout << "LEAF " << node.id << " " << node.sink_index << " "
                 << sink.id << " " << sink.loc.x << " " << sink.loc.y << "\n";
        }
    }
    fout << "\n";

    for (const common::TopoNode& node : tree.nodes) {
        if (!node.is_sink) {
            if (node.left >= 0) fout << "EDGE " << node.id << " " << node.left << "\n";
            if (node.right >= 0) fout << "EDGE " << node.id << " " << node.right << "\n";
        }
    }
    return true;
}

static bool collect_subtree_sinks(int node_id,
                                  const common::TopoTree& tree,
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
    const common::TopoNode& node = tree.nodes[idx];
    if (node.is_sink) {
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
        if (!collect_subtree_sinks(node.left, tree, problem, node_state,
                                   sink_seen, reachable_nodes, left_sinks, err)) {
            return false;
        }
        if (node.right >= 0 &&
            !collect_subtree_sinks(node.right, tree, problem, node_state,
                                   sink_seen, reachable_nodes, right_sinks, err)) {
            return false;
        }
        sink_indices.reserve(left_sinks.size() + right_sinks.size());
        sink_indices.insert(sink_indices.end(), left_sinks.begin(), left_sinks.end());
        sink_indices.insert(sink_indices.end(), right_sinks.begin(), right_sinks.end());
        std::sort(sink_indices.begin(), sink_indices.end());
        if (node.sink_indices != sink_indices) {
            err = "Subtree sink_indices do not match descendant leaves";
            return false;
        }
    }

    node_state[idx] = 2;
    return true;
}

static bool validate_tree(const common::TopoTree& tree,
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
        const common::TopoNode& node = tree.nodes[i];
        if (node.id != static_cast<int>(i)) {
            err = "Tree node id does not match vector index";
            return false;
        }
        if (node.bbox.lx > node.bbox.ux || node.bbox.ly > node.bbox.uy) {
            err = "Tree node has invalid bbox";
            return false;
        }
        if (node.is_sink) {
            ++leaf_count;
            if (node.sink_index < 0 ||
                static_cast<std::size_t>(node.sink_index) >= problem.sinks.size()) {
                err = "Leaf node has invalid sink index";
                return false;
            }
            if (node.left != -1 || node.right != -1 || node.sink_indices.size() != 1) {
                err = "Leaf node has invalid child or sink_indices fields";
                return false;
            }
            if (node.kind != common::NodeKind::Sink) {
                err = "Sink leaf node has non-SINK kind";
                return false;
            }
        } else {
            const bool unary_access = node.kind == common::NodeKind::ClusterAccess &&
                                      node.left >= 0 &&
                                      node.right == -1;
            if (node.left < 0 ||
                (!unary_access && node.right < 0) ||
                (!unary_access && node.left == node.right) ||
                static_cast<std::size_t>(node.left) >= tree.nodes.size() ||
                (!unary_access && static_cast<std::size_t>(node.right) >= tree.nodes.size())) {
                err = "Internal node has invalid children";
                return false;
            }
            const common::TopoNode& left = tree.nodes[static_cast<std::size_t>(node.left)];
            if (left.parent != node.id) {
                err = "Child parent pointer mismatch";
                return false;
            }
            common::BBox child_union = left.bbox;
            if (!unary_access) {
                const common::TopoNode& right = tree.nodes[static_cast<std::size_t>(node.right)];
                if (right.parent != node.id) {
                    err = "Child parent pointer mismatch";
                    return false;
                }
                child_union = union_bbox(left.bbox, right.bbox);
            }
            if (node.bbox.lx != child_union.lx ||
                node.bbox.ly != child_union.ly ||
                node.bbox.ux != child_union.ux ||
                node.bbox.uy != child_union.uy) {
                err = "Internal node bbox does not match union of children bboxes";
                return false;
            }
            if (node.kind == common::NodeKind::Sink) {
                err = "Internal node has SINK kind";
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
    if (!collect_subtree_sinks(tree.root, tree, problem, node_state, sink_seen,
                               reachable_nodes, root_sinks, err)) {
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

struct BuildContext {
    const common::Problem& problem;
    const common::PartitionTree& partition_tree;
    common::TopoTree& global;
    std::vector<int> partition_to_topo_root;
};

static int build_partition_topology(BuildContext& ctx,
                                    int pid,
                                    common::Point external_target) {
    if (pid < 0 || static_cast<std::size_t>(pid) >= ctx.partition_tree.nodes.size()) {
        ctx.global.error_msg = "Partition node id out of range";
        return -1;
    }
    const common::PartitionNode& pnode =
        ctx.partition_tree.nodes[static_cast<std::size_t>(pid)];

    if (pnode.is_leaf || pnode.children.empty()) {
        common::TopoTree local =
            partreer::build(ctx.problem, pnode.sink_indices, external_target);
        if (!local.valid) {
            ctx.global.error_msg = "Partreer failed at partition node " +
                                   std::to_string(pid) + ": " + local.error_msg;
            return -1;
        }
        const int root = append_local_tree(ctx.global, local);
        ctx.partition_to_topo_root[static_cast<std::size_t>(pid)] = root;
        if (g_debug_enabled) {
            std::cout << "[TREER] leaf partition " << pid << " sinks=";
            for (int si : pnode.sink_indices) std::cout << si << " ";
            std::cout << "cluster_root=" << root << "\n";
        }
        return root;
    }

    std::vector<int> child_roots;
    for (int child_pid : pnode.children) {
        common::Point child_target = pnode.centroid;
        int child_root = build_partition_topology(ctx, child_pid, child_target);
        if (child_root < 0) return -1;
        child_roots.push_back(child_root);
    }

    int root = merge_child_roots(child_roots, external_target, ctx.global, ctx.problem);
    ctx.partition_to_topo_root[static_cast<std::size_t>(pid)] = root;
    if (g_debug_enabled) {
        std::cout << "[TREER] internal partition " << pid << " child_roots=";
        for (int r : child_roots) std::cout << r << " ";
        std::cout << "root=" << root << "\n";
    }
    return root;
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
    partreer::debug_enable(enable);
}

void debug_output(const TopoTree& tree, const common::Problem& problem) {
    if (!g_debug_enabled) return;
    std::cout << "TREER_DEBUG\n";
    std::cout << "VALID " << (tree.valid ? 1 : 0) << "\n";
    std::cout << "ERROR_MSG " << tree.error_msg << "\n";
    std::cout << "ROOT " << tree.root << "\n";
    std::cout << "NUM_NODES " << tree.nodes.size() << "\n";
    int sinks = 0;
    int cluster_internal = 0;
    int cluster_access = 0;
    int global = 0;
    for (const common::TopoNode& node : tree.nodes) {
        if (node.kind == common::NodeKind::Sink) ++sinks;
        if (node.kind == common::NodeKind::ClusterInternal) ++cluster_internal;
        if (node.kind == common::NodeKind::ClusterAccess) ++cluster_access;
        if (node.kind == common::NodeKind::Global) ++global;
        write_node_line(std::cout, node);
        if (node.is_sink &&
            node.sink_index >= 0 &&
            static_cast<std::size_t>(node.sink_index) < problem.sinks.size()) {
            const common::Sink& sink = problem.sinks[static_cast<std::size_t>(node.sink_index)];
            std::cout << "  LEAF_SINK " << sink.id << " "
                      << sink.loc.x << " " << sink.loc.y << "\n";
        }
    }
    std::cout << "KIND_STATS SINK " << sinks
              << " CLUSTER_INTERNAL " << cluster_internal
              << " CLUSTER_ACCESS " << cluster_access
              << " GLOBAL " << global << "\n";
    std::cout << "END_TREER_DEBUG\n";
}

void debug_output_file(const TopoTree& tree,
                       const common::Problem& problem,
                       const std::string& input_path) {
    if (!ensure_tree_dir()) return;
    const std::string path = output_path_for(input_path);
    write_tree_file(tree, problem, path);
    if (g_debug_enabled) {
        std::cout << "[TREER] wrote topology output to " << path << "\n";
    }
}

TopoTree build(const common::Problem& problem,
               const common::PartitionTree& partition_tree,
               const std::string& sample_name_or_input_path) {
    TopoTree tree;
    if (!problem.valid) {
        tree.error_msg = "Cannot build topology from invalid problem: " + problem.error_msg;
        debug_output_file(tree, problem, sample_name_or_input_path);
        return tree;
    }
    if (!partition_tree.valid) {
        tree.error_msg = "Cannot build topology from invalid partition tree: " +
                         partition_tree.error_msg;
        debug_output_file(tree, problem, sample_name_or_input_path);
        return tree;
    }
    if (problem.sinks.empty()) {
        tree.error_msg = "Cannot build topology with zero sinks";
        debug_output_file(tree, problem, sample_name_or_input_path);
        return tree;
    }

    tree.nodes.reserve(problem.sinks.size() * 2U + partition_tree.nodes.size() * 2U);
    BuildContext ctx{problem,
                     partition_tree,
                     tree,
                     std::vector<int>(partition_tree.nodes.size(), -1)};

    const int root = build_partition_topology(ctx, partition_tree.root, problem.source.loc);
    if (root < 0) {
        debug_output_file(tree, problem, sample_name_or_input_path);
        return tree;
    }

    tree.root = root;
    tree.cluster_root = root;
    tree.nodes[static_cast<std::size_t>(root)].parent = -1;
    tree.valid = true;

    std::string err;
    if (!validate_tree(tree, problem, err)) {
        tree.valid = false;
        tree.error_msg = err;
    }

    if (g_debug_enabled) {
        std::cout << "[TREER] partition->topo root mapping\n";
        for (std::size_t i = 0; i < ctx.partition_to_topo_root.size(); ++i) {
            std::cout << "  " << i << " -> " << ctx.partition_to_topo_root[i] << "\n";
        }
        debug_output(tree, problem);
    }
    debug_output_file(tree, problem, sample_name_or_input_path);
    return tree;
}

}  // namespace treer
