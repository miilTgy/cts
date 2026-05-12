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
                std::cout << "[TREER] canonicalize absorb parent="
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
                std::cout << "[TREER] canonicalize replace parent="
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
                std::cout << "[TREER] canonicalize keep unary parent="
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

static void compact_to_source_children(common::TopoTree& tree,
                                       const std::vector<int>& source_children) {
    std::vector<int> order;
    std::vector<int> visited(tree.nodes.size(), 0);
    for (int child : source_children) {
        collect_reachable_nodes(tree, child, order, visited);
    }

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

    std::vector<int> remapped_source_children;
    remapped_source_children.reserve(source_children.size());
    for (int child : source_children) {
        if (child >= 0 &&
            static_cast<std::size_t>(child) < remap.size() &&
            remap[static_cast<std::size_t>(child)] >= 0) {
            remapped_source_children.push_back(remap[static_cast<std::size_t>(child)]);
        }
    }

    tree.nodes = std::move(compacted);
    tree.root = -1;
    tree.cluster_root = -1;
    tree.source_children = std::move(remapped_source_children);
    for (int child : tree.source_children) {
        tree.nodes[static_cast<std::size_t>(child)].parent = -1;
    }
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

static int canonicalize_local_tree(common::TopoTree& tree) {
    if (tree.cluster_root < 0) return -1;
    swallow_access_internal_wrappers(tree);
    int root = tree.cluster_root;
    canonicalize_unary_nodes(tree, root);
    tree.cluster_root = root;
    compact_to_reachable(tree, tree.cluster_root);
    if (tree.root >= 0) {
        tree.nodes[static_cast<std::size_t>(tree.root)].parent = -1;
        recompute_subtree(tree, tree.root);
        tree.nodes[static_cast<std::size_t>(tree.root)].parent = -1;
    }
    return tree.cluster_root;
}

static int append_local_tree(common::TopoTree& global,
                             common::TopoTree local) {
    canonicalize_local_tree(local);
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

static bool is_leaf_representative(const common::TopoTree& tree, int node_id) {
    return node_id >= 0 &&
           static_cast<std::size_t>(node_id) < tree.nodes.size() &&
           tree.nodes[static_cast<std::size_t>(node_id)].kind ==
               common::NodeKind::ClusterTop;
}

static void absorb_source_root_if_global(common::TopoTree& tree, int root) {
    tree.source_children.clear();
    if (root < 0 ||
        static_cast<std::size_t>(root) >= tree.nodes.size() ||
        tree.nodes[static_cast<std::size_t>(root)].kind != common::NodeKind::Global) {
        return;
    }

    const common::TopoNode root_node = tree.nodes[static_cast<std::size_t>(root)];
    std::vector<int> children;
    if (root_node.left >= 0) children.push_back(root_node.left);
    if (root_node.right >= 0) children.push_back(root_node.right);
    if (children.empty()) return;

    for (int child : children) {
        tree.nodes[static_cast<std::size_t>(child)].parent = -1;
    }
    tree.nodes[static_cast<std::size_t>(root)].left = -1;
    tree.nodes[static_cast<std::size_t>(root)].right = -1;

    if (g_debug_enabled) {
        std::cout << "[TREER] source absorbed root Global " << root
                  << " children=";
        for (int child : children) std::cout << child << " ";
        std::cout << "\n";
    }
    compact_to_source_children(tree, children);
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
    fout << "SOURCE " << problem.source.loc.x << " "
         << problem.source.loc.y << "\n";
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
    if (tree.root >= 0) {
        fout << "EDGE SRC " << tree.root << "\n";
    } else {
        for (int child : tree.source_children) {
            fout << "EDGE SRC " << child << "\n";
        }
    }
    return true;
}

struct ClusterRootInfo {
    int node_id = -1;
    common::BBox bbox;
    common::Point loc;
};

static common::Point bbox_center(const common::BBox& bbox) {
    common::Point p;
    p.x = (bbox.lx + bbox.ux) / 2;
    p.y = (bbox.ly + bbox.uy) / 2;
    return p;
}

static common::BBox bbox_union_many(const std::vector<ClusterRootInfo>& roots) {
    if (roots.empty()) return {0, 0, 0, 0};
    common::BBox bbox = roots.front().bbox;
    for (std::size_t i = 1; i < roots.size(); ++i) {
        bbox = union_bbox(bbox, roots[i].bbox);
    }
    return bbox;
}

enum class SweepAxis {
    X,
    Y
};

enum class SourceRelation {
    Left,
    Right,
    Below,
    Above,
    Inside
};

static SourceRelation classify_source_relation(const common::Point& source,
                                               const common::BBox& union_bbox) {
    if (source.x < union_bbox.lx) return SourceRelation::Left;
    if (source.x > union_bbox.ux) return SourceRelation::Right;
    if (source.y < union_bbox.ly) return SourceRelation::Below;
    if (source.y > union_bbox.uy) return SourceRelation::Above;
    return SourceRelation::Inside;
}

static SweepAxis pick_primary_axis(const common::BBox& union_bbox) {
    const int width = union_bbox.ux - union_bbox.lx;
    const int height = union_bbox.uy - union_bbox.ly;
    return width >= height ? SweepAxis::X : SweepAxis::Y;
}

static bool compare_root_by_relation(const ClusterRootInfo& a,
                                     const ClusterRootInfo& b,
                                     SourceRelation relation,
                                     SweepAxis axis,
                                     const common::Point& source) {
    const int ax2 = a.loc.x * 2;
    const int ay2 = a.loc.y * 2;
    const int bx2 = b.loc.x * 2;
    const int by2 = b.loc.y * 2;
    auto tie_id = [&](const ClusterRootInfo& lhs, const ClusterRootInfo& rhs) {
        return lhs.node_id < rhs.node_id;
    };

    if (relation == SourceRelation::Left) {
        if (ax2 != bx2) return ax2 < bx2;
        if (ay2 != by2) return ay2 > by2;
        return tie_id(a, b);
    }
    if (relation == SourceRelation::Right) {
        if (ax2 != bx2) return ax2 > bx2;
        if (ay2 != by2) return ay2 > by2;
        return tie_id(a, b);
    }
    if (relation == SourceRelation::Below) {
        if (ay2 != by2) return ay2 < by2;
        if (ax2 != bx2) return ax2 < bx2;
        return tie_id(a, b);
    }
    if (relation == SourceRelation::Above) {
        if (ay2 != by2) return ay2 > by2;
        if (ax2 != bx2) return ax2 < bx2;
        return tie_id(a, b);
    }

    if (axis == SweepAxis::X) {
        const bool a_left = a.loc.x < source.x ||
                            (a.loc.x == source.x && a.loc.y <= source.y);
        const bool b_left = b.loc.x < source.x ||
                            (b.loc.x == source.x && b.loc.y <= source.y);
        if (a_left != b_left) return a_left;
        if (a_left) {
            if (ax2 != bx2) return ax2 < bx2;
            if (ay2 != by2) return ay2 > by2;
            return tie_id(a, b);
        }
        if (ax2 != bx2) return ax2 > bx2;
        if (ay2 != by2) return ay2 > by2;
        return tie_id(a, b);
    }

    const bool a_below = a.loc.y < source.y ||
                         (a.loc.y == source.y && a.loc.x <= source.x);
    const bool b_below = b.loc.y < source.y ||
                         (b.loc.y == source.y && b.loc.x <= source.x);
    if (a_below != b_below) return a_below;
    if (a_below) {
        if (ay2 != by2) return ay2 < by2;
        if (ax2 != bx2) return ax2 < bx2;
        return tie_id(a, b);
    }
    if (ay2 != by2) return ay2 > by2;
    if (ax2 != bx2) return ax2 < bx2;
    return tie_id(a, b);
}

static int create_global_parent(common::TopoTree& tree,
                                int left_id,
                                int right_id,
                                common::Point external_target,
                                const common::Problem& problem) {
    return create_parent(tree, left_id, right_id, external_target, problem);
}

static int build_chain_from_ordered_roots(common::TopoTree& tree,
                                         const std::vector<int>& ordered_roots,
                                         const common::Problem& problem,
                                         common::Point external_target) {
    if (ordered_roots.empty()) return -1;
    if (ordered_roots.size() == 1) return ordered_roots.front();

    int root = ordered_roots.back();
    for (std::size_t i = ordered_roots.size() - 1; i-- > 0;) {
        root = create_global_parent(tree, ordered_roots[i], root,
                                    external_target, problem);
        if (root < 0) return -1;
    }
    return root;
}

static int build_source_aware_global_tree(common::TopoTree& tree,
                                          const common::Problem& problem,
                                          const std::vector<int>& cluster_roots) {
    if (cluster_roots.empty()) return -1;
    if (cluster_roots.size() == 1) return cluster_roots.front();

    std::vector<ClusterRootInfo> roots;
    roots.reserve(cluster_roots.size());
    for (int root_id : cluster_roots) {
        if (root_id < 0 ||
            static_cast<std::size_t>(root_id) >= tree.nodes.size()) {
            continue;
        }
        const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(root_id)];
        roots.push_back({root_id, node.bbox, bbox_center(node.bbox)});
    }
    if (roots.empty()) return -1;
    if (roots.size() == 1) return roots.front().node_id;

    const common::BBox union_bbox = bbox_union_many(roots);
    const SourceRelation relation =
        classify_source_relation(problem.source.loc, union_bbox);
    const SweepAxis axis = pick_primary_axis(union_bbox);

    auto sort_group = [&](std::vector<ClusterRootInfo>& group) {
        std::stable_sort(group.begin(), group.end(),
                         [&](const ClusterRootInfo& a, const ClusterRootInfo& b) {
            return compare_root_by_relation(a, b, relation, axis, problem.source.loc);
        });
    };

    if (relation == SourceRelation::Inside) {
        std::vector<ClusterRootInfo> neg;
        std::vector<ClusterRootInfo> pos;
        neg.reserve(roots.size());
        pos.reserve(roots.size());

        for (const ClusterRootInfo& info : roots) {
            if (axis == SweepAxis::X) {
                const bool negative = info.loc.x < problem.source.loc.x ||
                                      (info.loc.x == problem.source.loc.x &&
                                       info.loc.y <= problem.source.loc.y);
                if (negative) {
                    neg.push_back(info);
                } else {
                    pos.push_back(info);
                }
            } else {
                const bool negative = info.loc.y < problem.source.loc.y ||
                                      (info.loc.y == problem.source.loc.y &&
                                       info.loc.x <= problem.source.loc.x);
                if (negative) {
                    neg.push_back(info);
                } else {
                    pos.push_back(info);
                }
            }
        }

        sort_group(neg);
        sort_group(pos);

        std::vector<int> neg_ids;
        std::vector<int> pos_ids;
        neg_ids.reserve(neg.size());
        pos_ids.reserve(pos.size());
        for (const ClusterRootInfo& info : neg) neg_ids.push_back(info.node_id);
        for (const ClusterRootInfo& info : pos) pos_ids.push_back(info.node_id);

        const int neg_root = build_chain_from_ordered_roots(tree, neg_ids,
                                                            problem, problem.source.loc);
        const int pos_root = build_chain_from_ordered_roots(tree, pos_ids,
                                                            problem, problem.source.loc);
        if (neg_root >= 0 && pos_root >= 0) {
            return create_global_parent(tree, neg_root, pos_root,
                                        problem.source.loc, problem);
        }
        return neg_root >= 0 ? neg_root : pos_root;
    }

    sort_group(roots);
    std::vector<int> ordered_roots;
    ordered_roots.reserve(roots.size());
    for (const ClusterRootInfo& info : roots) {
        ordered_roots.push_back(info.node_id);
    }
    return build_chain_from_ordered_roots(tree, ordered_roots,
                                          problem, problem.source.loc);
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
    const bool source_root_mode = tree.root == -1 && !tree.source_children.empty();
    if (source_root_mode) {
        for (int child : tree.source_children) {
            if (child < 0 || static_cast<std::size_t>(child) >= tree.nodes.size()) {
                err = "SOURCE edge references invalid child";
                return false;
            }
            if (tree.nodes[static_cast<std::size_t>(child)].parent != -1) {
                err = "SOURCE child must not have a topology parent";
                return false;
            }
        }
    } else {
        if (tree.root < 0 || static_cast<std::size_t>(tree.root) >= tree.nodes.size()) {
            err = "Invalid tree root";
            return false;
        }
        if (!tree.source_children.empty()) {
            err = "source_children must be empty when ROOT is a topology node";
            return false;
        }
        if (tree.nodes[static_cast<std::size_t>(tree.root)].parent != -1) {
            err = "Tree root must not have a parent";
            return false;
        }
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
                err = "Sink leaf node has invalid kind";
                return false;
            }
        } else {
            const bool unary_node = node.left >= 0 && node.right == -1;
            const bool binary_node = node.left >= 0 &&
                                     node.right >= 0 &&
                                     node.left != node.right;
            const bool allow_unary =
                unary_node &&
                (node.kind == common::NodeKind::ClusterAccess ||
                 node.kind == common::NodeKind::ClusterTop);
            if (node.kind == common::NodeKind::ClusterAccess &&
                !unary_node &&
                !binary_node) {
                err = "ClusterAccess must have one child or two distinct children";
                return false;
            }
            if (node.kind == common::NodeKind::ClusterTop &&
                !unary_node &&
                !binary_node) {
                err = "ClusterTop must have one child or two distinct children";
                return false;
            }
            if (node.left < 0 ||
                (!allow_unary && node.right < 0) ||
                (!allow_unary && node.left == node.right) ||
                static_cast<std::size_t>(node.left) >= tree.nodes.size() ||
                (!allow_unary && static_cast<std::size_t>(node.right) >= tree.nodes.size())) {
                err = "Internal node has invalid children";
                return false;
            }
            const common::TopoNode& left = tree.nodes[static_cast<std::size_t>(node.left)];
            if (left.parent != node.id) {
                err = "Child parent pointer mismatch";
                return false;
            }
            if (node.kind == common::NodeKind::ClusterAccess &&
                left.kind == common::NodeKind::ClusterAccess) {
                err = "ClusterAccess nodes must not be parent-child";
                return false;
            }
            if (allow_unary &&
                (node.kind == common::NodeKind::ClusterAccess ||
                 node.kind == common::NodeKind::ClusterTop) &&
                left.kind == common::NodeKind::ClusterInternal) {
                err = "Canonicalized access/top node must not wrap a ClusterInternal child";
                return false;
            }
            if (node.kind == common::NodeKind::ClusterTop &&
                left.kind == common::NodeKind::Global) {
                err = "ClusterTop must not have a Global child";
                return false;
            }
            if (node.kind == common::NodeKind::ClusterBridge &&
                node.parent >= 0) {
                const common::TopoNode& parent =
                    tree.nodes[static_cast<std::size_t>(node.parent)];
                if (parent.kind != common::NodeKind::ClusterBridge &&
                    parent.kind != common::NodeKind::ClusterTop) {
                    err = "ClusterBridge appears outside a leaf access tree";
                    return false;
                }
            }
            if (node.kind == common::NodeKind::ClusterTop && node.parent >= 0) {
                const common::TopoNode& parent =
                    tree.nodes[static_cast<std::size_t>(node.parent)];
                if (parent.kind != common::NodeKind::Global) {
                    err = "ClusterTop parent must be Global or root";
                    return false;
                }
            }
            common::BBox child_union = left.bbox;
            if (!allow_unary) {
                const common::TopoNode& right = tree.nodes[static_cast<std::size_t>(node.right)];
                if (right.parent != node.id) {
                    err = "Child parent pointer mismatch";
                    return false;
                }
                if (node.kind == common::NodeKind::ClusterAccess &&
                    right.kind == common::NodeKind::ClusterAccess) {
                    err = "ClusterAccess nodes must not be parent-child";
                    return false;
                }
                if (node.kind == common::NodeKind::ClusterTop &&
                    right.kind == common::NodeKind::Global) {
                    err = "ClusterTop must not have a Global child";
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

        const bool is_source_child =
            source_root_mode &&
            std::find(tree.source_children.begin(), tree.source_children.end(),
                      node.id) != tree.source_children.end();
        if (node.id != tree.root && !is_source_child) {
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
    if (source_root_mode) {
        for (int child : tree.source_children) {
            std::vector<int> child_sinks;
            if (!collect_subtree_sinks(child, tree, problem, node_state, sink_seen,
                                       reachable_nodes, child_sinks, err)) {
                return false;
            }
            root_sinks.insert(root_sinks.end(), child_sinks.begin(), child_sinks.end());
        }
        std::sort(root_sinks.begin(), root_sinks.end());
    } else {
        if (!collect_subtree_sinks(tree.root, tree, problem, node_state, sink_seen,
                                   reachable_nodes, root_sinks, err)) {
            return false;
        }
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
    std::vector<int> cluster_roots;
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
        if (!is_leaf_representative(local, local.cluster_root) ||
            local.root != local.cluster_root) {
            ctx.global.error_msg = "Partreer did not return a ClusterTop root at partition node " +
                                   std::to_string(pid);
            return -1;
        }
        const int root = append_local_tree(ctx.global, local);
        ctx.partition_to_topo_root[static_cast<std::size_t>(pid)] = root;
        ctx.cluster_roots.push_back(root);
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

    if (g_debug_enabled) {
        std::cout << "[TREER] internal partition " << pid << " child_roots=";
        for (int r : child_roots) std::cout << r << " ";
        std::cout << "mapped_root=" << (child_roots.empty() ? -1 : child_roots.front())
                  << "\n";
    }
    const int mapped_root = child_roots.empty() ? -1 : child_roots.front();
    ctx.partition_to_topo_root[static_cast<std::size_t>(pid)] = mapped_root;
    return mapped_root;
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
    std::cout << "SOURCE_CHILDREN ";
    for (int child : tree.source_children) std::cout << child << " ";
    std::cout << "\n";
    std::cout << "NUM_NODES " << tree.nodes.size() << "\n";
    int sinks = 0;
    int cluster_internal = 0;
    int cluster_access = 0;
    int cluster_bridge = 0;
    int cluster_top = 0;
    int global = 0;
    int edge_count = tree.root >= 0 ? 1 : static_cast<int>(tree.source_children.size());
    for (const common::TopoNode& node : tree.nodes) {
        if (node.kind == common::NodeKind::Sink) ++sinks;
        if (node.kind == common::NodeKind::ClusterInternal) ++cluster_internal;
        if (node.kind == common::NodeKind::ClusterAccess) ++cluster_access;
        if (node.kind == common::NodeKind::ClusterBridge) ++cluster_bridge;
        if (node.kind == common::NodeKind::ClusterTop) ++cluster_top;
        if (node.kind == common::NodeKind::Global) ++global;
        if (!node.is_sink) {
            if (node.left >= 0) ++edge_count;
            if (node.right >= 0) ++edge_count;
        }
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
              << " CLUSTER_BRIDGE " << cluster_bridge
              << " CLUSTER_TOP " << cluster_top
              << " GLOBAL " << global << "\n";
    std::cout << "EDGE_COUNT " << edge_count << "\n";
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
    if (problem.sinks.empty()) {
        tree.root = -1;
        tree.cluster_root = -1;
        tree.valid = true;
        debug_output_file(tree, problem, sample_name_or_input_path);
        return tree;
    }
    if (!partition_tree.valid) {
        tree.error_msg = "Cannot build topology from invalid partition tree: " +
                         partition_tree.error_msg;
        debug_output_file(tree, problem, sample_name_or_input_path);
        return tree;
    }

    tree.nodes.reserve(problem.sinks.size() * 2U + partition_tree.nodes.size() * 2U);
    BuildContext ctx{problem,
                     partition_tree,
                     tree,
                     std::vector<int>(partition_tree.nodes.size(), -1),
                     {}};

    const int traversal_root =
        build_partition_topology(ctx, partition_tree.root, problem.source.loc);
    if (traversal_root < 0) {
        debug_output_file(tree, problem, sample_name_or_input_path);
        return tree;
    }

    const int root =
        build_source_aware_global_tree(tree, problem, ctx.cluster_roots);
    if (root < 0) {
        tree.error_msg = "Failed to build source-aware global topology";
        debug_output_file(tree, problem, sample_name_or_input_path);
        return tree;
    }

    // absorb_source_root_if_global(tree, root);

    if (tree.source_children.empty()) {
        tree.root = root;
        tree.cluster_root = root;
        tree.nodes[static_cast<std::size_t>(root)].parent = -1;
    }
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
        std::cout << "[TREER] cluster_roots ";
        for (int cluster_root : ctx.cluster_roots) std::cout << cluster_root << " ";
        std::cout << "\n";
        debug_output(tree, problem);
    }
    debug_output_file(tree, problem, sample_name_or_input_path);
    return tree;
}

}  // namespace treer
