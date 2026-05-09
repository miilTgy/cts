#include "partitioner.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <system_error>
#include <vector>

namespace partitioner {
namespace {

static bool g_debug_enabled = false;

constexpr int kMinClusterSize = 4;
constexpr double kGapRatio = 4.0;
constexpr int kMinAbsGap = 10;
constexpr double kOutlierGapRatio = 3.5;
constexpr int kOutlierMinAbsGap = 20;

static int clamp_int(int value, int lo, int hi) {
    return std::max(lo, std::min(value, hi));
}

struct BBox {
    int lx = 0;
    int ly = 0;
    int ux = 0;
    int uy = 0;
};

static BBox compute_bbox(const std::vector<int>& sink_indices,
                         const common::Problem& problem) {
    BBox bb;
    if (sink_indices.empty()) {
        return bb;
    }
    bb.lx = problem.die_width;
    bb.ly = problem.die_height;
    bb.ux = 0;
    bb.uy = 0;
    for (int idx : sink_indices) {
        const common::Sink& s = problem.sinks[static_cast<std::size_t>(idx)];
        bb.lx = std::min(bb.lx, s.loc.x);
        bb.ly = std::min(bb.ly, s.loc.y);
        bb.ux = std::max(bb.ux, s.loc.x);
        bb.uy = std::max(bb.uy, s.loc.y);
    }
    bb.lx = clamp_int(bb.lx, 0, problem.die_width);
    bb.ly = clamp_int(bb.ly, 0, problem.die_height);
    bb.ux = clamp_int(bb.ux, 0, problem.die_width);
    bb.uy = clamp_int(bb.uy, 0, problem.die_height);
    return bb;
}

static common::Point compute_centroid(const std::vector<int>& sink_indices,
                                      const common::Problem& problem) {
    if (sink_indices.empty()) {
        return {0, 0};
    }
    long long sum_x = 0;
    long long sum_y = 0;
    for (int idx : sink_indices) {
        const common::Sink& s = problem.sinks[static_cast<std::size_t>(idx)];
        sum_x += s.loc.x;
        sum_y += s.loc.y;
    }
    common::Point c;
    int n = static_cast<int>(sink_indices.size());
    c.x = clamp_int(static_cast<int>(sum_x / n), 0, problem.die_width);
    c.y = clamp_int(static_cast<int>(sum_y / n), 0, problem.die_height);
    return c;
}

struct AxisCandidate {
    bool valid = false;
    std::vector<std::vector<int>> groups;
    double total_big_gap = 0.0;
    int num_big_gaps = 0;
    int num_outliers = 0;
    int num_tiny_groups = 0;  // groups of exactly size 2
};

static AxisCandidate evaluate_axis_split(bool use_x,
                                         const std::vector<int>& sink_indices,
                                         const common::Problem& problem) {
    AxisCandidate cand;
    int n = static_cast<int>(sink_indices.size());
    if (n <= 1) {
        cand.valid = false;
        return cand;
    }

    std::vector<int> sorted = sink_indices;
    if (use_x) {
        std::stable_sort(sorted.begin(), sorted.end(),
                         [&problem](int a, int b) {
                             return problem.sinks[static_cast<std::size_t>(a)].loc.x <
                                    problem.sinks[static_cast<std::size_t>(b)].loc.x;
                         });
    } else {
        std::stable_sort(sorted.begin(), sorted.end(),
                         [&problem](int a, int b) {
                             return problem.sinks[static_cast<std::size_t>(a)].loc.y <
                                    problem.sinks[static_cast<std::size_t>(b)].loc.y;
                         });
    }

    std::vector<int> gaps;
    gaps.reserve(static_cast<std::size_t>(n - 1));
    for (int i = 0; i < n - 1; ++i) {
        const common::Sink& a = problem.sinks[static_cast<std::size_t>(sorted[i])];
        const common::Sink& b = problem.sinks[static_cast<std::size_t>(sorted[i + 1])];
        int gap = use_x ? (b.loc.x - a.loc.x) : (b.loc.y - a.loc.y);
        gaps.push_back(gap);
    }

    std::vector<int> positive_gaps;
    for (int g : gaps) {
        if (g > 0) {
            positive_gaps.push_back(g);
        }
    }
    if (positive_gaps.empty()) {
        cand.valid = false;
        return cand;
    }

    std::sort(positive_gaps.begin(), positive_gaps.end());
    int pn = static_cast<int>(positive_gaps.size());
    double small_gap = static_cast<double>(positive_gaps[(pn - 1) / 2]);

    std::vector<bool> is_big(gaps.size(), false);
    double total_big = 0.0;
    int big_count = 0;
    for (std::size_t i = 0; i < gaps.size(); ++i) {
        if (gaps[i] >= kMinAbsGap &&
            static_cast<double>(gaps[i]) >= kGapRatio * small_gap) {
            is_big[i] = true;
            total_big += static_cast<double>(gaps[i]);
            ++big_count;
        }
    }

    if (big_count == 0) {
        cand.valid = false;
        return cand;
    }

    if (g_debug_enabled) {
        std::cout << "  PART_DEBUG: axis=" << (use_x ? 'x' : 'y')
                  << " sinks=" << n << " small_gap=" << small_gap
                  << " big_gaps=[";
        for (std::size_t i = 0; i < is_big.size(); ++i) {
            if (is_big[i]) {
                std::cout << gaps[i] << "(" << sorted[i] << "-" << sorted[i + 1] << ") ";
            }
        }
        std::cout << "]\n";
    }

    std::vector<std::vector<int>> init_groups;
    std::vector<int> cur;
    for (int i = 0; i < n; ++i) {
        cur.push_back(sorted[i]);
        if (i < n - 1 && is_big[static_cast<std::size_t>(i)]) {
            init_groups.push_back(std::move(cur));
            cur.clear();
        }
    }
    if (!cur.empty()) {
        init_groups.push_back(std::move(cur));
    }

    std::vector<std::vector<int>> groups;
    int outlier_count = 0;
    std::size_t gi = 0;
    while (gi < init_groups.size()) {
        if (init_groups[gi].size() >= 2) {
            groups.push_back(init_groups[gi]);
            ++gi;
            continue;
        }
        // size == 1: check outlier condition
        int sink = init_groups[gi][0];

        // find the big gap index that separates this single-sink group
        // init_groups[gi] spans from sorted[start_pos] to sorted[end_pos]
        // The big gap on the left is before start_pos, on the right is after end_pos
        // We need to find which big gaps bound this single sink
        // First, find position of this sink in sorted
        auto it = std::lower_bound(sorted.begin(), sorted.end(), sink,
                                   [&problem, use_x](int a, int b) {
                                       if (use_x) {
                                           return problem.sinks[static_cast<std::size_t>(a)].loc.x <
                                                  problem.sinks[static_cast<std::size_t>(b)].loc.x;
                                       }
                                       return problem.sinks[static_cast<std::size_t>(a)].loc.y <
                                              problem.sinks[static_cast<std::size_t>(b)].loc.y;
                                   });
        int pos = static_cast<int>(std::distance(sorted.begin(), it));

        bool left_big = (pos > 0) && is_big[static_cast<std::size_t>(pos - 1)];
        bool right_big = (pos < n - 1) && is_big[static_cast<std::size_t>(pos)];

        bool is_outlier = false;
        if (left_big && right_big) {
            int left_gap = gaps[static_cast<std::size_t>(pos - 1)];
            int right_gap = gaps[static_cast<std::size_t>(pos)];
            is_outlier = (left_gap >= kOutlierMinAbsGap &&
                          static_cast<double>(left_gap) >= kOutlierGapRatio * small_gap &&
                          right_gap >= kOutlierMinAbsGap &&
                          static_cast<double>(right_gap) >= kOutlierGapRatio * small_gap);
        } else if (left_big) {
            int left_gap = gaps[static_cast<std::size_t>(pos - 1)];
            is_outlier = (left_gap >= kOutlierMinAbsGap &&
                          static_cast<double>(left_gap) >= kOutlierGapRatio * small_gap);
        } else if (right_big) {
            int right_gap = gaps[static_cast<std::size_t>(pos)];
            is_outlier = (right_gap >= kOutlierMinAbsGap &&
                          static_cast<double>(right_gap) >= kOutlierGapRatio * small_gap);
        }

        if (is_outlier) {
            groups.push_back(init_groups[gi]);
            ++outlier_count;
            ++gi;
            if (g_debug_enabled) {
                std::cout << "  PART_DEBUG: outlier sink=" << sink << " on axis="
                          << (use_x ? 'x' : 'y') << "\n";
            }
        } else {
            // merge this single sink into a neighbor group
            if (g_debug_enabled) {
                std::cout << "  PART_DEBUG: merging single sink=" << sink
                          << " on axis=" << (use_x ? 'x' : 'y') << "\n";
            }

            // choose neighbor: prefer smaller, then left
            bool merge_left = false;
            if (gi > 0 && gi + 1 < init_groups.size()) {
                if (init_groups[gi - 1].size() <= init_groups[gi + 1].size()) {
                    merge_left = true;
                }
            } else if (gi > 0) {
                merge_left = true;
            }

            if (merge_left) {
                // merge into previous group (which may already be in groups)
                auto& target = groups.back();
                target.insert(target.end(), init_groups[gi].begin(),
                              init_groups[gi].end());
            } else {
                // merge into next group, then process that group
                init_groups[gi + 1].insert(init_groups[gi + 1].begin(),
                                           init_groups[gi].begin(),
                                           init_groups[gi].end());
            }
            ++gi;
        }
    }

    if (groups.size() <= 1) {
        cand.valid = false;
        return cand;
    }

    int tiny_count = 0;
    for (const auto& g : groups) {
        if (g.size() == 2) {
            ++tiny_count;
        }
    }

    cand.valid = true;
    cand.groups = std::move(groups);
    cand.total_big_gap = total_big;
    cand.num_big_gaps = big_count;
    cand.num_outliers = outlier_count;
    cand.num_tiny_groups = tiny_count;
    return cand;
}

static bool is_major_axis(bool use_x, const BBox& bbox) {
    int dx = bbox.ux - bbox.lx;
    int dy = bbox.uy - bbox.ly;
    if (dx > dy) return use_x;
    if (dy > dx) return !use_x;
    return use_x;  // tie: prefer x
}

static char select_axis(const AxisCandidate& cx,
                        const AxisCandidate& cy,
                        const BBox& bbox) {
    if (cx.valid && !cy.valid) return 'x';
    if (!cx.valid && cy.valid) return 'y';
    if (!cx.valid && !cy.valid) return '\0';

    double score_x = cx.total_big_gap + (is_major_axis(true, bbox) ? 0.5 : 0.0) -
                     static_cast<double>(cx.num_tiny_groups) * 0.3;
    double score_y = cy.total_big_gap + (is_major_axis(false, bbox) ? 0.5 : 0.0) -
                     static_cast<double>(cy.num_tiny_groups) * 0.3;

    if (g_debug_enabled) {
        std::cout << "  PART_DEBUG: select_axis score_x=" << score_x << " score_y=" << score_y
                  << " x_big=" << cx.total_big_gap << " y_big=" << cy.total_big_gap
                  << " x_tiny=" << cx.num_tiny_groups << " y_tiny=" << cy.num_tiny_groups
                  << " x_major=" << is_major_axis(true, bbox)
                  << " y_major=" << is_major_axis(false, bbox) << "\n";
    }

    if (score_x >= score_y) return 'x';
    return 'y';
}

static int build_recursive(const std::vector<int>& sink_indices,
                           const common::Problem& problem,
                           common::PartitionTree& tree) {
    int n = static_cast<int>(sink_indices.size());
    BBox bbox = compute_bbox(sink_indices, problem);
    common::Point centroid = compute_centroid(sink_indices, problem);

    int node_id = static_cast<int>(tree.nodes.size());
    common::PartitionNode node;
    node.id = node_id;
    node.sink_indices = sink_indices;
    node.bbox_lx = bbox.lx;
    node.bbox_ly = bbox.ly;
    node.bbox_ux = bbox.ux;
    node.bbox_uy = bbox.uy;
    node.centroid = centroid;

    if (n <= kMinClusterSize) {
        node.is_leaf = true;
        node.is_outlier = (n == 1);
        tree.nodes.push_back(std::move(node));
        if (g_debug_enabled) {
            std::cout << "  PART_DEBUG: leaf node_id=" << node_id
                      << " sinks=" << n << " outlier=" << node.is_outlier
                      << " bbox=(" << bbox.lx << "," << bbox.ly << ","
                      << bbox.ux << "," << bbox.uy << ")\n";
        }
        return node_id;
    }

    AxisCandidate cx = evaluate_axis_split(true, sink_indices, problem);
    AxisCandidate cy = evaluate_axis_split(false, sink_indices, problem);

    char chosen = select_axis(cx, cy, bbox);
    if (chosen == '\0') {
        node.is_leaf = true;
        tree.nodes.push_back(std::move(node));
        if (g_debug_enabled) {
            std::cout << "  PART_DEBUG: leaf (no axis) node_id=" << node_id
                      << " sinks=" << n
                      << " bbox=(" << bbox.lx << "," << bbox.ly << ","
                      << bbox.ux << "," << bbox.uy << ")\n";
        }
        return node_id;
    }

    const AxisCandidate& chosen_cand = (chosen == 'x') ? cx : cy;

    if (g_debug_enabled) {
        std::cout << "  PART_DEBUG: split node_id=" << node_id
                  << " axis=" << chosen << " groups=" << chosen_cand.groups.size()
                  << " sinks=" << n
                  << " bbox=(" << bbox.lx << "," << bbox.ly << ","
                  << bbox.ux << "," << bbox.uy << ")\n";
    }

    // push placeholder, will be updated after children are built
    tree.nodes.push_back(std::move(node));
    int parent_idx = static_cast<int>(tree.nodes.size()) - 1;

    std::vector<int> child_ids;
    for (const auto& group : chosen_cand.groups) {
        int child_id = build_recursive(group, problem, tree);
        child_ids.push_back(child_id);
    }

    tree.nodes[static_cast<std::size_t>(parent_idx)].children = std::move(child_ids);

    if (g_debug_enabled) {
        std::cout << "  PART_DEBUG: split complete node_id=" << parent_idx
                  << " children=";
        for (int c : tree.nodes[static_cast<std::size_t>(parent_idx)].children) {
            std::cout << c << " ";
        }
        std::cout << "\n";
    }

    return parent_idx;
}

static std::string get_basename(const std::string& input_path) {
    if (input_path.empty()) {
        return "unknown.txt";
    }
    const std::size_t pos = input_path.find_last_of("/\\");
    if (pos == std::string::npos) {
        return input_path;
    }
    if (pos + 1 >= input_path.size()) {
        return "unknown.txt";
    }
    return input_path.substr(pos + 1);
}

static bool ensure_part_dir() {
    std::error_code ec;
    std::filesystem::create_directories("part", ec);
    if (ec) {
        std::cerr << "Warning: cannot create part directory: " << ec.message() << "\n";
        return false;
    }
    return true;
}

static void debug_print_node(const common::PartitionNode& node, int depth,
                             const common::PartitionTree& tree) {
    std::string indent(static_cast<std::size_t>(depth) * 2, ' ');
    std::cout << indent << "PART_NODE " << node.id
              << " sinks=" << node.sink_indices.size()
              << " children=[";
    for (std::size_t i = 0; i < node.children.size(); ++i) {
        if (i > 0) std::cout << ",";
        std::cout << node.children[i];
    }
    std::cout << "]"
              << " bbox=(" << node.bbox_lx << "," << node.bbox_ly << ","
              << node.bbox_ux << "," << node.bbox_uy << ")"
              << " centroid=(" << node.centroid.x << "," << node.centroid.y << ")"
              << " leaf=" << node.is_leaf
              << " outlier=" << node.is_outlier
              << "\n";
    for (int child_id : node.children) {
        if (child_id >= 0 && static_cast<std::size_t>(child_id) < tree.nodes.size()) {
            debug_print_node(tree.nodes[static_cast<std::size_t>(child_id)],
                             depth + 1, tree);
        }
    }
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_output(const PartitionTree& tree) {
    if (!g_debug_enabled) return;
    std::cout << "BEGIN_PARTITIONER_DEBUG\n";
    if (!tree.valid) {
        std::cout << "  PART_DEBUG: invalid tree: " << tree.error_msg << "\n";
        std::cout << "END_PARTITIONER_DEBUG\n";
        return;
    }
    if (tree.root >= 0 && static_cast<std::size_t>(tree.root) < tree.nodes.size()) {
        debug_print_node(tree.nodes[static_cast<std::size_t>(tree.root)], 0, tree);
    }
    std::cout << "END_PARTITIONER_DEBUG\n";
}

void write_output_file(const PartitionTree& tree, const common::Problem& problem, const std::string& input_path) {
    if (!ensure_part_dir()) {
        return;
    }

    std::string out_path = "part/" + get_basename(input_path);
    std::ofstream fout(out_path);
    if (!fout) {
        std::cerr << "Warning: cannot write partition file: " << out_path << "\n";
        return;
    }

    fout << "PARTITION_TREE\n";
    fout << "NUM_NODES " << tree.nodes.size() << "\n";
    if (!tree.valid) {
        fout << "INVALID " << tree.error_msg << "\n";
        return;
    }

    int cluster_idx = 0;
    for (const auto& node : tree.nodes) {
        if (node.is_leaf) {
            fout << "CLUSTER " << cluster_idx
                 << " NODE " << node.id
                 << " OUTLIER " << (node.is_outlier ? 1 : 0)
                 << " SINKS";
            for (int si : node.sink_indices) {
                fout << " " << problem.sinks[static_cast<std::size_t>(si)].id;
            }
            fout << "\n";
            ++cluster_idx;
        } else {
            fout << "NODE " << node.id
                 << " CHILDREN";
            for (int ci : node.children) {
                fout << " " << ci;
            }
            fout << " SINKS";
            for (int si : node.sink_indices) {
                fout << " " << problem.sinks[static_cast<std::size_t>(si)].id;
            }
            fout << " BBOX (" << node.bbox_lx << "," << node.bbox_ly << ","
                 << node.bbox_ux << "," << node.bbox_uy << ")";
            fout << "\n";
        }
    }

    if (g_debug_enabled) {
        std::cout << "PART_DEBUG: wrote partition output to " << out_path << "\n";
    }
}

PartitionTree build(const common::Problem& problem) {
    PartitionTree tree;
    if (!problem.valid) {
        tree.error_msg = "Cannot build partition from invalid problem: " +
                         problem.error_msg;
        return tree;
    }
    if (problem.sinks.empty()) {
        tree.error_msg = "Cannot build partition with zero sinks";
        return tree;
    }

    std::vector<int> all_sinks;
    all_sinks.reserve(problem.sinks.size());
    for (std::size_t i = 0; i < problem.sinks.size(); ++i) {
        all_sinks.push_back(static_cast<int>(i));
    }

    tree.nodes.reserve(problem.sinks.size() * 2);
    tree.root = build_recursive(all_sinks, problem, tree);
    tree.valid = true;

    if (g_debug_enabled) {
        debug_output(tree);
    }

    return tree;
}

}  // namespace partitioner
