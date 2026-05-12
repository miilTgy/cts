#include "bufferer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace bufferer {
namespace {

static bool g_debug_enabled = false;
static bool g_debug_file_enabled = false;

static constexpr long long SCORE_BASE = 5000000LL;
static constexpr long long SKEW_WEIGHT = 5000LL;
static constexpr long long WIRE_WEIGHT = 50LL;
static constexpr long long BUFFER_WEIGHT = 200LL;
static constexpr int MAX_PASSES = 20;

struct EdgeKey {
    int parent = -1;
    int child = -1;

    bool operator<(const EdgeKey& other) const {
        if (parent != other.parent) return parent < other.parent;
        return child < other.child;
    }
};

struct DelayProfile {
    std::vector<double> sink_delays;
    double min_delay = 0.0;
    double max_delay = 0.0;
    double skew = 0.0;
};

static std::vector<int> children_of(const common::TopoTree& tree, int node_id) {
    std::vector<int> children;
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size())
        return children;
    const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
    if (node.left >= 0) children.push_back(node.left);
    if (node.right >= 0 && node.right != node.left) children.push_back(node.right);
    if (!children.empty()) return children;
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        if (tree.nodes[i].parent == node_id)
            children.push_back(static_cast<int>(i));
    }
    return children;
}

static void dfs_postorder(const common::TopoTree& tree, int node_id,
                          std::vector<int>& state, std::vector<int>& order) {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= tree.nodes.size()) return;
    int& mark = state[static_cast<std::size_t>(node_id)];
    if (mark != 0) return;
    mark = 1;
    for (int child : children_of(tree, node_id))
        dfs_postorder(tree, child, state, order);
    mark = 2;
    order.push_back(node_id);
}

static std::vector<int> build_bottom_up_order(const common::TopoTree& tree) {
    std::vector<int> state(tree.nodes.size(), 0);
    std::vector<int> order;
    if (!tree.source_children.empty()) {
        for (int child : tree.source_children)
            dfs_postorder(tree, child, state, order);
    } else if (tree.root >= 0) {
        dfs_postorder(tree, tree.root, state, order);
    }
    for (std::size_t i = 0; i < tree.nodes.size(); ++i)
        dfs_postorder(tree, static_cast<int>(i), state, order);
    return order;
}

static int polyline_length(const common::RouterEdgeDebug& edge) {
    int total = 0;
    for (std::size_t i = 1; i < edge.polyline.size(); ++i) {
        total += static_cast<int>(std::llround(
            std::abs(edge.polyline[i].x - edge.polyline[i - 1U].x) +
            std::abs(edge.polyline[i].y - edge.polyline[i - 1U].y)));
    }
    return total;
}

static bool build_edge_delay_map(const common::RouterResult& route_result,
                                 std::map<EdgeKey, int>& edge_delay) {
    edge_delay.clear();
    for (const common::RouterEdgeDebug& edge : route_result.edge_debugs) {
        if (!edge.routed || edge.failure_reason != "OK") continue;
        edge_delay[EdgeKey{edge.parent, edge.child}] = polyline_length(edge);
    }
    return true;
}

static int get_edge_delay(const std::map<EdgeKey, int>& edge_delay,
                          int parent, int child) {
    auto it = edge_delay.find(EdgeKey{parent, child});
    return (it != edge_delay.end()) ? it->second : 0;
}

static void compute_downstream_sink_count(const common::TopoTree& tree,
                                          const std::vector<int>& bottom_up_order,
                                          std::vector<int>& sink_count) {
    sink_count.assign(tree.nodes.size(), 0);
    for (int node_id : bottom_up_order) {
        const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
        int cnt = node.is_sink ? 1 : 0;
        for (int child : children_of(tree, node_id))
            cnt += sink_count[static_cast<std::size_t>(child)];
        sink_count[static_cast<std::size_t>(node_id)] = cnt;
    }
}

static void compute_delay_profiles(const common::TopoTree& tree,
                                   const std::vector<int>& bottom_up_order,
                                   const std::map<EdgeKey, int>& edge_delay,
                                   const std::vector<int>& buffer_at,
                                   const common::Problem& problem,
                                   std::vector<DelayProfile>& profiles) {
    profiles.assign(tree.nodes.size(), DelayProfile{});
    for (int node_id : bottom_up_order) {
        const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
        DelayProfile& profile = profiles[static_cast<std::size_t>(node_id)];
        profile.sink_delays.clear();

        if (node.is_sink) {
            profile.sink_delays.push_back(0.0);
        } else {
            for (int child : children_of(tree, node_id)) {
                const DelayProfile& child_profile =
                    profiles[static_cast<std::size_t>(child)];
                const int buf_idx = buffer_at[static_cast<std::size_t>(child)];
                double buf_delay = 0.0;
                if (buf_idx >= 0 && static_cast<std::size_t>(buf_idx) < problem.buffer_types.size())
                    buf_delay = problem.buffer_types[static_cast<std::size_t>(buf_idx)].delay;
                const double edge_d = static_cast<double>(
                    get_edge_delay(edge_delay, node_id, child));
                for (double d : child_profile.sink_delays)
                    profile.sink_delays.push_back(d + buf_delay + edge_d);
            }
        }

        if (profile.sink_delays.empty())
            profile.sink_delays.push_back(0.0);

        const auto [min_it, max_it] =
            std::minmax_element(profile.sink_delays.begin(), profile.sink_delays.end());
        profile.min_delay = *min_it;
        profile.max_delay = *max_it;
        profile.skew = profile.max_delay - profile.min_delay;
    }
}

static double compute_objective_skew(const common::TopoTree& tree,
                                     const std::vector<DelayProfile>& profiles,
                                     const std::map<EdgeKey, int>& edge_delay,
                                      const common::Problem& problem,
                                      const std::vector<int>& buffer_at) {
    std::vector<double> source_delays;
    auto collect_delays = [&](int child_id) {
        if (child_id < 0 || static_cast<std::size_t>(child_id) >= profiles.size())
            return;
        const DelayProfile& child_profile =
            profiles[static_cast<std::size_t>(child_id)];
        const int buf_idx = buffer_at[static_cast<std::size_t>(child_id)];
        double buf_delay = 0.0;
        if (buf_idx >= 0 && static_cast<std::size_t>(buf_idx) < problem.buffer_types.size())
            buf_delay = problem.buffer_types[static_cast<std::size_t>(buf_idx)].delay;
        const double edge_d = static_cast<double>(
            get_edge_delay(edge_delay, -1, child_id));
        for (double d : child_profile.sink_delays)
            source_delays.push_back(d + buf_delay + edge_d);
    };

    if (!tree.source_children.empty()) {
        for (int child : tree.source_children) collect_delays(child);
    } else if (tree.root >= 0) {
        collect_delays(tree.root);
    }

    if (source_delays.size() <= 1U) return 0.0;
    const auto [min_it, max_it] =
        std::minmax_element(source_delays.begin(), source_delays.end());
    return *max_it - *min_it;
}

static long long compute_score(double objective_skew, long long wirelength,
                               long long buffer_cost) {
    return SCORE_BASE - SKEW_WEIGHT * static_cast<long long>(std::llround(objective_skew))
           - WIRE_WEIGHT * wirelength - BUFFER_WEIGHT * buffer_cost;
}

static long long compute_total_wirelength(const common::RouterResult& route_result) {
    long long total = 0;
    for (const common::RouterEdgeDebug& edge : route_result.edge_debugs) {
        if (!edge.routed || edge.failure_reason != "OK") continue;
        total += static_cast<long long>(std::llround(edge.wirelength));
    }
    return total;
}

static bool validate_inputs(const common::Problem& problem,
                            const common::TopoTree& tree,
                            const common::LocerResult& loc_result,
                            const common::RouterResult& route_result,
                            std::string& err) {
    if (!problem.valid) { err = "Invalid problem: " + problem.error_msg; return false; }
    if (!tree.valid) { err = "Invalid tree: " + tree.error_msg; return false; }
    if (!loc_result.valid) { err = "Invalid locer result: " + loc_result.error_msg; return false; }
    if (!route_result.valid) { err = "Invalid router result: " + route_result.error_msg; return false; }
    if (tree.nodes.empty()) { err = "Empty topology tree"; return false; }
    if (loc_result.node_results.size() != tree.nodes.size()) {
        err = "Locer node count mismatch"; return false;
    }
    return true;
}

static std::string basename_without_ext(const std::string& input_path) {
    std::filesystem::path path(input_path);
    return path.stem().string();
}

static std::string make_debug_buffer_path(const std::string& input_path) {
    const std::string base = basename_without_ext(input_path);
    if (base.empty()) return "buffer/buffer_debug.txt";
    return "buffer/" + base + "_buffer.txt";
}

static bool ensure_buffer_dir(std::string& error_msg) {
    std::error_code ec;
    std::filesystem::create_directories("buffer", ec);
    if (ec) {
        error_msg = "Cannot create buffer directory: " + ec.message();
        return false;
    }
    return true;
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_file_enable(bool enable) {
    g_debug_file_enabled = enable;
}

void debug_output(const common::BuffererResult& result,
                  const common::Problem&,
                  const common::TopoTree& tree,
                  const common::LocerResult&,
                  const common::RouterResult&) {
    if (!g_debug_enabled) return;
    std::cout << "[BUFFERER] valid=" << (result.valid ? 1 : 0)
              << " error_msg=" << result.error_msg
              << " num_buffers=" << result.buffers.size()
              << " total_buffer_cost=" << result.total_buffer_cost << "\n";
    for (const common::BufferInsertion& buf : result.buffers) {
        std::cout << "[BUFFERER] buffer id=" << buf.id
                  << " type=" << buf.type_name
                  << " node_id=" << buf.node_id
                  << " loc=(" << buf.loc.x << "," << buf.loc.y << ")\n";
    }
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const common::TopoNode& node = tree.nodes[i];
        std::cout << "[BUFFERER] node=" << static_cast<int>(i)
                  << " kind=" << static_cast<int>(node.kind)
                  << " sink_count=" << node.sink_indices.size() << "\n";
    }
}

bool write_debug_buffer_file(const common::BuffererResult& result,
                             const common::RouterResult& route_result,
                             const std::string& input_path,
                             std::string& error_msg) {
    if (!ensure_buffer_dir(error_msg)) return false;
    const std::string path = make_debug_buffer_path(input_path);
    std::ofstream fout(path);
    if (!fout) {
        error_msg = "Cannot open buffer debug file: " + path;
        return false;
    }
    fout << "# BUFFERER_DEBUG v1\n";
    fout << "# valid=" << (result.valid ? 1 : 0) << "\n";
    fout << "# num_buffers=" << result.buffers.size() << "\n";
    fout << "# total_buffer_cost=" << result.total_buffer_cost << "\n";
    for (const common::BufferInsertion& buf : result.buffers) {
        fout << "buffer " << buf.id << " " << buf.type_name << " "
             << buf.node_id << " " << buf.loc.x << " " << buf.loc.y << "\n";
    }
    for (std::size_t i = 0; i < route_result.edge_debugs.size(); ++i) {
        const common::RouterEdgeDebug& edge = route_result.edge_debugs[i];
        fout << "edge " << edge.edge_id << " " << edge.parent << " " << edge.child
             << " 0 " << edge.polyline.size();
        for (const common::SegmentPoint& p : edge.polyline)
            fout << " " << p.x << " " << p.y;
        fout << "\n";
    }
    return true;
}

common::BuffererResult run(const common::Problem& problem,
                           const common::TopoTree& tree,
                           const common::LocerResult& loc_result,
                           common::RouterResult& route_result,
                           const std::string& input_path) {
    common::BuffererResult result;

    std::string err;
    if (!validate_inputs(problem, tree, loc_result, route_result, err)) {
        result.error_msg = err;
        return result;
    }
    if (problem.buffer_types.empty()) {
        result.valid = true;
        return result;
    }

    const std::vector<int> bottom_up_order = build_bottom_up_order(tree);

    std::map<EdgeKey, int> edge_delay;
    build_edge_delay_map(route_result, edge_delay);

    const long long total_wirelength = compute_total_wirelength(route_result);

    std::vector<int> sink_count;
    compute_downstream_sink_count(tree, bottom_up_order, sink_count);

    std::vector<int> buffer_at(tree.nodes.size(), -1);

    std::vector<DelayProfile> profiles;
    compute_delay_profiles(tree, bottom_up_order, edge_delay, buffer_at,
                           problem, profiles);

    double obj_skew = compute_objective_skew(tree, profiles, edge_delay,
                                              problem, buffer_at);
    long long buffer_cost = 0;
    long long cur_score = compute_score(obj_skew, total_wirelength, buffer_cost);

    double initial_skew = obj_skew;
    long long initial_score = cur_score;

    if (g_debug_enabled) {
        std::cout << "[BUFFERER] initial skew=" << obj_skew
                  << " score=" << cur_score
                  << " wirelength=" << total_wirelength << "\n";
    }

    std::vector<int> candidate_nodes;
    for (int node_id : bottom_up_order) {
        const common::TopoNode& node = tree.nodes[static_cast<std::size_t>(node_id)];
        if (node.is_sink) continue;
        if (sink_count[static_cast<std::size_t>(node_id)] == 0) continue;
        candidate_nodes.push_back(node_id);
    }

    int committed = 0;
    for (int pass = 0; pass < MAX_PASSES; ++pass) {
        int best_node = -1;
        int best_buf = -1;
        long long best_score = cur_score;

        for (int node_id : candidate_nodes) {
            if (buffer_at[static_cast<std::size_t>(node_id)] >= 0) continue;
            const int node_sinks = sink_count[static_cast<std::size_t>(node_id)];

            for (std::size_t bi = 0; bi < problem.buffer_types.size(); ++bi) {
                const common::BufferType& buf = problem.buffer_types[bi];
                if (buf.max_fanout < node_sinks) {
                    if (g_debug_enabled) {
                        std::cout << "[BUFFERER] reject node=" << node_id
                                  << " buf=" << buf.name
                                  << " reason=FANOUT_ILLEGAL sinks=" << node_sinks
                                  << " fanout=" << buf.max_fanout << "\n";
                    }
                    continue;
                }

                buffer_at[static_cast<std::size_t>(node_id)] = static_cast<int>(bi);
                compute_delay_profiles(tree, bottom_up_order, edge_delay,
                                       buffer_at, problem, profiles);
                double new_skew = compute_objective_skew(tree, profiles, edge_delay,
                                                          problem, buffer_at);
                long long new_buf_cost = 0;
                for (int b : buffer_at)
                    if (b >= 0) new_buf_cost += problem.buffer_types[static_cast<std::size_t>(b)].cost;
                long long new_score = compute_score(new_skew, total_wirelength, new_buf_cost);
                buffer_at[static_cast<std::size_t>(node_id)] = -1;

                if (new_score > best_score ||
                    (new_score == best_score && new_skew < obj_skew)) {
                    best_score = new_score;
                    best_node = node_id;
                    best_buf = static_cast<int>(bi);
                    if (g_debug_enabled) {
                        std::cout << "[BUFFERER] pass=" << pass
                                  << " candidate node=" << node_id
                                  << " buf=" << buf.name
                                  << " new_skew=" << new_skew
                                  << " delta=" << (new_score - cur_score) << "\n";
                    }
                } else if (g_debug_enabled) {
                    std::cout << "[BUFFERER] reject node=" << node_id
                              << " buf=" << buf.name
                              << " reason=NO_SCORE_IMPROVEMENT"
                              << " new_skew=" << new_skew
                              << " delta=" << (new_score - cur_score) << "\n";
                }
            }
        }

        if (best_node < 0) break;

        buffer_at[static_cast<std::size_t>(best_node)] = best_buf;
        const common::BufferType& chosen_buf =
            problem.buffer_types[static_cast<std::size_t>(best_buf)];
        compute_delay_profiles(tree, bottom_up_order, edge_delay,
                               buffer_at, problem, profiles);
        obj_skew = compute_objective_skew(tree, profiles, edge_delay,
                                           problem, buffer_at);
        buffer_cost = 0;
        for (int b : buffer_at)
            if (b >= 0) buffer_cost += problem.buffer_types[static_cast<std::size_t>(b)].cost;
        cur_score = compute_score(obj_skew, total_wirelength, buffer_cost);

        const std::string buffer_id = "BUF_" + std::to_string(committed);
        const common::LocerNodeResult& loc_node =
            loc_result.node_results[static_cast<std::size_t>(best_node)];
        common::BufferInsertion insertion;
        insertion.id = buffer_id;
        insertion.type_name = chosen_buf.name;
        insertion.node_id = best_node;
        insertion.loc = loc_node.loc;
        result.buffers.push_back(insertion);
        result.total_buffer_cost += chosen_buf.cost;
        ++committed;

        if (g_debug_enabled) {
            std::cout << "[BUFFERER] commit pass=" << pass
                      << " node=" << best_node
                      << " buf=" << chosen_buf.name
                      << " skew=" << obj_skew
                      << " score=" << cur_score
                      << " delta=" << (cur_score - initial_score) << "\n";
        }
    }

    if (g_debug_enabled) {
        std::cout << "[BUFFERER] final skew=" << obj_skew
                  << " score=" << cur_score
                  << " initial_skew=" << initial_skew
                  << " initial_score=" << initial_score
                  << " committed=" << committed << "\n";
    }

    result.valid = true;

    if (g_debug_enabled)
        debug_output(result, problem, tree, loc_result, route_result);
    if (g_debug_file_enabled) {
        std::string file_error;
        if (!write_debug_buffer_file(result, route_result, input_path, file_error)) {
            result.valid = false;
            result.error_msg = file_error;
        }
    }
    return result;
}

}  // namespace bufferer
