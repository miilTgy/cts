#pragma once

#include <map>
#include <string>
#include <vector>

namespace common {

struct Point {
    int x = 0;
    int y = 0;
};

struct Sink {
    std::string id;
    Point loc;
};

struct BufferType {
    std::string name;
    int delay = 0;
    int max_fanout = 0;
    int cost = 0;
};

struct Problem {
    int die_width = 0;
    int die_height = 0;
    Sink source;
    std::vector<Sink> sinks;
    std::vector<BufferType> buffer_types;
    bool valid = false;
    std::string error_msg;
};

struct BBox {
    int lx = 0;
    int ly = 0;
    int ux = 0;
    int uy = 0;
};

enum class NodeKind {
    Sink,
    ClusterInternal,
    ClusterAccess,
    ClusterBridge,
    ClusterTop,
    Global
};

struct TopoNode {
    int id = -1;
    Point loc;

    int parent = -1;
    int left = -1;
    int right = -1;

    bool is_sink = false;
    int sink_index = -1;

    std::vector<int> sink_indices;
    BBox bbox;

    int left_min_delay_to_node = 0;
    int left_max_delay_to_node = 0;
    int left_skew_to_node = 0;

    int right_min_delay_to_node = 0;
    int right_max_delay_to_node = 0;
    int right_skew_to_node = 0;

    int subtree_min_delay_to_node = 0;
    int subtree_max_delay_to_node = 0;
    int subtree_skew_to_node = 0;

    // Region is kept for the BU/TD stages, which use it as a geometric
    // feasibility hint after topology construction.
    int region_lx = 0;
    int region_ly = 0;
    int region_ux = 0;
    int region_uy = 0;

    NodeKind kind = NodeKind::ClusterInternal;
};

struct TopoTree {
    std::vector<TopoNode> nodes;
    int root = -1;
    int cluster_root = -1;
    std::vector<int> source_children;
    bool valid = false;
    std::string error_msg;
};

struct PartitionNode {
    int id = -1;
    std::vector<int> children;
    std::vector<int> sink_indices;
    int bbox_lx = 0;
    int bbox_ly = 0;
    int bbox_ux = 0;
    int bbox_uy = 0;
    Point centroid;
    bool is_leaf = false;
    bool is_outlier = false;
};

struct PartitionTree {
    std::vector<PartitionNode> nodes;
    int root = -1;
    bool valid = false;
    std::string error_msg;
};

struct SegmentPoint {
    double x = 0.0;
    double y = 0.0;
};

struct MergingSegment {
    SegmentPoint p1;
    SegmentPoint p2;
    bool valid = false;
};

struct BufferChoice {
    bool has_buffer = false;
    int buffer_type_index = -1;
};

enum class DmeNodeClass {
    Sink,
    Internal,
    Access
};

struct ClusterDmeNode {
    int local_id = -1;
    int origin_node_id = -1;
    DmeNodeClass node_class = DmeNodeClass::Internal;
    int left = -1;
    int right = -1;
    int parent = -1;
    int sink_index = -1;
    int sink_count = 0;
};

struct ClusterDmeInput {
    int cluster_id = -1;
    int root_local_id = -1;
    int root_origin_node_id = -1;
    std::vector<ClusterDmeNode> nodes;
    bool valid = false;
    std::string error_msg;
};

struct BottomUpNodeResult {
    int node_id = -1;
    int local_id = -1;
    int origin_node_id = -1;
    bool valid = false;

    MergingSegment ms;
    double edge_to_left = 0.0;
    double edge_to_right = 0.0;

    BufferChoice buffer_at_left_child;
    BufferChoice buffer_at_right_child;

    double min_delay = 0.0;
    double max_delay = 0.0;
    double skew = 0.0;

    double wire_est = 0.0;
    int buffer_cost = 0;
    double total_cost = 0.0;

    std::string extraction_mode;
    double detour_to_left = 0.0;
    double detour_to_right = 0.0;
};

struct BottomUpResult {
    int cluster_id = -1;
    std::vector<BottomUpNodeResult> node_results;
    std::vector<int> local_to_origin_node_id;
    int root_local_id = -1;
    int root_origin_node_id = -1;
    int root = -1;
    bool valid = false;
    std::string error_msg;
};

struct TopDownConfig {
    bool has_root_loc = false;
    SegmentPoint root_loc;
    std::string root_loc_mode;
};

struct TopDownNodeResult {
    int node_id = -1;
    int local_id = -1;
    int origin_node_id = -1;
    bool valid = false;

    SegmentPoint loc;
    int parent_id = -1;
    int parent_local_id = -1;
    int parent_origin_node_id = -1;

    double assigned_edge_to_parent = 0.0;
    double geometric_distance_to_parent = 0.0;
    double routed_length_to_parent = 0.0;
    double compensation_detour_to_parent = 0.0;
    double final_length_to_parent = 0.0;

    std::vector<SegmentPoint> route_to_parent;
    std::vector<SegmentPoint> compensation_snake_to_parent;
    std::vector<SegmentPoint> final_route_to_parent;

    MergingSegment feasible_ms;
    bool used_feasible_intersection = false;
    std::string loc_mode;
    int candidate_count = 0;
    double loc_score = 0.0;

    bool has_buffer = false;
    int buffer_type_index = -1;

    double min_delay = 0.0;
    double max_delay = 0.0;
    double skew = 0.0;

    double td_common_extra_delay = 0.0;
};

struct TopDownResult {
    int cluster_id = -1;
    std::vector<TopDownNodeResult> node_results;
    std::vector<int> local_to_origin_node_id;
    int root_local_id = -1;
    int root_origin_node_id = -1;
    int root = -1;
    bool valid = false;
    std::string error_msg;
};

struct LocerNodeResult {
    int node_id = -1;
    bool valid = false;

    SegmentPoint loc;
    std::string node_class;
    int cluster_id = -1;

    std::string loc_mode;
    double loc_score = 0.0;
    int candidate_count = 0;

    bool inside_related_bbox = false;
    double congestion_penalty = 0.0;
    double lshape_penalty = 0.0;
    double wire_est_to_parent = 0.0;

    std::vector<double> sink_delays_to_node;
    double min_sink_delay_to_node = 0.0;
    double max_sink_delay_to_node = 0.0;
    double skew_to_node = 0.0;
    double skew_penalty = 0.0;
};

struct LocerResult {
    std::vector<LocerNodeResult> node_results;
    bool valid = false;
    std::string error_msg;
};

struct RouterEdgeDebug {
    int edge_id = -1;
    int parent = -1;
    int child = -1;
    std::string parent_class;
    std::string child_class;
    std::string policy;
    std::string selected_shape;
    std::string parent_exit_dir;
    std::string child_entry_dir;

    bool parent_port_available = false;
    bool child_port_available = false;
    bool used_preferred_parent = false;
    bool used_preferred_child = false;

    int pattern_candidate_count = 0;
    int maze_candidate_count = 0;
    int legal_candidate_count = 0;
    bool maze_used = false;
    int maze_expanded_nodes = 0;
    double maze_best_cost = 0.0;
    std::string maze_failed_reason;

    double selected_score = 0.0;
    double wirelength = 0.0;
    int bends = 0;

    bool routed = false;
    std::string failure_reason;
    std::vector<SegmentPoint> polyline;

    std::map<std::string, int> reject_stats;
};

struct RouterResult {
    bool valid = false;
    std::string error_msg;
    std::string output_path;
    std::vector<RouterEdgeDebug> edge_debugs;
};

struct DetourRecord {
    int edge_id = -1;
    int node_parent = -1;
    int node_child = -1;
    int segment_index = -1;
    int anchor_index = -1;
    int level = 1;
    int added_delay = 2;
    std::string side;
    bool upgraded = false;
};

struct DetourNodeResult {
    int node_id = -1;
    bool valid = false;
    std::vector<double> sink_delays_to_node;
    double min_sink_delay_to_node = 0.0;
    double max_sink_delay_to_node = 0.0;
    double skew_to_node = 0.0;
};

struct DetourerResult {
    bool valid = false;
    std::string error_msg;
    std::vector<DetourRecord> detour_records;
    std::vector<DetourNodeResult> node_results;
};

}  // namespace common
