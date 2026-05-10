#pragma once

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

struct BottomUpNodeResult {
    int node_id = -1;
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
    std::vector<BottomUpNodeResult> node_results;
    int root = -1;
    bool valid = false;
    std::string error_msg;
};

struct TopDownNodeResult {
    int node_id = -1;
    bool valid = false;

    SegmentPoint loc;
    int parent_id = -1;

    double assigned_edge_to_parent = 0.0;
    double geometric_distance_to_parent = 0.0;
    double routed_length_to_parent = 0.0;
    double compensation_detour_to_parent = 0.0;
    double final_length_to_parent = 0.0;

    std::vector<SegmentPoint> route_to_parent;
    std::vector<SegmentPoint> compensation_snake_to_parent;
    std::vector<SegmentPoint> final_route_to_parent;

    MergingSegment feasible_ms;

    bool has_buffer = false;
    int buffer_type_index = -1;

    double min_delay = 0.0;
    double max_delay = 0.0;
    double skew = 0.0;

    double td_common_extra_delay = 0.0;
};

struct TopDownResult {
    std::vector<TopDownNodeResult> node_results;
    int root = -1;
    bool valid = false;
    std::string error_msg;
};

}  // namespace common
