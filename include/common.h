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

struct TreeNode {
    int id = -1;

    bool is_leaf = false;
    int sink_index = -1;

    int parent = -1;
    int left = -1;
    int right = -1;

    int sink_count = 0;

    double cx = 0.0;
    double cy = 0.0;

    int bbox_lx = 0;
    int bbox_ly = 0;
    int bbox_ux = 0;
    int bbox_uy = 0;

    int region_lx = 0;
    int region_ly = 0;
    int region_ux = 0;
    int region_uy = 0;

    double est_delay = 0.0;
};

struct TopologyTree {
    std::vector<TreeNode> nodes;
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
