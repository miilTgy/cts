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

    double est_delay = 0.0;
};

struct TopologyTree {
    std::vector<TreeNode> nodes;
    int root = -1;
    bool valid = false;
    std::string error_msg;
};

}  // namespace common
