#pragma once

#include <string>

#include "common.h"

namespace treer {

using TopologyTree = common::TopologyTree;
using TreeNode = common::TreeNode;

void debug_enable(bool enable);
void debug_output(const TopologyTree& tree, const common::Problem& problem);
void debug_output_file(const TopologyTree& tree,
                       const common::Problem& problem,
                       const std::string& input_path);
TopologyTree build(const common::Problem& problem, const std::string& input_path = "");

}  // namespace treer
