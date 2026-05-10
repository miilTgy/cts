#pragma once

#include <string>

#include "common.h"

namespace treer {

using TopoTree = common::TopoTree;
using TopoNode = common::TopoNode;

void debug_enable(bool enable);
void debug_output(const TopoTree& tree, const common::Problem& problem);
void debug_output_file(const TopoTree& tree,
                       const common::Problem& problem,
                       const std::string& input_path);
TopoTree build(const common::Problem& problem,
               const common::PartitionTree& partition_tree,
               const std::string& sample_name_or_input_path = "");

}  // namespace treer
