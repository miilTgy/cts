#pragma once

#include <string>

#include "common.h"

namespace partitioner {

using PartitionTree = common::PartitionTree;
using PartitionNode = common::PartitionNode;

void debug_enable(bool enable);
void debug_output(const PartitionTree& tree);
void write_output_file(const PartitionTree& tree, const common::Problem& problem, const std::string& input_path);
PartitionTree build(const common::Problem& problem);

}  // namespace partitioner
