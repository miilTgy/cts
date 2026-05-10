#pragma once

#include <vector>

#include "common.h"

namespace partreer {

void debug_enable(bool enable);
void debug_output(const common::TopoTree& tree);

common::TopoTree build(const common::Problem& problem,
                       const std::vector<int>& sink_indices,
                       common::Point external_target);

}  // namespace partreer
