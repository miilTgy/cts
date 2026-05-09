#pragma once

#include "common.h"

namespace bu {

using BottomUpResult = common::BottomUpResult;
using BottomUpNodeResult = common::BottomUpNodeResult;
using MergingSegment = common::MergingSegment;
using BufferChoice = common::BufferChoice;

void debug_enable(bool enable);
void debug_output(const BottomUpResult& result,
                  const common::Problem& problem,
                  const common::TopologyTree& tree);
BottomUpResult run(const common::Problem& problem,
                   const common::TopologyTree& tree);

}  // namespace bu
