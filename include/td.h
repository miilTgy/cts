#pragma once

#include "common.h"

namespace td {

using TopDownResult = common::TopDownResult;
using TopDownNodeResult = common::TopDownNodeResult;
using SegmentPoint = common::SegmentPoint;
using MergingSegment = common::MergingSegment;

void debug_enable(bool enable);
void debug_output(const TopDownResult& result,
                  const common::Problem& problem,
                  const common::TopologyTree& tree,
                  const common::BottomUpResult& bu_result);
TopDownResult run(const common::Problem& problem,
                  const common::TopologyTree& tree,
                  const common::BottomUpResult& bu_result);

}  // namespace td
