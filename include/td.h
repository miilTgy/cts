#pragma once

#include "common.h"

namespace td {

using ClusterDmeInput = common::ClusterDmeInput;
using TopDownConfig = common::TopDownConfig;
using TopDownResult = common::TopDownResult;
using TopDownNodeResult = common::TopDownNodeResult;
using SegmentPoint = common::SegmentPoint;
using MergingSegment = common::MergingSegment;

void debug_enable(bool enable);
void debug_output(const TopDownResult& result,
                  const common::Problem& problem,
                  const ClusterDmeInput& input,
                  const common::BottomUpResult& bu_result,
                  const TopDownConfig& config);
TopDownResult run(const common::Problem& problem,
                  const ClusterDmeInput& input,
                  const common::BottomUpResult& bu_result,
                  const TopDownConfig& config);

}  // namespace td
