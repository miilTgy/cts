#pragma once

#include "common.h"

#include <string>

namespace locer {

using LocerResult = common::LocerResult;
using LocerNodeResult = common::LocerNodeResult;

void debug_enable(bool enable);
void debug_file_enable(bool enable);
void debug_output(const LocerResult& result,
                  const common::Problem& problem,
                  const common::TopoTree& tree);
bool write_debug_loc_file(const LocerResult& result,
                          const common::Problem& problem,
                          const common::TopoTree& tree,
                          const std::string& input_path,
                          std::string& error_msg);
LocerResult run(const common::Problem& problem,
                const common::TopoTree& tree,
                const std::string& input_path = "");

}  // namespace locer
