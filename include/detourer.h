#pragma once

#include <string>

#include "common.h"

namespace detourer {

void debug_enable(bool enable);
void debug_file_enable(bool enable);
void debug_output(const common::DetourerResult& result,
                  const common::Problem& problem,
                  const common::TopoTree& tree,
                  const common::LocerResult& loc_result,
                  const common::RouterResult& route_result);
bool write_debug_detour_file(const common::DetourerResult& result,
                             const common::RouterResult& route_result,
                             const std::string& input_path,
                             std::string& error_msg);

common::DetourerResult run(const common::Problem& problem,
                           const common::TopoTree& tree,
                           const common::LocerResult& loc_result,
                           common::RouterResult& route_result,
                           const std::string& input_path = "");

}  // namespace detourer
