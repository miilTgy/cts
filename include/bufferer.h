#pragma once

#include <string>

#include "common.h"

namespace bufferer {

void debug_enable(bool enable);
void debug_file_enable(bool enable);

void debug_output(const common::BuffererResult& result,
                  const common::Problem& problem,
                  const common::TopoTree& tree,
                  const common::LocerResult& loc_result,
                  const common::RouterResult& route_result);

bool write_debug_buffer_file(const common::BuffererResult& result,
                             const common::RouterResult& route_result,
                             const std::string& input_path,
                             std::string& error_msg);

common::BuffererResult run(const common::Problem& problem,
                           const common::TopoTree& tree,
                           const common::LocerResult& loc_result,
                           common::RouterResult& route_result,
                           const std::string& input_path);

}  // namespace bufferer
