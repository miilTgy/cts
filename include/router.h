#pragma once

#include <string>

#include "common.h"

namespace router {

using RouterResult = common::RouterResult;
using RouterEdgeDebug = common::RouterEdgeDebug;

void debug_enable(bool enable);
void debug_file_enable(bool enable);
void debug_output(const RouterResult& result,
                  const common::Problem& problem,
                  const common::TopoTree& tree,
                  const common::LocerResult& loc_result);
bool write_debug_route_file(const RouterResult& result,
                            const common::Problem& problem,
                            const common::TopoTree& tree,
                            const common::LocerResult& loc_result,
                            const std::string& input_path,
                            std::string& error_msg);
RouterResult run(const common::Problem& problem,
                 const common::TopoTree& tree,
                 const common::LocerResult& loc_result,
                 const std::string& input_path = "");

}  // namespace router
