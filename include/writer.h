#pragma once

#include <string>

#include "common.h"

namespace writer {

struct WriterResult {
    bool valid = false;
    std::string error_msg;
    std::string output_path;
};

void debug_enable(bool enable);

WriterResult write_solution(const std::string& input_path,
                            const common::Problem& problem,
                            const common::TopoTree& tree,
                            const common::LocerResult& loc_result,
                            const common::RouterResult& router_result,
                            const common::BuffererResult& bufferer_result);

}  // namespace writer
