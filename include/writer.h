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
                            const common::TopologyTree& tree,
                            const common::BottomUpResult& bu_result,
                            const common::TopDownResult& td_result);

}  // namespace writer
