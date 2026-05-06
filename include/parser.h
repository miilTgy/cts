#pragma once

#include <string>

#include "common.h"

namespace parser {

void debug_enable(bool enable);
void debug_output(const common::Problem& problem);
common::Problem parse(const std::string& in_path);

}  // namespace parser
