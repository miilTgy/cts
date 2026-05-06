#include <iostream>

#include "common.h"
#include "parser.h"
#include "treer.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./main <input.txt>\n";
        return 1;
    }

    parser::debug_enable(true);
    treer::debug_enable(true);

    common::Problem problem = parser::parse(argv[1]);
    if (!problem.valid) {
        std::cerr << "Parser error: " << problem.error_msg << "\n";
        return 1;
    }

    treer::TopologyTree tree = treer::build(problem, argv[1]);
    if (!tree.valid) {
        std::cerr << "Treer error: " << tree.error_msg << "\n";
        return 1;
    }

    return 0;
}
