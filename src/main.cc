#include <iostream>

#include "common.h"
#include "parser.h"
#include "treer.h"
#include "bu.h"
#include "td.h"
#include "writer.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./main <input.txt>\n";
        return 1;
    }

    parser::debug_enable(true);
    treer::debug_enable(true);
    bu::debug_enable(true);
    td::debug_enable(true);
    writer::debug_enable(true);

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

    // Temporarily stop after topology generation. Re-enable the following
    // stages when BU/TD/Writer are ready to be run in the full CTS flow.
    
    common::BottomUpResult bu_result = bu::run(problem, tree);
    if (!bu_result.valid) {
        std::cerr << "BU error: " << bu_result.error_msg << "\n";
        return 1;
    }
    
    common::TopDownResult td_result = td::run(problem, tree, bu_result);
    if (!td_result.valid) {
        std::cerr << "TD error: " << td_result.error_msg << "\n";
        return 1;
    }
    
    writer::WriterResult writer_result =
        writer::write_solution(argv[1], problem, tree, bu_result, td_result);
    if (!writer_result.valid) {
        std::cerr << "Writer error: " << writer_result.error_msg << "\n";
        return 1;
    }
    
    std::cout << "Wrote solution: " << writer_result.output_path << "\n";

    return 0;
}
