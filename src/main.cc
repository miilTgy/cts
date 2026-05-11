#include <iostream>
#include <filesystem>

#include "common.h"
#include "parser.h"
#include "treer.h"
#include "bu.h"
#include "td.h"
#include "locer.h"
#include "router.h"
#include "writer.h"
#include "partitioner.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./main <input.txt>\n";
        return 1;
    }

    parser::debug_enable(true);
    partitioner::debug_enable(true);
    treer::debug_enable(true);
    bu::debug_enable(true);
    td::debug_enable(true);
    locer::debug_enable(true);
    locer::debug_file_enable(true);
    router::debug_enable(true);
    router::debug_file_enable(true);
    writer::debug_enable(true);

    common::Problem problem = parser::parse(argv[1]);
    if (!problem.valid) {
        std::cerr << "Parser error: " << problem.error_msg << "\n";
        return 1;
    }

    common::PartitionTree partition = partitioner::build(problem);
    partitioner::write_output_file(partition, problem, argv[1]);
    std::cout << "Partitioner: wrote part/" + std::string(argv[1]).substr(std::string(argv[1]).find_last_of("/\\") + 1) << "\n";

    treer::TopoTree tree = treer::build(problem, partition, argv[1]);
    if (!tree.valid) {
        std::cerr << "Treer error: " << tree.error_msg << "\n";
        return 1;
    }

    common::LocerResult loc_result = locer::run(problem, tree, argv[1]);
    if (!loc_result.valid) {
        std::cerr << "LOCER error: " << loc_result.error_msg << "\n";
        return 1;
    }
    std::filesystem::path input_path(argv[1]);
    std::cout << "Locer: wrote loc/" << input_path.stem().string()
              << "_loc.txt\n";

    common::RouterResult route_result = router::run(problem, tree, loc_result, argv[1]);
    if (!route_result.valid) {
        std::cerr << "ROUTER error: " << route_result.error_msg << "\n";
        return 1;
    }
    if (!route_result.output_path.empty()) {
        std::cout << "Router: wrote " << route_result.output_path << "\n";
    }

    return 0;
}
