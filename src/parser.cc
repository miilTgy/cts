#include "parser.h"

#include <fstream>
#include <iostream>
#include <istream>
#include <string>
#include <unordered_set>

namespace parser {
namespace {

bool g_debug_enabled = false;

static std::string got_text(const std::string& token, bool ok) {
    return ok ? token : "EOF";
}

static bool read_keyword(std::istream& in, const std::string& expected, std::string& err) {
    std::string keyword;
    const bool ok = static_cast<bool>(in >> keyword);
    if (!ok || keyword != expected) {
        err = "Expected " + expected + ", got " + got_text(keyword, ok);
        return false;
    }
    return true;
}

static bool check_point_in_die(const common::Problem& problem,
                               const common::Point& p,
                               const std::string& label,
                               std::string& err) {
    // HW3 samples and diagrams use an inclusive die boundary.
    if (p.x < 0 || p.x > problem.die_width ||
        p.y < 0 || p.y > problem.die_height) {
        err = label + " coordinate out of die boundary: (" +
              std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
        return false;
    }
    return true;
}

static bool read_sink(std::istream& in,
                      const std::string& expected_keyword,
                      common::Sink& sink,
                      std::string& err) {
    if (!read_keyword(in, expected_keyword, err)) {
        return false;
    }
    if (!(in >> sink.id >> sink.loc.x >> sink.loc.y)) {
        err = "Invalid " + expected_keyword + " record";
        return false;
    }
    return true;
}

static bool read_buffer_type(std::istream& in, common::BufferType& buf, std::string& err) {
    if (!read_keyword(in, "BUFFER_TYPE", err)) {
        return false;
    }
    if (!(in >> buf.name >> buf.delay >> buf.max_fanout >> buf.cost)) {
        err = "Invalid BUFFER_TYPE record";
        return false;
    }
    if (buf.delay < 0 || buf.max_fanout <= 0 || buf.cost < 0) {
        err = "Invalid BUFFER_TYPE values for " + buf.name +
              ": delay and cost must be nonnegative, max_fanout must be positive";
        return false;
    }
    return true;
}

}  // namespace

void debug_enable(bool enable) {
    g_debug_enabled = enable;
}

void debug_output(const common::Problem& problem) {
    if (!g_debug_enabled) {
        return;
    }

    std::cout << "PARSER_DEBUG\n";
    std::cout << "VALID " << (problem.valid ? 1 : 0) << "\n";
    std::cout << "ERROR_MSG " << problem.error_msg << "\n";
    std::cout << "DIE " << problem.die_width << " " << problem.die_height << "\n";
    std::cout << "SOURCE " << problem.source.id << " "
              << problem.source.loc.x << " " << problem.source.loc.y << "\n";
    std::cout << "NUM_SINKS " << problem.sinks.size() << "\n";
    for (std::size_t i = 0; i < problem.sinks.size(); ++i) {
        const common::Sink& sink = problem.sinks[i];
        std::cout << "SINK " << i << " " << sink.id << " "
                  << sink.loc.x << " " << sink.loc.y << "\n";
    }
    std::cout << "NUM_BUFFER_TYPES " << problem.buffer_types.size() << "\n";
    for (std::size_t i = 0; i < problem.buffer_types.size(); ++i) {
        const common::BufferType& buf = problem.buffer_types[i];
        std::cout << "BUFFER_TYPE " << i << " " << buf.name << " "
                  << buf.delay << " " << buf.max_fanout << " " << buf.cost << "\n";
    }
    std::cout << "END_PARSER_DEBUG\n";
}

common::Problem parse(const std::string& in_path) {
    common::Problem problem;
    std::ifstream fin(in_path);
    if (!fin) {
        problem.error_msg = "Cannot open input file: " + in_path;
        return problem;
    }

    std::string err;
    if (!read_keyword(fin, "DIE", err) ||
        !(fin >> problem.die_width >> problem.die_height)) {
        problem.error_msg = err.empty() ? "Invalid DIE record" : err;
        return problem;
    }
    if (problem.die_width < 0 || problem.die_height < 0) {
        problem.error_msg = "DIE width and height must be nonnegative";
        return problem;
    }

    if (!read_sink(fin, "SOURCE", problem.source, err)) {
        problem.error_msg = err;
        return problem;
    }
    if (!check_point_in_die(problem, problem.source.loc, "SOURCE", err)) {
        problem.error_msg = err;
        return problem;
    }

    if (!read_keyword(fin, "NUM_SINKS", err)) {
        problem.error_msg = err;
        return problem;
    }
    int num_sinks = 0;
    if (!(fin >> num_sinks)) {
        problem.error_msg = "Invalid NUM_SINKS value";
        return problem;
    }
    if (num_sinks < 0) {
        problem.error_msg = "NUM_SINKS must be nonnegative";
        return problem;
    }

    std::unordered_set<std::string> node_ids;
    node_ids.insert(problem.source.id);
    problem.sinks.reserve(static_cast<std::size_t>(num_sinks));
    for (int i = 0; i < num_sinks; ++i) {
        common::Sink sink;
        if (!read_sink(fin, "SINK", sink, err)) {
            problem.error_msg = err;
            return problem;
        }
        if (node_ids.count(sink.id) != 0U) {
            problem.error_msg = "Duplicate node id: " + sink.id;
            return problem;
        }
        if (!check_point_in_die(problem, sink.loc, "SINK " + sink.id, err)) {
            problem.error_msg = err;
            return problem;
        }
        node_ids.insert(sink.id);
        problem.sinks.push_back(sink);
    }

    if (!read_keyword(fin, "NUM_BUFFERS", err)) {
        problem.error_msg = err;
        return problem;
    }
    int num_buffers = 0;
    if (!(fin >> num_buffers)) {
        problem.error_msg = "Invalid NUM_BUFFERS value";
        return problem;
    }
    if (num_buffers < 0) {
        problem.error_msg = "NUM_BUFFERS must be nonnegative";
        return problem;
    }

    std::unordered_set<std::string> buffer_names;
    problem.buffer_types.reserve(static_cast<std::size_t>(num_buffers));
    for (int i = 0; i < num_buffers; ++i) {
        common::BufferType buf;
        if (!read_buffer_type(fin, buf, err)) {
            problem.error_msg = err;
            return problem;
        }
        if (buffer_names.count(buf.name) != 0U) {
            problem.error_msg = "Duplicate buffer type: " + buf.name;
            return problem;
        }
        buffer_names.insert(buf.name);
        problem.buffer_types.push_back(buf);
    }

    std::string extra;
    if (fin >> extra) {
        problem.error_msg = "Unexpected trailing token: " + extra;
        return problem;
    }

    problem.valid = true;
    if (g_debug_enabled) {
        debug_output(problem);
    }
    return problem;
}

}  // namespace parser
