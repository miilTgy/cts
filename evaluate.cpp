#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

namespace {

constexpr long long SCORE_BASE = 5000000LL;
constexpr long long SKEW_WEIGHT = 5000LL;
constexpr long long WIRE_WEIGHT = 50LL;
constexpr long long BUFFER_WEIGHT = 200LL;

struct Coord {
    int x = 0;
    int y = 0;

    bool operator==(const Coord& other) const {
        return x == other.x && y == other.y;
    }
};

struct CoordHash {
    size_t operator()(const Coord& p) const {
        return (static_cast<size_t>(static_cast<unsigned int>(p.x)) << 32U) ^
               static_cast<size_t>(static_cast<unsigned int>(p.y));
    }
};

struct EdgeKey {
    Coord a;
    Coord b;

    bool operator==(const EdgeKey& other) const {
        return a == other.a && b == other.b;
    }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey& e) const {
        return CoordHash{}(e.a) * 1315423911ULL ^ CoordHash{}(e.b);
    }
};

struct BufferType {
    string name;
    int delay = 0;
    int max_fanout = 0;
    int cost = 0;
};

struct Sink {
    string id;
    Coord point;
};

struct Buffer {
    string id;
    string type_name;
    Coord point;
};

struct Route {
    string sink_id;
    vector<Coord> points;
};

struct InputData {
    int die_width = 0;
    int die_height = 0;
    string source_id;
    Coord source;
    vector<Sink> sinks;
    unordered_map<string, BufferType> buffer_types;
};

struct OutputData {
    vector<Buffer> buffers;
    vector<Route> routes;
};

string coordKey(const Coord& p) {
    return to_string(p.x) + "," + to_string(p.y);
}

EdgeKey makeEdgeKey(const Coord& u, const Coord& v) {
    if (u.x < v.x || (u.x == v.x && u.y < v.y)) {
        return {u, v};
    }
    return {v, u};
}

bool inDie(const InputData& input, const Coord& p) {
    return 0 <= p.x && p.x <= input.die_width &&
           0 <= p.y && p.y <= input.die_height;
}

void printIllegal(const string& message) {
    cout << "LEGAL NO\n";
    cout << "MESSAGE " << message << "\n";
}

bool parseInput(const string& path, InputData& data, string& error) {
    ifstream in(path);
    if (!in) {
        error = "Cannot open input file";
        return false;
    }

    string token;
    if (!(in >> token) || token != "DIE" || !(in >> data.die_width >> data.die_height)) {
        error = "Invalid DIE line";
        return false;
    }
    if (!(in >> token) || token != "SOURCE" ||
        !(in >> data.source_id >> data.source.x >> data.source.y)) {
        error = "Invalid SOURCE line";
        return false;
    }

    int num_sinks = 0;
    if (!(in >> token) || token != "NUM_SINKS" || !(in >> num_sinks)) {
        error = "Invalid NUM_SINKS line";
        return false;
    }

    unordered_set<string> used_ids;
    used_ids.insert(data.source_id);
    for (int i = 0; i < num_sinks; ++i) {
        Sink sink;
        if (!(in >> token) || token != "SINK" ||
            !(in >> sink.id >> sink.point.x >> sink.point.y)) {
            error = "Invalid SINK line";
            return false;
        }
        if (used_ids.count(sink.id)) {
            error = "Duplicate node id in input: " + sink.id;
            return false;
        }
        used_ids.insert(sink.id);
        data.sinks.push_back(sink);
    }

    int num_types = 0;
    if (!(in >> token) || token != "NUM_BUFFERS" || !(in >> num_types)) {
        error = "Invalid NUM_BUFFERS line";
        return false;
    }
    for (int i = 0; i < num_types; ++i) {
        BufferType type;
        if (!(in >> token) || token != "BUFFER_TYPE" ||
            !(in >> type.name >> type.delay >> type.max_fanout >> type.cost)) {
            error = "Invalid BUFFER_TYPE line";
            return false;
        }
        if (data.buffer_types.count(type.name)) {
            error = "Duplicate buffer type: " + type.name;
            return false;
        }
        data.buffer_types[type.name] = type;
    }
    return true;
}

bool parseOutput(const string& path, OutputData& data, string& error) {
    ifstream in(path);
    if (!in) {
        error = "Cannot open output file";
        return false;
    }

    string token;
    int num_bufs = 0;
    if (!(in >> token) || token != "NUM_BUFS" || !(in >> num_bufs)) {
        error = "Invalid NUM_BUFS line";
        return false;
    }
    for (int i = 0; i < num_bufs; ++i) {
        Buffer buffer;
        if (!(in >> token) || token != "BUF" ||
            !(in >> buffer.id >> buffer.type_name >> buffer.point.x >> buffer.point.y)) {
            error = "Invalid BUF line";
            return false;
        }
        data.buffers.push_back(buffer);
    }

    int num_routes = 0;
    if (!(in >> token) || token != "NUM_ROUTES" || !(in >> num_routes)) {
        error = "Invalid NUM_ROUTES line";
        return false;
    }
    for (int i = 0; i < num_routes; ++i) {
        Route route;
        int point_count = 0;
        if (!(in >> token) || token != "ROUTE" ||
            !(in >> route.sink_id >> point_count)) {
            error = "Invalid ROUTE line";
            return false;
        }
        if (point_count < 2) {
            error = "ROUTE must contain at least two points";
            return false;
        }
        route.points.resize(point_count);
        for (int j = 0; j < point_count; ++j) {
            if (!(in >> route.points[j].x >> route.points[j].y)) {
                error = "Invalid ROUTE point";
                return false;
            }
        }
        data.routes.push_back(route);
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 3) {
        cerr << "Usage: ./evaluate <input.txt> <output.txt>\n";
        return 1;
    }

    InputData input;
    OutputData output;
    string error;
    if (!parseInput(argv[1], input, error)) {
        cerr << error << '\n';
        return 1;
    }
    if (!parseOutput(argv[2], output, error)) {
        cerr << error << '\n';
        return 1;
    }

    unordered_map<string, Coord> sink_point_by_id;
    unordered_map<string, string> sink_id_by_coord;
    unordered_set<string> all_node_ids;
    all_node_ids.insert(input.source_id);
    for (const auto& sink : input.sinks) {
        sink_point_by_id[sink.id] = sink.point;
        sink_id_by_coord[coordKey(sink.point)] = sink.id;
        all_node_ids.insert(sink.id);
    }

    unordered_map<string, BufferType> buffer_type_by_id;
    unordered_map<string, Coord> buffer_point_by_id;
    unordered_map<string, string> buffer_id_by_coord;
    for (const auto& buffer : output.buffers) {
        if (all_node_ids.count(buffer.id)) {
            printIllegal("Duplicate node id in output: " + buffer.id);
            return 0;
        }
        all_node_ids.insert(buffer.id);
        if (!input.buffer_types.count(buffer.type_name)) {
            printIllegal("Unknown buffer type: " + buffer.type_name);
            return 0;
        }
        if (!inDie(input, buffer.point)) {
            printIllegal("Buffer out of die: " + buffer.id);
            return 0;
        }

        string key = coordKey(buffer.point);
        if (key == coordKey(input.source) || sink_id_by_coord.count(key) || buffer_id_by_coord.count(key)) {
            printIllegal("Buffer coordinates must be unique and distinct from SOURCE/SINKs");
            return 0;
        }

        buffer_type_by_id[buffer.id] = input.buffer_types[buffer.type_name];
        buffer_point_by_id[buffer.id] = buffer.point;
        buffer_id_by_coord[key] = buffer.id;
    }

    if (static_cast<int>(output.routes.size()) != static_cast<int>(input.sinks.size())) {
        printIllegal("NUM_ROUTES must equal NUM_SINKS");
        return 0;
    }

    unordered_set<string> seen_sink_routes;
    unordered_set<EdgeKey, EdgeKeyHash> unique_edges;
    unordered_set<Coord, CoordHash> all_vertices;
    all_vertices.insert(input.source);

    for (const auto& route : output.routes) {
        if (!sink_point_by_id.count(route.sink_id)) {
            printIllegal("Unknown sink id in ROUTE: " + route.sink_id);
            return 0;
        }
        if (seen_sink_routes.count(route.sink_id)) {
            printIllegal("Duplicate ROUTE for sink: " + route.sink_id);
            return 0;
        }
        seen_sink_routes.insert(route.sink_id);

        if (!(route.points.front() == input.source)) {
            printIllegal("Each ROUTE must start at SOURCE");
            return 0;
        }
        if (!(route.points.back() == sink_point_by_id[route.sink_id])) {
            printIllegal("Each ROUTE must end at its sink");
            return 0;
        }

        unordered_set<string> route_vertices;
        Coord prev = route.points.front();
        if (!inDie(input, prev)) {
            printIllegal("ROUTE point out of die");
            return 0;
        }
        route_vertices.insert(coordKey(prev));
        all_vertices.insert(prev);

        for (size_t i = 1; i < route.points.size(); ++i) {
            Coord next = route.points[i];
            if (!inDie(input, next)) {
                printIllegal("ROUTE point out of die");
                return 0;
            }
            if (prev == next) {
                printIllegal("ROUTE contains zero-length segment");
                return 0;
            }
            if (prev.x != next.x && prev.y != next.y) {
                printIllegal("ROUTE segments must be horizontal or vertical");
                return 0;
            }

            int dx = (next.x == prev.x) ? 0 : ((next.x > prev.x) ? 1 : -1);
            int dy = (next.y == prev.y) ? 0 : ((next.y > prev.y) ? 1 : -1);
            Coord current = prev;
            while (!(current == next)) {
                Coord step{current.x + dx, current.y + dy};
                string step_key = coordKey(step);
                if (route_vertices.count(step_key)) {
                    printIllegal("Each ROUTE must be simple");
                    return 0;
                }
                route_vertices.insert(step_key);
                all_vertices.insert(step);
                unique_edges.insert(makeEdgeKey(current, step));
                current = step;
            }
            prev = next;
        }

        for (const auto& sink : input.sinks) {
            if (sink.id != route.sink_id && route_vertices.count(coordKey(sink.point))) {
                printIllegal("ROUTE cannot pass through another sink");
                return 0;
            }
        }
    }

    if (static_cast<int>(seen_sink_routes.size()) != static_cast<int>(input.sinks.size())) {
        printIllegal("Missing ROUTE for some sink");
        return 0;
    }

    unordered_map<Coord, vector<Coord>, CoordHash> graph;
    for (const auto& edge : unique_edges) {
        graph[edge.a].push_back(edge.b);
        graph[edge.b].push_back(edge.a);
    }

    if (!graph.count(input.source)) {
        printIllegal("No routed wire starts from SOURCE");
        return 0;
    }

    for (const auto& sink : input.sinks) {
        if (!graph.count(sink.point)) {
            printIllegal("Sink not reachable from SOURCE: " + sink.id);
            return 0;
        }
    }

    for (const auto& buffer : output.buffers) {
        if (!graph.count(buffer.point)) {
            printIllegal("Every BUFFER must lie on at least one route");
            return 0;
        }
    }

    unordered_set<Coord, CoordHash> visited;
    unordered_map<Coord, Coord, CoordHash> parent;
    unordered_map<Coord, long long, CoordHash> wire_dist;
    vector<Coord> order;

    queue<Coord> q;
    q.push(input.source);
    visited.insert(input.source);
    parent[input.source] = {-1, -1};
    wire_dist[input.source] = 0;

    while (!q.empty()) {
        Coord u = q.front();
        q.pop();
        order.push_back(u);
        for (const auto& v : graph[u]) {
            if (visited.count(v)) continue;
            visited.insert(v);
            parent[v] = u;
            wire_dist[v] = wire_dist[u] + 1;
            q.push(v);
        }
    }

    if (visited.size() != all_vertices.size()) {
        printIllegal("The union of routes must be connected");
        return 0;
    }
    if (static_cast<long long>(unique_edges.size()) != static_cast<long long>(all_vertices.size()) - 1LL) {
        printIllegal("The union of routes must form a tree");
        return 0;
    }

    for (const auto& sink : input.sinks) {
        if (graph[sink.point].size() != 1U) {
            printIllegal("Each SINK must be a tree leaf");
            return 0;
        }
    }

    unordered_map<string, BufferType> buffer_type_by_coord;
    for (const auto& buffer : output.buffers) {
        buffer_type_by_coord[coordKey(buffer.point)] = input.buffer_types[buffer.type_name];
    }

    unordered_map<Coord, vector<Coord>, CoordHash> children;
    for (const auto& entry : parent) {
        if (entry.first == input.source) continue;
        children[entry.second].push_back(entry.first);
    }

    unordered_map<Coord, long long, CoordHash> subtree_sink_count;
    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        long long count = sink_id_by_coord.count(coordKey(*it)) ? 1LL : 0LL;
        for (const auto& child : children[*it]) {
            count += subtree_sink_count[child];
        }
        subtree_sink_count[*it] = count;
    }

    for (const auto& buffer : output.buffers) {
        const auto& type = input.buffer_types[buffer.type_name];
        if (subtree_sink_count[buffer.point] > type.max_fanout) {
            printIllegal("Buffer downstream sink count exceeds max_fanout: " + buffer.id);
            return 0;
        }
    }

    unordered_map<Coord, long long, CoordHash> delay_sum;
    delay_sum[input.source] = 0;
    for (const auto& u : order) {
        for (const auto& child : children[u]) {
            long long extra = 0;
            auto it = buffer_type_by_coord.find(coordKey(child));
            if (it != buffer_type_by_coord.end()) {
                extra = it->second.delay;
            }
            delay_sum[child] = delay_sum[u] + extra;
        }
    }

    long long min_arrival = numeric_limits<long long>::max();
    long long max_arrival = numeric_limits<long long>::min();
    for (const auto& sink : input.sinks) {
        long long arrival = wire_dist[sink.point] + delay_sum[sink.point];
        min_arrival = min(min_arrival, arrival);
        max_arrival = max(max_arrival, arrival);
    }

    long long skew = max_arrival - min_arrival;
    long long wirelength = static_cast<long long>(unique_edges.size());
    long long buffercost = 0;
    for (const auto& buffer : output.buffers) {
        buffercost += input.buffer_types[buffer.type_name].cost;
    }
    long long score = SCORE_BASE
                      - SKEW_WEIGHT * skew
                      - WIRE_WEIGHT * wirelength
                      - BUFFER_WEIGHT * buffercost;

    cout << "LEGAL YES\n";
    cout << "SKEW " << skew << "\n";
    cout << "WIRELENGTH " << wirelength << "\n";
    cout << "BUFFERCOST " << buffercost << "\n";
    cout << "SCORE " << score << "\n";
    return 0;
}
