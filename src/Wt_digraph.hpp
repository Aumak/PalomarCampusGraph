// File: Wt_digraph.hpp
// Description: Weighted undirected graph implementation using templates and STL

#ifndef WT_DIGRAPH_HPP
#define WT_DIGRAPH_HPP

#include <iostream>
#include <unordered_map>
#include <map>
#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <cmath>
#include <set>
#include <algorithm>  


// Abstract Graph base class
template<typename KEY_TYPE>
class Graph {
public:
    virtual int build_graph() = 0;
    virtual int is_graph_empty() = 0;
    virtual void find_shortest_path() = 0;
    virtual int get_min() = 0;
    virtual void show_paths(KEY_TYPE src, KEY_TYPE dst) = 0;
    virtual int check_graph() = 0;
    virtual void print_graph() = 0;
    virtual ~Graph() {}
};

// Weighted undirected graph class
template<typename KEY_TYPE>
class Wt_digraph : public Graph<KEY_TYPE> {
private:
    struct Vertex {
        KEY_TYPE key;
        double x, y; // Coordinates for suggestion features
        std::unordered_map<KEY_TYPE, int> edges; // Adjacent vertex and weight
    };

    std::unordered_map<KEY_TYPE, Vertex> vertices;

    double euclidean_distance(double x1, double y1, double x2, double y2) {
        return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

public:
    // Basic graph construction methods
    void add_vertex(KEY_TYPE key, double x = 0, double y = 0) {
        vertices[key] = { key, x, y, {} };
    }

    void add_edge(KEY_TYPE from, KEY_TYPE to, int weight) {
        vertices[from].edges[to] = weight;
        vertices[to].edges[from] = weight;
    }

    // Graph traversal: Breadth-first search
    void bfs(KEY_TYPE start) {
        std::unordered_map<KEY_TYPE, bool> visited;
        std::queue<KEY_TYPE> queue;
        queue.push(start);
        visited[start] = true;

        while (!queue.empty()) {
            KEY_TYPE current = queue.front(); queue.pop();
            std::cout << current << " ";
            for (auto& [adj, _] : vertices[current].edges) {
                if (!visited[adj]) {
                    visited[adj] = true;
                    queue.push(adj);
                }
            }
        }
        std::cout << "\n";
    }

    // Graph traversal: Depth-first search
    void dfs(KEY_TYPE start) {
        std::unordered_map<KEY_TYPE, bool> visited;
        std::stack<KEY_TYPE> stack;
        stack.push(start);

        while (!stack.empty()) {
            KEY_TYPE current = stack.top(); stack.pop();
            if (!visited[current]) {
                visited[current] = true;
                std::cout << current << " ";
                for (auto& [adj, _] : vertices[current].edges) {
                    if (!visited[adj]) {
                        stack.push(adj);
                    }
                }
            }
        }
        std::cout << "\n";
    }

    // Shortest path using Dijkstra
    void dijkstra(KEY_TYPE start, std::unordered_map<KEY_TYPE, int>& dist, std::unordered_map<KEY_TYPE, KEY_TYPE>& prev) {
        using pii = std::pair<int, KEY_TYPE>;
        std::priority_queue<pii, std::vector<pii>, std::greater<pii>> pq;
        std::set<KEY_TYPE> visited;

        for (auto& [key, _] : vertices) dist[key] = std::numeric_limits<int>::max();
        dist[start] = 0;
        pq.emplace(0, start);

        while (!pq.empty()) {
            auto [d, u] = pq.top(); pq.pop();
            if (visited.count(u)) continue;
            visited.insert(u);

            for (auto& [v, w] : vertices[u].edges) {
                if (dist[u] + w < dist[v]) {
                    dist[v] = dist[u] + w;
                    prev[v] = u;
                    pq.emplace(dist[v], v);
                }
            }
        }
    }

    void show_paths(KEY_TYPE src, KEY_TYPE dst) override {
        std::unordered_map<KEY_TYPE, int> dist;
        std::unordered_map<KEY_TYPE, KEY_TYPE> prev;
        dijkstra(src, dist, prev);

        if (dist[dst] == std::numeric_limits<int>::max()) {
            std::cout << "No path found from " << src << " to " << dst << "\n";
            return;
        }

        std::vector<KEY_TYPE> path;
        for (KEY_TYPE at = dst; at != src; at = prev[at])
            path.push_back(at);
        path.push_back(src);

        std::reverse(path.begin(), path.end());

        std::cout << "Path: ";
        for (auto v : path) std::cout << v << " ";
        std::cout << "\nCost: " << dist[dst] << ", Hops: " << path.size() - 1 << "\n";
    }

    // Nearby location suggestion
    void suggest_nearby(KEY_TYPE source, double radius_miles) {
        double lat1 = vertices[source].x;
        double lon1 = vertices[source].y;

        std::cout << "Nearby locations within " << radius_miles << " miles:\n";
        for (auto& [id, vtx] : vertices) {
            if (id == source) continue;
            double dist_deg = euclidean_distance(lat1, lon1, vtx.x, vtx.y);
            double miles = dist_deg * 69.0; // rough degree-to-mile conversion
            if (miles <= radius_miles)
                std::cout << " - " << id << " (" << miles << " miles)\n";
        }
    }

    // Required base overrides
    int build_graph() override { return 1; }
    int is_graph_empty() override { return vertices.empty(); }
    void find_shortest_path() override {} // Not needed due to show_paths
    int get_min() override { return 0; }
    int check_graph() override { return 1; }

    void print_graph() override {
        for (auto& [key, vtx] : vertices) {
            std::cout << "Vertex " << key << ": ";
            for (auto& [adj, weight] : vtx.edges)
                std::cout << adj << "(" << weight << ") ";
            std::cout << "\n";
        }
    }
};

#endif // WT_DIGRAPH_HPP
