#include <iostream>
#include "Wt_digraph.hpp"
#include "MapRep.hpp"  

int main() {
    // Step 1: Load vertices and edges from provided map data file
    const std::string inputFilename = "/mnt/d/PalomarCampusGraph/Data/palomarMapEdge.txt";
    auto locations = loadMapData(inputFilename);  // Load map data

    // Step 2: Create the graph object (using the loaded locations data)
    Wt_digraph<std::string> graph;  // Change vertex type to std::string (or use int if you prefer)
    for (const auto& [name, location] : locations) {
        // Assuming the graph uses location names as IDs
        graph.add_vertex(name, location.lat, location.lon);
    }

    // Add edges based on adjacency list
    for (const auto& [name, location] : locations) {
        for (const auto& adj : location.adjacencies) {
            graph.add_edge(name, adj, 1);  // Weight 1 (or modify as needed)
        }
    }

    // Step 3: Generate Graphviz DOT file (optional visualization)
    generateGraphvizDOT(locations, "campus_map.dot");

    // Step 4: Demonstrate Graph Traversals (same as in the original code)
    std::cout << "\nBFS from vertex BuildingA:\n";
    graph.bfs("BuildingA");

    std::cout << "\nDFS from vertex BuildingA:\n";
    graph.dfs("BuildingA");

    // Step 5: Compute Shortest Path (modify based on string-based nodes)
    std::cout << "\nShortest path from BuildingA to BuildingB:\n";
    graph.show_paths("BuildingA", "BuildingB");

    // Step 6: Suggest Nearby Locations (optional)
    std::cout << "\nSuggestions near BuildingA within 0.1 miles:\n";
    graph.suggest_nearby("BuildingA", 0.1);

    return 0;
}
