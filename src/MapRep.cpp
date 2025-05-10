#include "MapRep.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

// Function to parse the map file and load data into a map
std::map<std::string, Location> loadMapData(const std::string& filename) {
    std::map<std::string, Location> locations;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return locations;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream stream(line);
        Location location;
        std::string name, adj;
        double lat, lon;

        // Read the location name, latitude, and longitude
        std::getline(stream, name, ',');
        stream >> lat;
        stream.ignore(); // skip the comma
        stream >> lon;

        location.name = name;
        location.lat = lat;
        location.lon = lon;

        // Read the adjacent locations
        while (std::getline(stream, adj, ',')) {
            location.adjacencies.push_back(adj);
        }

        // Store the location in the map
        locations[name] = location;
    }

    return locations;
}

// Function to generate DOT format for Graphviz
void generateGraphvizDOT(const std::map<std::string, Location>& locations, const std::string& outputFilename) {
    std::ofstream outFile(outputFilename);
    if (!outFile.is_open()) {
        std::cerr << "Error opening output file: " << outputFilename << std::endl;
        return;
    }

    outFile << "graph campus_map {\n";
    
    // Add nodes with their coordinates
    for (const auto& [name, location] : locations) {
        outFile << "  \"" << name << "\" [label=\"" << name << "\" shape=circle width=0.5 style=filled fillcolor=skyblue pos=\"" 
                << location.lon << "," << location.lat << "!\"];\n";
    }

    // Add edges between adjacent locations
    for (const auto& [name, location] : locations) {
        for (const auto& adj : location.adjacencies) {
            outFile << "  \"" << name << "\" -- \"" << adj << "\";\n";
        }
    }

    outFile << "}\n";
}
