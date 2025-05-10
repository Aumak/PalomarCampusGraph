#ifndef MAPREP_HPP
#define MAPREP_HPP

#include <map>
#include <string>
#include <vector>

// Structure to represent each location (node)
struct Location {
    std::string name;
    double lat;
    double lon;
    std::vector<std::string> adjacencies; // List of adjacent locations (edges)
};

// Function to parse the map file and load data into a map
std::map<std::string, Location> loadMapData(const std::string& filename);

// Function to generate DOT format for Graphviz
void generateGraphvizDOT(const std::map<std::string, Location>& locations, const std::string& outputFilename);

#endif // MAPREP_HPP
