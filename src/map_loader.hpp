// File: map_loader.hpp
// Description: Functions to load vertices and edges from input files

#ifndef MAP_LOADER_HPP
#define MAP_LOADER_HPP

#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <tuple>
#include <string>

// Struct to store each point's data
struct Point {
    std::string name;
    double lat;
    double lon;
    std::vector<std::string> adjacent_locations; // list of connected locations
};

// Load vertices from palomarMap.txt
std::map<std::string, Point> readVertices(const std::string& path) {
    std::map<std::string, Point> vertices;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error opening vertex file: " << path << "\n";
        return vertices;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string name;
        double lat, lon;
        Point pt;
        
        // Parsing the name, latitude, longitude
        std::getline(iss, name, ',');
        iss >> lat;
        iss.ignore(); // skip comma
        iss >> lon;

        pt.name = name;
        pt.lat = lat;
        pt.lon = lon;

        // Parse remaining connections (adjacent locations)
        std::string connected;
        while (std::getline(iss, connected, ',')) {
            pt.adjacent_locations.push_back(connected);
        }

        vertices[name] = pt;  // Store the point by its name
    }
    return vertices;
}

#endif // MAP_LOADER_HPP
