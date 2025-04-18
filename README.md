# DSA PROJECT

## Overview
This project implements an efficient path-finding solution for Jodhpur city using OpenStreetMap data. It showcases the practical application of fundamental Data Structures and Algorithms (DSA) concepts through a real-world navigation problem. The implementation uses C++ to build a graph representation of Jodhpur's road network and applies Dijkstra's algorithm to find optimal routes between locations.

## Key Features 

- Parses OpenStreetMap (OSM) data for Jodhpur city.

- Uses Harvesine Formula to calculate the distance (weights) between the points (nodes).

- Builds a weighted undirected graph representation of the road network.

- Implements Dijkstra's algorithm for shortest path calculation.

- Exports calculated paths to CSV files for visualization.

- Provides a command-line interface for selecting start and end points.

## Technical Implementation
### Data Structures
- Graph: Implemented as an adjacency list using unordered_map<long long, vector<pair<long long, double>>> for efficient neighbor access

- Priority Queue: Implemented using C++ set<pair<double, long long>> to efficiently select nodes with minimum distance

- Node Storage: Uses unordered_map<long long, Node> to store node information including ID, latitude, and longitude

- Distance Tracking: Maintains distance and predecessor maps during algorithm execution

### Algorithms
- Dijkstra's Algorithm: Optimized implementation with the following features:

- Priority queue for efficient node selection

- Early termination when destination is reached

- Path reconstruction to generate the final route

- Haversine Formula: Calculates the great-circle distance between two points on Earth's surface

## Project Structure

- main.cpp: Core implementation of the path-finding algorithm

- nodes.csv: Contains node IDs with their latitude/longitude coordinates

- edges.csv: Contains connections between nodes with distance weights

- path.csv: Contains the calculated route coordinates from the starting point to the destination

- map.html: Visualization interface for the calculated paths

- northern-zone-latest.osm_01_01.osm: The osm file which contains the data of Jodhpur map

## Dependencies
- C++ compiler with C++11 support

- Standard Template Library (STL)

- OpenStreetMap data for Jodhpur region

## Usage 
1. Run the compiled program
2.  Wait for the the program to display sample nodes
3.  Give the start and end node for the task (ex - Mehrangarh fort (node ID - 12388645022)  and the School of odd thinkers (node ID - 12725285247))
4.  The program will then display the path and give the distance
5.  Run the .html file to see the path on web

## Algorithm Details

 The project implements Dijkstra's algorithm with the following optimizations:

- Priority queue (min-heap) for efficient node selection

- Haversine formula for accurate geographical distance calculations

- Early termination when destination is reached


[Download the file from Google Drive](https://drive.google.com/file/d/1VYIUOehGnX_jFng920PKt46XlDMOeZrB/view?usp=drive_link)
