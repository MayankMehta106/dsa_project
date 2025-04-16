//in this code user has to give the node id as input 
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include <limits>
#include <cmath>
#include <algorithm>

using namespace std;

struct Node {
    long long id;
    double lat;
    double lon;
};

struct Way {
    long long id;
    vector<long long> node_refs;
};

unordered_map<long long, Node> nodes;
vector<Way> ways;
unordered_map<long long, vector<pair<long long, double>>> graph;

// Haversine distance
double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double dphi = (lat2 - lat1) * M_PI / 180.0;
    double dlambda = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dphi / 2.0) * sin(dphi / 2.0) +
               cos(phi1) * cos(phi2) * sin(dlambda / 2.0) * sin(dlambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return R * c;
}

// Extract attribute value from a tag line
string getAttribute(const string& line, const string& attr) {
    size_t start = line.find(attr + "=\"");
    if (start == string::npos) return "";
    start += attr.length() + 2;
    size_t end = line.find("\"", start);
    return line.substr(start, end - start);
}

bool parseOSM(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Failed to open file!" << endl;
        return false;
    }

    string line;
    Way currentWay;
    bool inWay = false;
    bool isHighway = false;
    
    while (getline(file, line)) {
        if (line.find("<node") != string::npos) {
            Node node;
            node.id = stoll(getAttribute(line, "id"));
            node.lat = stod(getAttribute(line, "lat"));
            node.lon = stod(getAttribute(line, "lon"));
            nodes[node.id] = node;
        }
        else if (line.find("<way") != string::npos) {
            currentWay = Way();
            currentWay.id = stoll(getAttribute(line, "id"));
            currentWay.node_refs.clear();
            inWay = true;
            isHighway = false;
        }
        else if (line.find("<nd") != string::npos && inWay) {
            long long ref = stoll(getAttribute(line, "ref"));
            currentWay.node_refs.push_back(ref);
        }
        else if (line.find("<tag") != string::npos && inWay) {
            string k = getAttribute(line, "k");
            string v = getAttribute(line, "v");
            if (k == "highway") {
                isHighway = true;
            }
        }
        else if (line.find("</way>") != string::npos && inWay) {
            if (isHighway) {
                ways.push_back(currentWay);
            }
            inWay = false;
        }
    }
    
    return true;
}

void buildGraph() {
    for (const Way& way : ways) {
        for (size_t i = 0; i + 1 < way.node_refs.size(); ++i) {
            long long from = way.node_refs[i];
            long long to = way.node_refs[i + 1];
            if (nodes.find(from) != nodes.end() && nodes.find(to) != nodes.end()) {
                const Node& fromNode = nodes[from];
                const Node& toNode = nodes[to];
                double distance = haversine(fromNode.lat, fromNode.lon, toNode.lat, toNode.lon);
                graph[from].emplace_back(to, distance);
                graph[to].emplace_back(from, distance);
            }
        }
    }
}

vector<long long> dijkstra(long long start, long long goal) {
    unordered_map<long long, double> dist;
    unordered_map<long long, long long> prev;
    set<pair<double, long long>> pq;
    
    // Initialize distances
    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        dist[it->first] = numeric_limits<double>::infinity();
    }
    
    dist[start] = 0.0;
    pq.insert(make_pair(0.0, start));
    
    while (!pq.empty()) {
        auto top = *pq.begin();
        pq.erase(pq.begin());
        
        double currDist = top.first;
        long long u = top.second;
        
        if (u == goal) break;
        
        if (graph.find(u) == graph.end()) continue;
        
        vector<pair<long long, double>> neighbors = graph[u];
        for (size_t i = 0; i < neighbors.size(); ++i) {
            long long v = neighbors[i].first;
            double weight = neighbors[i].second;
            double alt = currDist + weight;
            
            if (alt < dist[v]) {
                pq.erase(make_pair(dist[v], v));
                dist[v] = alt;
                prev[v] = u;
                pq.insert(make_pair(alt, v));
            }
        }
    }
    
    // Reconstruct path
    vector<long long> path;
    long long at = goal;
    
    if (prev.find(goal) == prev.end()) {
        // No path found
        return path;
    }
    
    while (at != start) {
        path.push_back(at);
        at = prev[at];
    }
    
    path.push_back(start);
    reverse(path.begin(), path.end());
    
    return path;
}

long long findNearestNode(double lat, double lon) {
    long long nearestId = -1;
    double minDist = numeric_limits<double>::max();
    
    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        double d = haversine(lat, lon, it->second.lat, it->second.lon);
        if (d < minDist) {
            minDist = d;
            nearestId = it->first;
        }
    }
    
    return nearestId;
}

void exportGraphToCSV(const string& nodeFile, const string& edgeFile) {
    ofstream nodeOut(nodeFile);
    ofstream edgeOut(edgeFile);
    
    nodeOut << "id,lat,lon\n";
    edgeOut << "from,to,distance\n";
    
    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
        nodeOut << it->first << "," << it->second.lat << "," << it->second.lon << "\n";
    }
    
    for (auto it = graph.begin(); it != graph.end(); ++it) {
        long long from = it->first;
        for (size_t i = 0; i < it->second.size(); ++i) {
            long long to = it->second[i].first;
            double dist = it->second[i].second;
            edgeOut << from << "," << to << "," << dist << "\n";
        }
    }
    
    nodeOut.close();
    edgeOut.close();
}

void exportPathToCSV(const vector<long long>& path, const string& filename) {
    ofstream out(filename);
    out << "lat,lon\n";
    
    for (long long nodeId : path) {
        if (nodes.find(nodeId) != nodes.end()) {
            const Node& node = nodes[nodeId];
            out << node.lat << "," << node.lon << "\n";
        }
    }
    
    out.close();
}

// New function to check if a node ID exists in the graph
bool nodeExists(long long id) {
    return nodes.find(id) != nodes.end();
}

// New function to list available nodes (optional, for user convenience)
void listSomeNodes(int count) {
    cout << "Listing " << count << " sample nodes from the dataset:" << endl;
    int i = 0;
    for (auto it = nodes.begin(); it != nodes.end() && i < count; ++it, ++i) {
        cout << "Node ID: " << it->first << " (lat: " << it->second.lat << ", lon: " << it->second.lon << ")" << endl;
    }
}
int main() {
    string filename = "northern-zone-latest.osm_01_01.osm";
    
    if (parseOSM(filename)) {
        cout << "Parsed " << nodes.size() << " nodes and " << ways.size() << " ways!" << endl;
        
        // Display some sample nodes and ways for reference
        int count = 0;
        for (auto it = nodes.begin(); it != nodes.end() && count < 5; ++it, ++count) {
            const Node& node = it->second;
            cout << "Node ID: " << node.id << " (lat: " << node.lat << ", lon: " << node.lon << ")" << endl;
        }
        
        for (size_t i = 0; i < ways.size() && i < 5; ++i) {
            const Way& way = ways[i];
            cout << "Way ID: " << way.id << ", nodes: ";
            for (long long ref : way.node_refs) {
                cout << ref << " ";
            }
            cout << endl;
        }
        
        buildGraph();
        
        count = 0;
        for (auto it = graph.begin(); it != graph.end() && count < 5; ++it, ++count) {
            long long nodeId = it->first;
            cout << "Node " << nodeId << " has neighbors: ";
            for (const auto& neighbor : it->second) {
                cout << "(" << neighbor.first << ", " << neighbor.second << "m) ";
            }
            cout << endl;
        }
        
        exportGraphToCSV("nodes.csv", "edges.csv");
        
        // User input for start and end node IDs
        long long start, end;
        bool validInput = false;
        
        while (!validInput) {
            cout << "Enter the start node ID: ";
            cin >> start;
            
            if (!nodeExists(start)) {
                cout << "Error: Node ID " << start << " does not exist in the dataset." << endl;
                cout << "Would you like to see some sample node IDs? (y/n): ";
                char choice;
                cin >> choice;
                if (choice == 'y' || choice == 'Y') {
                    listSomeNodes(10);
                }
                continue;
            }
            
            cout << "Enter the end node ID: ";
            cin >> end;
            
            if (!nodeExists(end)) {
                cout << "Error: Node ID " << end << " does not exist in the dataset." << endl;
                continue;
            }
            
            validInput = true;
        }
        
        cout << "Start Node ID: " << start << " (lat: " << nodes[start].lat << ", lon: " << nodes[start].lon << ")" << endl;
        cout << "End Node ID: " << end << " (lat: " << nodes[end].lat << ", lon: " << nodes[end].lon << ")" << endl;
        
        vector<long long> path = dijkstra(start, end);
        
        cout << "Shortest path from start to end:" << endl;
        if (path.size() > 1) {
            double totalDist = 0.0;
            cout << "Path: ";
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                const Node& a = nodes[path[i]];
                const Node& b = nodes[path[i + 1]];
                double d = haversine(a.lat, a.lon, b.lat, b.lon);
                totalDist += d;
                cout << path[i] << " -> ";
            }
            cout << path.back() << "\nTotal distance: " << totalDist << " meters\n";
        } else {
            cout << "No path found between the selected nodes.\n";
        }
        
        exportPathToCSV(path, "path.csv");
    } else {
        cout << "Failed to parse OSM file!" << endl;
    }
    
    return 0;
} 