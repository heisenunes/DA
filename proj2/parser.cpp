#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <unordered_map>
#include "data_structures/Graph.h"
#include <cmath>
#include <utility>

using namespace std;

struct vertice {
    double longitude;
    double latitude;
};

struct hash_pair {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2> &pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0; 
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}


Graph<int> construct_toygraph(const string &filename){
    Graph<int> g;
    //int edges = 0;
   
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return g; // Return empty vector
    }

    string line;
    getline(file, line); // Skip header line


     while (getline(file, line)) {
        stringstream ss(line);

        string origem,destino,distancia;

        getline(ss, origem, ',');
        getline(ss, destino, ',');
        getline(ss, distancia, ',');

        int origem_num = stoi(origem);
        int destino_num = stoi(destino);
       
        if(g.findVertex(origem_num) == nullptr){

             g.addVertex(origem_num);
 
        }

        if(g.findVertex(destino_num) == nullptr){
             g.addVertex(destino_num);

        }
        
      
        double distancia_num = stod(distancia);
       
      
        g.addBidirectionalEdge(origem_num,destino_num,distancia_num);
           
    }



    return g;
}

Graph<int> construct_extra_fully_connected(const string &filename){
    Graph<int> g;
    //int edges = 0;
   
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return g; // Return empty vector
    }

    string line;

     while (getline(file, line)) {
        stringstream ss(line);

        string origem,destino,distancia;

        getline(ss, origem, ',');
        getline(ss, destino, ',');
        getline(ss, distancia, ',');

        int origem_num = stoi(origem);
        int destino_num = stoi(destino);
       
        if(g.findVertex(origem_num) == nullptr){

             g.addVertex(origem_num);
 
        }

        if(g.findVertex(destino_num) == nullptr){
             g.addVertex(destino_num);

        }
        
      
        double distancia_num = stod(distancia);
       
      
        g.addBidirectionalEdge(origem_num,destino_num,distancia_num);

        
           
    }



    return g;
}
  
Graph<int> construct_realWorld(const string &vertexFile, const string &edgeFile) {
    Graph<int> g;
    std::unordered_map<int, vertice> vertices;
    std::unordered_map<std::pair<int, int>, double, hash_pair> edges;

    std::ifstream vfile(vertexFile);

    if (!vfile.is_open()) {
        std::cerr << "Error opening vertex file: " << vertexFile << std::endl;
        return g;
    }

    std::string line;
    getline(vfile, line); 

    while (getline(vfile, line)) {
        std::stringstream ss(line);

        std::string id, lat, lon;
        getline(ss, id, ',');
        getline(ss, lon, ',');
        getline(ss, lat, ',');

        int id_num = std::stoi(id);
        double lon_num = std::stod(lon);
        double lat_num = std::stod(lat);

        vertices[id_num] = {lat_num, lon_num};
        g.addVertex(id_num);
    }
    vfile.close();

    std::ifstream efile(edgeFile);
    if (!efile.is_open()) {
        std::cerr << "Error opening edge file: " << edgeFile << std::endl;
        return g;
    }

    getline(efile, line); 

    while (getline(efile, line)) {
        std::stringstream ss(line);

        std::string origem, destino, distancia;
        getline(ss, origem, ',');
        getline(ss, destino, ',');
        getline(ss, distancia, ',');

        int origem_num = std::stoi(origem);
        int destino_num = std::stoi(destino);
        double distancia_num = std::stod(distancia);

        edges[{origem_num, destino_num}] = distancia_num;
        edges[{destino_num, origem_num}] = distancia_num;
        g.addBidirectionalEdge(origem_num, destino_num, distancia_num);
    }
    efile.close();

    for (auto it1 = vertices.begin(); it1 != vertices.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != vertices.end(); ++it2) {
            int v1 = it1->first;
            int v2 = it2->first;
            if (edges.find({v1, v2}) == edges.end()) {
                double distance = haversine(it1->second.latitude, it1->second.longitude, it2->second.latitude, it2->second.longitude);
                g.addBidirectionalEdge(v1, v2, distance);
                edges[{v1, v2}] = distance;
                edges[{v2, v1}] = distance;
            }
        }
    }

    return g;

    return g;
}