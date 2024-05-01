//
//

#ifndef DA_PROJ2_REALWORLDGRAPH_H
#define DA_PROJ2_REALWORLDGRAPH_H

#include <vector>
#include <iostream>
#include <climits>

using namespace std;

class RealWorldGraph {

    int nVertices =0;
    int nEdges=0;
    int graphType; //0 - tourism.csv, 1 - stadiums.csv, 2 - shipping.csv

    class Edge {
    public:
        int destination;
        float distance;
    };

    class Vertex {
    public:
        int index;
        double longitude;
        double latitude;
        vector<Edge> adj; //adjacency list
        bool visited = false;

    };

    RealWorldGraph();
    explicit RealWorldGraph(int nVertices);

    vector<Vertex> vertices;

    const vector<Vertex>& getVertices() const;

    RealWorldGraph::Vertex getVertex(int i);

    void setNumVertices(int nVert);
    void setNumEdges (int nEdges);
    int getGraphType();
    void setGraphType(int graphType);
    int numEdges();
    int numVertices();
    int addVertex(int vertex);
    int addVertexLongLat(int vertex, double longi, double lat);
    float findEdgeCost(int startVertex, int endVertex) const;
    void addEdge(int origin, int destination, float distance);

};


#endif //DA_PROJ2_REALWORLDGRAPH_H
