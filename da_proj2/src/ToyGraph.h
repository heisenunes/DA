#ifndef DA_PROJ2_TOYGRAPH_H
#define DA_PROJ2_TOYGRAPH_H

#include <vector>
#include <iostream>
#include <climits>

using namespace std;

class ToyGraph {

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
        string label;
        vector<Edge> adj; //adjacency list
        bool visited = false;

    };


    /** auxiliary function that finds the path with mininum cost/distance **/
    void tspBacktracking(int currVertex, vector<bool>& visited,vector<int>& currentPath, vector<int>& bestPath,int startVertex, int numVisited, float currentCost, float& minCost);
    //**debug version to store discarded paths and their cost **//
    void tspBacktrackingDebug(int currVertex, vector<bool>& visited,vector<int>& currentPath, vector<int>& bestPath,vector<pair<vector<int>, float>>& discardedPaths,int startVertex, int numVisited, float currentCost, float& minCost);
public:
    //**//
    ToyGraph();
    explicit ToyGraph(int nVertices);

    vector<Vertex> vertices;

    //vector<ToyGraph::Vertex>getVertices();
    const vector<Vertex>& getVertices() const;

    ToyGraph::Vertex getVertex(int i);

    void setNumVertices(int nVert);
    void setNumEdges (int nEdges);
    int getGraphType();
    void setGraphType(int graphType);
    int numEdges();
    int numVertices();
    int addVertex(int vertex);
    int addVertexLabel(int vertex, string label);
    float findEdgeCost(int startVertex, int endVertex) const;
    void addEdge(int origin, int destination, float distance);

    //**BackTracking Algorithm**//
    float tsp();

    //debugging//
    float tspDebug();
    vector<pair<vector<int>, float>> discardedPaths;  // stores  all paths that were not chosen as well as their respective costs//
    const vector<pair<vector<int>, float>>& getDiscardedPaths() const { return discardedPaths; }


};


#endif //DA_PROJ2_TOYGRAPH_H
