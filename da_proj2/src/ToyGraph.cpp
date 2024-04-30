
#include "ToyGraph.h"

using namespace std;

ToyGraph::ToyGraph() {}

int ToyGraph::addVertex(int vertex) {
    int ind = (int)vertices.size();
    for (int i=0; i < nVertices; i++)
        if(vertices[i].index == vertex)
            return i;

    Vertex v = Vertex();
    v.index = vertex;

    vertices.push_back(v);
    nVertices++;
    return ind;
}

void ToyGraph::addEdge(int origin, int destination, float distance) {
    vertices[origin].adj.push_back({destination, distance});
    vertices[destination].adj.push_back({origin, distance});
    nEdges ++;
}

ToyGraph::ToyGraph(int nVertices) {}

void ToyGraph::setNumVertices(int nVert) {}

void ToyGraph::setNumEdges(int nEdges) {}

int ToyGraph::numEdges() {
    return nEdges;
}

int ToyGraph::numVertices() {
    return nVertices;
}

ToyGraph::Vertex ToyGraph::getVertex(int i) {
    return vertices[i];
}

const vector<ToyGraph::Vertex>& ToyGraph::getVertices() const {
    return vertices;
}

int ToyGraph::addVertexLabel(int vertex, string newLabel) {
    int ind = (int)vertices.size();
    for (int i=0; i < nVertices; i++)
        if(vertices[i].index == vertex)
            return i;

    Vertex v = Vertex();
    v.index = vertex;
    v.label = newLabel;

    vertices.push_back(v);
    nVertices++;
    return ind;
}

void ToyGraph::setGraphType(int newGraphType) {
    graphType = newGraphType;
}

int ToyGraph::getGraphType() {
    return graphType;
}

float ToyGraph::findEdgeCost(int startVertex, int endVertex) const {
    if (startVertex < 0 || startVertex >= vertices.size() || endVertex < 0 || endVertex >= vertices.size()) {
        throw out_of_range("Vertex index out of range");
    }

    for (const auto& edge : vertices[startVertex].adj) {
        if (edge.destination == endVertex) {
            return edge.distance;
        }
    }

    return std::numeric_limits<float>::infinity(); // returns infinity if no path was found //
}


void ToyGraph::tspBacktracking(int currVertex, vector<bool>& visited,vector<int>& currentPath, vector<int>& bestPath,
                               int startVertex, int numVisited, float currentCost, float& minCost) {

    // All vertices are visited and there is a return path to the start //
    if (numVisited == numVertices() && findEdgeCost(currVertex, startVertex) != std::numeric_limits<float>::infinity()) {
        float totalCost = currentCost + findEdgeCost(currVertex, startVertex);
        if (totalCost < minCost) {
            minCost = totalCost;
            bestPath = currentPath;
            bestPath.push_back(startVertex);  // return to the starting point 0 //
        }
        return;
    }

    for (const auto& edge : vertices[currVertex].adj) { // travel to other vertices that were not visited yet //
        if (!visited[edge.destination]) {
            visited[edge.destination] = true;
            currentPath.push_back(edge.destination);

            tspBacktracking(edge.destination, visited, currentPath, bestPath, startVertex, numVisited + 1,
                            currentCost + edge.distance, minCost);

            visited[edge.destination] = false; // backtrack//
            currentPath.pop_back();
        }
    }
}


float ToyGraph::tsp() {
    vector<bool> visited(numVertices(), false);
    vector<int> currentPath, bestPath;
    float minCost = std::numeric_limits<float>::infinity();

    visited[0] = true;  // vertex labeled 0 starting point //
    currentPath.push_back(0);

    tspBacktracking(0, visited, currentPath, bestPath, 0, 1, 0.0, minCost);

    //best path found output //
    cout << "- Minimum distance traveled is " << minCost << std::endl;
    cout << "- Optimal Path - " << endl;
    for (int v : bestPath) {
        cout << v << " -> ";
    }
    cout << endl;

    return minCost;
}


void ToyGraph::tspBacktrackingDebug(int currVertex, vector<bool>& visited,vector<int>& currentPath, vector<int>& bestPath,
                                    vector<pair<vector<int>, float>>& discardedPaths,
                                    int startVertex, int numVisited, float currentCost, float& minCost) {
    if (numVisited == numVertices() && findEdgeCost(currVertex, startVertex) != std::numeric_limits<float>::infinity()) {
        float totalCost = currentCost + findEdgeCost(currVertex, startVertex);
        if (totalCost < minCost) {
            minCost = totalCost;
            bestPath = currentPath;
            bestPath.push_back(startVertex);  //returning to the startint point//
        } else {
            discardedPaths.push_back({currentPath, totalCost});
        }
        return;
    }

    for (const auto& edge : vertices[currVertex].adj) {
        if (!visited[edge.destination]) {
            visited[edge.destination] = true;
            currentPath.push_back(edge.destination);

            tspBacktrackingDebug(edge.destination, visited, currentPath, bestPath, discardedPaths, startVertex, numVisited + 1,
                                 currentCost + edge.distance, minCost);

            //backtrack//
            discardedPaths.push_back({currentPath, currentCost + edge.distance}); // Include cost at backtrack
            visited[edge.destination] = false;
            currentPath.pop_back();
        }
    }
}

float ToyGraph::tspDebug() {
    vector<bool> visited(numVertices(), false);
    vector<int> currentPath, bestPath;
    vector<pair<vector<int>, float>> discardedPaths; // save discarded paths
    float minCost = std::numeric_limits<float>::infinity();

    visited[0] = true;  // vertex labeled 0 starting poing
    currentPath.push_back(0);

    tspBacktrackingDebug(0, visited, currentPath, bestPath, discardedPaths, 0, 1, 0.0, minCost);

    //best path found output //
    cout << "- Minimum distance traveled is " << minCost << std::endl;
    cout << "- Optimal Path - " << endl;
    for (int v : bestPath) {
        std::cout << v << " -> ";
    }
    std::cout << "Full Tour" << std::endl;

    //display discarded paths for debug purposes //
    std::cout << "Discarded Paths:" << std::endl;
    for (const auto& pair : discardedPaths) {
        std::cout << "Distance: " << pair.second << " Path: ";
        for (int v : pair.first) {
            std::cout << v << " -> ";
        }
        std::cout << std::endl;
    }

    return minCost;
}




