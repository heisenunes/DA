//
//

#include "RealWorldGraph.h"

RealWorldGraph::RealWorldGraph(int nVertices) {

}

RealWorldGraph::RealWorldGraph() {}

int RealWorldGraph::numEdges() {
    return nEdges;
}

int RealWorldGraph::numVertices() {
    return nVertices;
}

RealWorldGraph::Vertex RealWorldGraph::getVertex(int i) {
    return vertices[i];
}

const vector<RealWorldGraph::Vertex>& RealWorldGraph::getVertices() const {
    return vertices;
}

void RealWorldGraph::setGraphType(int newGraphType) {
    graphType = newGraphType;
}

int RealWorldGraph::getGraphType() {
    return graphType;
}

float RealWorldGraph::findEdgeCost(int startVertex, int endVertex) const {
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

void RealWorldGraph::addEdge(int origin, int destination, float distance) {
    vertices[origin].adj.push_back({destination, distance});
    vertices[destination].adj.push_back({origin, distance});
    nEdges ++;
}

int RealWorldGraph::addVertex(int vertex) {
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

int RealWorldGraph::addVertexLongLat(int vertex, double longi, double lat) {
    int ind = (int)vertices.size();
    for (int i=0; i < nVertices; i++)
        if(vertices[i].index == vertex)
            return i;

    Vertex v = Vertex();
    v.index = vertex;
    v.longitude = longi;
    v.latitude = lat;

    vertices.push_back(v);
    nVertices++;
    return ind;
}




