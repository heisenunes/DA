#include <vector>
#include "data_structures/Graph.h"
#include <limits>
#include <unordered_set>
#include <unordered_map>
#include <utility>

using namespace std;

double totalDistance;


struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ h2; 
    }
};

using EdgeWeightMap = std::unordered_map<std::pair<int, int>, double, pair_hash>;

EdgeWeightMap precomputeEdgeWeights(Graph<int>* g) {
    EdgeWeightMap edgeWeights;
    for (const auto& vertex : g->getVertexSet()) {
        int u = vertex->getInfo();
        for (const auto& edge : vertex->getAdj()) {
            int v = edge->getDest()->getInfo();
            double weight = edge->getWeight();
            edgeWeights[{u, v}] = weight;
            edgeWeights[{v, u}] = weight;
        }
    }
    return edgeWeights;
}

/**
 * @brief function that solves the Traveling Salesman Problem (TSP) using backtracking approach
 * 
 * @param g Pointer to the graph.
 * @param currVertex Vertex being visited.
 * @param currentPath Path being explored
 * @param bestPath Mininum cost path found until now
 * @param startVertex The starting vertex
 * @param numVisited The number of vertices visited so far.
 * @param currentCost The cost of the path being explored
 * @param minCost The cost of the best path found until now
 */

void tspBacktracking(Graph<int> *g, int currVertex,vector<int>& currentPath, vector<int>& bestPath,
                               int startVertex, int numVisited, float currentCost, float& minCost) {

    Vertex<int>* origin = g->findVertex(currVertex);
    Vertex<int>* dest = g->findVertex(startVertex);   

    double edge_cost;                            
    
   for(auto e: origin->getAdj()){
       if(e->getDest()== dest) 
        edge_cost = e->getWeight();
   }    

  // cout << edge_cost << endl;
    
    // all vertices are visited and there is a return path to the start //
    if (numVisited == g->getNumVertex() && edge_cost != std::numeric_limits<float>::infinity()) {
        float totalCost = currentCost + edge_cost;
        if (totalCost < minCost) {
            minCost = totalCost;
            bestPath = currentPath;
            bestPath.push_back(startVertex);  // return to the starting point 0 //
        }
        return;
    }

    for (const auto& edge : origin->getAdj()) { // travel to other vertices that were not visited yet //
        if (!edge->getDest()->isVisited()) {
            edge->getDest()->setVisited(true);
            currentPath.push_back(edge->getDest()->getInfo());

            tspBacktracking(g,edge->getDest()->getInfo(), currentPath, bestPath, startVertex, numVisited + 1,
                            currentCost + edge->getWeight(), minCost);

            edge->getDest()->setVisited(false); // backtrack//
            currentPath.pop_back();
        }
    }
}

/**
 * @brief Function to initialize and solve the Traveling Salesman Problem (TSP)
 * 
 * @param g Pointer to the graph
 * @return The cost of the best path found 
 */

float tsp(Graph<int> *g){
    for(auto v: g->getVertexSet()){
       v->setVisited(false);
    }

    vector<int> currentPath, bestPath;
    float minCost = std::numeric_limits<float>::infinity();
    
    
    for(auto v: g->getVertexSet()){
        if(v->getInfo() == 0)
         v->setVisited(true);
    }

    currentPath.push_back(0);

    tspBacktracking(g,0, currentPath, bestPath, 0, 1, 0.0, minCost);

    cout << "- Tour:  " << endl;
    for (int v : bestPath) {
        cout << v << " ";
    }
    cout << endl;

    return minCost;

}


template <class T>
Graph<T> primMST(Graph<T> * g) {
    
    Graph<T> mstGraph; // Create a new graph for the MST

    // Check if the graph is empty
    if (g->getVertexSet().empty()) {
        return mstGraph; // Return an empty graph if the input graph is empty
    }

    // Initialize the vertices in the graph
    for(auto v : g->getVertexSet()) {
        v->setDist(INF); // Set distance to infinity
        v->setPath(nullptr); // Set path to null
        v->setVisited(false); // Mark as not visited
    }

    // Priority queue to store vertices based on their distances
    MutablePriorityQueue<Vertex<T>> q;

    // Set to keep track of added vertices
    std::unordered_set<Vertex<T>*> addedVertices;

    // Select the first vertex as the starting point
    Vertex<T>* s = g->getVertexSet().front();
    s->setDist(0); // Set distance of the starting vertex to 0
    q.insert(s);

    // Main loop for the Prim's algorithm
    while( ! q.empty() ) {
        // Extract the vertex with the minimum distance from the priority queue
        auto v = q.extractMin();
        v->setVisited(true); // Mark the vertex as visited

        // Add the vertex to the MST graph if not already added
        if (addedVertices.find(v) == addedVertices.end()) {
            mstGraph.addVertex(v->getInfo());
            addedVertices.insert(v);
        }

        // Iterate through the adjacent edges of the current vertex
        for(auto &e : v->getAdj()) {
            Vertex<T>* w = e->getDest(); // Get the destination vertex of the edge

            // Check if the destination vertex is not visited
            if (!w->isVisited()) {
                auto oldDist = w->getDist(); // Get the current distance of the destination vertex

                // Check if the weight of the edge is less than the current distance of the destination vertex
                if(e->getWeight() < oldDist) {
                    w->setDist(e->getWeight()); // Update the distance of the destination vertex
                    w->setPath(e); // Update the path to the current edge

                    // If the destination vertex had infinite distance, insert it into the priority queue
                    if (oldDist == INF) {
                        q.insert(w);
                    }
                    // If the destination vertex had finite distance, decrease its key in the priority queue
                    else {
                        q.decreaseKey(w);
                    }
                }
            }
        }
    }

    // Add edges to the MST graph
    for(auto v : addedVertices) {
        if (v->getPath() != nullptr) {
            mstGraph.addBidirectionalEdge(v->getInfo(), v->getPath()->getDest()->getInfo(), v->getPath()->getWeight());
        }
    }

    return mstGraph;
}


template <class T>
std::vector<T> pre_order(Graph<T> * g, double &sum_Distances){

    std::vector<T> res;
    for (auto v : g->getVertexSet())
        v->setVisited(false);
    for (auto v : g->getVertexSet())
        if (!v->isVisited())
            pre_order_Visit(v, res,sum_Distances);

    return res;

}


template <class T>
void pre_order_Visit(Vertex<T> *v, std::vector<T> & res, double &sum_Distances){
    v->setVisited(true);
    res.push_back(v->getInfo());
   
    for (auto & e : v->getAdj()) {
        auto w = e->getDest();

        if (!w->isVisited()) {
            pre_order_Visit(w, res,sum_Distances);
        }
        
        sum_Distances += e->getWeight();
    }
  
}


vector<int> nearestNeighbor(Graph<int> *g) {
    int startCity = 0;
    vector<int> path;
    unordered_set<int> visited; 

    totalDistance = 0.0;

    int numCities = g->getNumVertex();

    int currCity = startCity;
    path.push_back(currCity);
    visited.insert(currCity);

    while(visited.size() < numCities) {
        int nearestCity = -1;
        double minDistance = numeric_limits<double>::max();

        Vertex<int>* curr = g->findVertex(currCity); 

        for(auto e : curr->getAdj()) {
            if(visited.find(e->getDest()->getInfo()) == visited.end()) {
                if(e->getWeight() < minDistance) {
                    minDistance = e->getWeight();
                    nearestCity = e->getDest()->getInfo();
                }
            }
        }

        if(nearestCity != -1) {
            path.push_back(nearestCity);
            visited.insert(nearestCity);
            currCity = nearestCity;
        }
        else {
            break;
        }
        totalDistance += minDistance;
    }
    path.push_back(startCity);

    Vertex<int>* startVert = g->findVertex(0);

    for (const auto& e : startVert->getAdj()) {
        if (e->getDest()->getInfo() == currCity) {
            totalDistance += e->getWeight();
            break;
        }
    }

    std::cout << endl;

    std::cout << "Distance: "<< totalDistance << endl;
    
    return path;
}

vector<int> twoOpt(vector<int> path, Graph<int>*g) {

    EdgeWeightMap edgeWeights = precomputeEdgeWeights(g);

    int numCities = path.size();

    double optimizedDistance;

    bool improved = true;

    while (improved) {
        improved = false;

        for (int i = 2; i <= numCities - 3; i++) {
            for (int j = i + 1; j <= numCities - 2; j++) {
                int num_1 = path.at(i-1);
                int num_2 = path.at(i);
                int num_3 = path.at(j);
                int num_4 = path.at(j+1);
                
                double edge_1 = edgeWeights[{num_1, num_2}];
                double edge_2 = edgeWeights[{num_4, num_2}];
                double edge_3 = edgeWeights[{num_1, num_3}];
                double edge_4 = edgeWeights[{num_4, num_3}];

                double newDistance = totalDistance - edge_1 - edge_4+ edge_3+ edge_2;
                if (newDistance < totalDistance) {
                    reverse(path.begin() + i, path.begin() + j + 1);
                    totalDistance = newDistance;
                    improved = true;
                } 
                optimizedDistance = totalDistance;
            }
        }
    }
    std::cout << "Optimized Distance: " << optimizedDistance << endl;
    return path;
}
