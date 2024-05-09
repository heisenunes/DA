#include <vector>
#include "data_structures/Graph.h"
#include <limits>

using namespace std;

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
    
    // All vertices are visited and there is a return path to the start //
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

float tsp(Graph<int> *g){
    for(auto v: g->getVertexSet()){
       v->setVisited(false);
    }

    vector<int> currentPath, bestPath;
    float minCost = std::numeric_limits<float>::infinity();

    currentPath.push_back(0);

    tspBacktracking(g,0, currentPath, bestPath, 0, 1, 0.0, minCost);

    cout << "- Minimum distance traveled is " << minCost << std::endl;
    cout << "- Optimal Path - " << endl;
    for (int v : bestPath) {
        cout << v << " -> ";
    }
    cout << endl;

    return minCost;

}
