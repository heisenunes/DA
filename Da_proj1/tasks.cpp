// TASK T2.1
#include "data_structures/Graph.h"
#include "Parser.h"

template <class T>
void testAndVisit(std::queue< Vertex<T>*> &q, Edge<T> *e, Vertex<T> *w, double residual) {
// Check if the vertex 'w' is not visited and there is residual capacity
if (! w->isVisited() && residual > 0) {
// Mark 'w' as visited, set the path through which it was reached, and enqueue it
  w->setVisited(true);
  w->setPath(e);
  q.push(w);
 }
}

// Function to find an augmenting path using Breadth-First Search
template <class T>
bool findAugmentingPath(Graph<T> *g, Vertex<T> *s, Vertex<T> *t) {
// Mark all vertices as not visited
   for(auto v : g->getVertexSet()) {
     v->setVisited(false);
   }
   // Mark the source vertex as visited and enqueue it
   s->setVisited(true);
   std::queue<Vertex<T> *> q;
   q.push(s);
// BFS to find an augmenting path
while( ! q.empty() && ! t->isVisited()) {
   auto v = q.front();
   q.pop();

   // Process outgoing edges
   for(auto e: v->getAdj()) {
     testAndVisit(q, e, e->getDest(), e->getWeight() - e->getFlow());
   }
  // Process incoming edges
 for(auto e: v->getIncoming()) {
   testAndVisit(q, e, e->getOrig(), e->getFlow());
 }
}
// Return true if a path to the target is found, false otherwise
  return t->isVisited();
}

// Function to find the minimum residual capacity along the augmenting path
template <class T>
double findMinResidualAlongPath(Vertex<T> *s, Vertex<T> *t) {
   double f = INF;
   // Traverse the augmenting path to find the minimum residual capacity
   for (auto v = t; v != s; ) {
     auto e = v->getPath();
     if (e->getDest() == v) {
       f = std::min(f, e->getWeight() - e->getFlow());
       v = e->getOrig();
   }
   else {
    f = std::min(f, e->getFlow());
    v = e->getDest();
   }
  }
 // Return the minimum residual capacity
  return f;
}

// Function to augment flow along the augmenting path with the given flow value
template <class T>
void augmentFlowAlongPath(Vertex<T> *s, Vertex<T> *t, double f) {
   // Traverse the augmenting path and update the flow values accordingly
   for (auto v = t; v != s; ) {
     auto e = v->getPath();
     double flow = e->getFlow();
    if (e->getDest() == v) {
      e->setFlow(flow + f);
      v = e->getOrig();
    }
    else {
     e->setFlow(flow - f);
     v = e->getDest();
    } 
   }
}

// Main function implementing the Edmonds-Karp algorithm
template <class T>
double edmondsKarp(Graph<T> *g, std::string source, std::string target) {

 // Find source and target vertices in the graph
  Vertex<T>* s = g->findVertex(source);
  Vertex<T>* t = g->findVertex(target);

  // Validate source and target vertices
  if (s == nullptr)
   throw std::logic_error("Invalid source vertex");

  if(t == nullptr)
   throw std::logic_error("Invalid target vertex");

  if(s == t)
   throw std::logic_error("Invalid input, source equals target vertex");


  double maxFlow = 0.0;

// Initialize flow on all edges to 0
for (auto v : g->getVertexSet()) {
  for (auto e: v->getAdj()) {
    e->setFlow(0);
  }
}
// While there is an augmenting path, augment the flow along the path
while( findAugmentingPath(g, s, t) ) {
   double f = findMinResidualAlongPath(s, t);
   augmentFlowAlongPath(s, t, f);
   maxFlow += f;
 }
 return maxFlow;
}


//TASK T2.2

template <class T>
void t2_2(Graph<T> *g){
  unordered_map<string,City> cities;

  cities = getCities("Project1LargeDataSet/Project1LargeDataSet/Cities.csv");
   
   for(auto v: g->getVertexSet()){
      if((v->getInfo()[0]) == 'C'){
        
        auto it = cities.find(v->getInfo());
        
        double max_flow = edmondsKarp(g,"S", v->getInfo());

        if(it->second.getDemand() > max_flow)
           std::cout << "(City_code: " << v->getInfo()<< ",Deficit: " << it->second.getDemand()-max_flow << ")" << endl;
        
          // std::cout << "(" << v->getInfo()<< "," << max_flow << ")" << endl;
        
      }
         
   }
  
}

//TASK T2.3

/** Functions to Calculate and Display Metrics*/
template <class T>
void printFlowDetails(const Graph<T> *g) {
    std::cout << "Origin || Destination || Flux || Capacity" << std::endl;

    for (auto& vertex : g->getVertexSet()) {
        for (auto& edge : vertex->getAdj()) { 
            //exclude unused edges and super source for readability
            if(edge->getFlow()> 0 && edge->getOrig()->getInfo() != "S" ){
              std::cout << edge->getOrig()->getInfo() << " || " 
                        << edge->getDest()->getInfo() << " || " 
                        << edge->getFlow() << " || "
                        << edge->getWeight() << std::endl;
                        }
                      }
    }
}

struct FlowMetrics {
    double averageDifference = 0;
    double variance = 0;
    double maxDifference = 0;
};

template <class T>
FlowMetrics flowMetrics(const Graph<T>* g) {
    FlowMetrics metrics;
    double totalDifference = 0;
    int count = 0;

    //average difference between Capacity and Flow
    for (auto& vertex : g->getVertexSet()) {
        for (auto& edge : vertex->getAdj()) {
            if (edge->getOrig()->getInfo() != "S") {
                totalDifference += (edge->getWeight() - edge->getFlow());
                ++count;
            }
        }
    }

    if (count > 0) {
        metrics.averageDifference = totalDifference / count;
    }

    //variance of difference between Capacity and Flow
    if (count > 1) {
        for (auto& vertex : g->getVertexSet()) {
            for (auto& edge : vertex->getAdj()) {
                if (edge->getOrig()->getInfo() != "S") {
                    double difference = (edge->getWeight() - edge->getFlow());
                    metrics.variance += pow(difference - metrics.averageDifference, 2);
                }
            }
        }
        metrics.variance /= (count - 1);
    }

    //maximum difference between Capacity and Flow
    for (auto& vertex : g->getVertexSet()) {
        for (auto& edge : vertex->getAdj()) {
            if (edge->getOrig()->getInfo() != "S") {
                double difference = (edge->getWeight() - edge->getFlow());
                if (difference > metrics.maxDifference) {
                    metrics.maxDifference = difference;
                }
            }
        }
    }

    return metrics;
}


void printFlowMetrics(const FlowMetrics& metrics) {
    std::cout << "Average Difference: " << metrics.averageDifference << std::endl;
    std::cout << "Variance: " << metrics.variance << std::endl;
    std::cout << "Maximum Difference: " << metrics.maxDifference << std::endl;
}

bool checkMetricImprovement(const FlowMetrics& beforeMetrics, const FlowMetrics& afterMetrics){

    if (afterMetrics.averageDifference < beforeMetrics.averageDifference ||
        afterMetrics.variance < beforeMetrics.variance ||
        afterMetrics.maxDifference < beforeMetrics.maxDifference) {
        return true; 
        //if any of the metrics have improved, return true
    }
    return false; //no improvement in the metrics  
}
/** **/


/** Path finding functions**/

template <class T>
void Graph<T>::dfs_target(const T& current, const T& dest, std::vector<Edge<T>*>& path, std::vector<std::vector<Edge<T>*> >& paths) {
    Vertex<T>* source = findVertex(current);
    source->setVisited(true);

    if(current == dest) {
        paths.push_back(path);
    }
    else {
        for(auto e : source->getAdj()) {
            Vertex<T>* u = e->getDest();
            if(!u->isVisited() && e->getWeight() - e->getFlow() > 0) {
                path.push_back(e);
                dfs_target(u->getInfo(), dest, path, paths);
                path.pop_back();
            }
        }
    }
}

template <class T>
std::vector<std::vector<Edge<T>*> > Graph<T>::getPaths(const T& source, const T& dest) {
    std::vector<std::vector<Edge<T>*> > paths;
    std::vector<Edge<T>*> path;

    //set all visited status to false
    for(auto v : vertexSet) {
        //std::cout << v->getInfo() << endl;
        v->setVisited(false);
    }
  
    dfs_target(source, dest, path, paths); //find a path between source and destination with dfs

    return paths;
}

/****/

/** Helper functions to print paths**/
template <class T>
void printPath(const std::vector<Edge<T>*>& path) {
    for (size_t i = 0; i < path.size(); ++i) {
        auto edge = path[i];
        std::cout << edge->getOrig()->getInfo() << " -> " 
                  << edge->getDest()->getInfo() 
                  << " | Flow: " << edge->getFlow() 
                  << " | Capacity: " << edge->getWeight();
        if (i < path.size() - 1) std::cout << " -> ";
    }
    std::cout << std::endl;
}

template <class T>
void printAllPaths(const std::vector<std::vector<Edge<T>*> >& paths) {
    if (paths.empty()) {
        std::cout << "No paths found." << std::endl;
        return;
    }

    std::cout << "Found " << paths.size() << " paths:" << std::endl;
    for (const auto& path : paths) {
       printPath(path);
    }
}

/** **/

/** Balance Flow **/
//sort edges based on their capacity to flow ratio or by flow if equal
template <class T>
class EdgeComparator {
public:
    bool operator()(const Edge<T>* a, const Edge<T>* b) const {
        double differenceA = (a->getWeight() - a->getFlow()) / a->getWeight();
        double differenceB = (b->getWeight() - b->getFlow()) / b->getWeight();
        return differenceA == differenceB ? a->getFlow() > b->getFlow() : differenceA < differenceB;
    }
};

template <class T>
void rebalanceFlow(Graph<T> *g){

  Graph<T> gOriginal = *g;

  FlowMetrics beforeMetrics = flowMetrics(g);
  FlowMetrics afterMetrics = flowMetrics (g);


  std::vector<Edge<T>*> pipes;

      for (auto v : g->getVertexSet()) {
          for (auto e : v->getAdj()) {
              pipes.push_back(e);
          }
      }

  bool continueBalancing = true;
  int improvementMade = 0;
  int count = 0;
  
  while (continueBalancing) {
    
    std::sort(pipes.begin(), pipes.end(), EdgeComparator<T>());

    for (Edge<T>* edge : pipes) {
      if (edge->getFlow() == 0) {break;}

      auto paths = g->getPaths(edge->getOrig()->getInfo(), edge->getDest()->getInfo());
      if (paths.empty()) {
        //std::cout << "NO PATH FOUND!" << endl; 
        continue;}

        double maxDiff = -1;
        std::vector<Edge<T>*> bestPath;

        for (auto& p : paths) {
          double minDiff = std::numeric_limits<double>::max();
          for (Edge<T>* e : p) {
            double spareCapacity = e->getWeight() - e->getFlow();
            minDiff = std::min(minDiff, spareCapacity);
          }
            if (minDiff > maxDiff) {
              maxDiff = minDiff;
              bestPath = p;
            }
        }

        double flowToRedirect = std::min(maxDiff, edge->getFlow());
        edge->setFlow(edge->getFlow() - flowToRedirect);
        for (Edge<T>* e : bestPath) {
          e->setFlow(e->getFlow() + flowToRedirect);
        }
    }

    //update metrics
    afterMetrics = flowMetrics(g);

    //check if we need/can iterate
    continueBalancing = checkMetricImprovement(beforeMetrics, afterMetrics) && count < pipes.size();
    if (continueBalancing){improvementMade ++;};

    //printFlowMetrics(beforeMetrics);
    beforeMetrics = afterMetrics;
    count++;

  }


   if (improvementMade > 0){
      cout << "Network balance was Improved " << endl;
      printFlowMetrics(afterMetrics);
    }

    if (improvementMade == 0){
      cout << "No improvement to the network is possible" << endl;
    }
  //printFlowDetails(&gOriginal);
  //beforeMetrics = flowMetrics(&gOriginal);
  //printFlowMetrics(beforeMetrics);
  
  //printFlowDetails(g);
  //printFlowMetrics(afterMetrics);
  //cout << "Count: " << count << endl;
}

/** **/

template <class T>
double t2_3(Graph<T> *g, std::string source, std::string target) {

 // Find source and target vertices in the graph
  Vertex<T>* s = g->findVertex(source);
  Vertex<T>* t = g->findVertex(target);

  // Validate source and target vertices
  if (s == nullptr)
   throw std::logic_error("Invalid source vertex");

  if(t == nullptr)
   throw std::logic_error("Invalid target vertex");

  if(s == t)
   throw std::logic_error("Invalid input, source equals target vertex");


  double maxFlow = 0.0;

  // Initialize flow on all edges to 0
  for (auto v : g->getVertexSet()) {
    for (auto e: v->getAdj()) {
      e->setFlow(0);
    }
  }
  // While there is an augmenting path, augment the flow along the path
  while( findAugmentingPath(g, s, t) ) {
    double f = findMinResidualAlongPath(s, t);
    augmentFlowAlongPath(s, t, f);
    maxFlow += f;
  }

  cout << "BEFORE BALANCING:" << endl;

  FlowMetrics beforeMetrics = flowMetrics(g);
  //printFlowDetails(g);
  printFlowMetrics(beforeMetrics);
  
  //auto paths = g->getPaths("R_7", "C_20");
  //auto paths = g->getPaths("PS_51", "C_10");
  //auto paths = g->getPaths("PS_51", "C_16");
  //auto paths = g->getPaths("PS_70", "C_10");
  //printAllPaths(paths);
  cout << "AFTER BALANCING:" << endl;
  rebalanceFlow(g);

  return maxFlow;
}
