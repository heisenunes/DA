// TASK T2.1
#include "data_structures/Graph.h"
#include "Parser.h"

template <class T>
void testAndVisit(std::queue< Vertex<T>*> &q, Edge<T> *e, Vertex<T> *w, double residual, unordered_map<Vertex<T>*, double>* maxDelivery, unordered_map<Vertex<T>*, double>* Demand) {
  unordered_map<string, Reservoir> reservoirs = getReservoirs("Project1DataSetSmall/Project1DataSetSmall/Reservoirs_Madeira.csv");
  unordered_map<string, City> cities = getCities("Project1DataSetSmall/Project1DataSetSmall/Cities_Madeira.csv");
  if(w->getInfo()[0] == 'C' && (*Demand)[w] == cities.find(w->getInfo())->second.getDemand()) {
  }

  else if(w->getInfo()[0] == 'R' && (*maxDelivery)[w] == reservoirs.find(w->getInfo())->second.get_Maximum_Delivery()) {
  }
  
  // Check if the vertex 'w' is not visited and there is residual capacity
  else if (! w->isVisited() && residual > 0) {
    // Mark 'w' as visited, set the path through which it was reached, and enqueue it
    w->setVisited(true);
    w->setPath(e);
    q.push(w);
  }
}

// Function to find an augmenting path using Breadth-First Search
template <class T>
bool findAugmentingPath(Graph<T> *g, Vertex<T> *s, Vertex<T> *t, unordered_map<Vertex<T>*, double>* maxDelivery, unordered_map<Vertex<T>*, double>* Demand) {
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
      testAndVisit(q, e, e->getDest(), e->getWeight() - e->getFlow(), &(*maxDelivery), &(*Demand));
    }
    // Process incoming edges
    for(auto e: v->getIncoming()) {
      testAndVisit(q, e, e->getOrig(), e->getFlow(), &(*maxDelivery), &(*Demand));
    }
  }
  // Return true if a path to the target is found, false otherwise
  return t->isVisited();
}

// Function to find the minimum residual capacity along the augmenting path
template <class T>
double findMinResidualAlongPath(Vertex<T> *s, Vertex<T> *t, unordered_map<Vertex<T>*, double>* maxDelivery, unordered_map<Vertex<T>*, double>* Demand) {
  double f = INF;
  unordered_map<string, Reservoir> reservoirs = getReservoirs("Project1DataSetSmall/Project1DataSetSmall/Reservoirs_Madeira.csv");
  unordered_map<string, City> cities = getCities("Project1DataSetSmall/Project1DataSetSmall/Cities_Madeira.csv");
  Vertex<T>* city;
  Vertex<T>* reservoir;
  // Traverse the augmenting path to find the minimum residual capacity
  for (auto v = t; v != s; ) {
    auto e = v->getPath();
    if(v == t) {
      city = e->getOrig();
    }
    if (e->getDest() == v) {
      f = std::min(f, e->getWeight() - e->getFlow());
      v = e->getOrig();
      if(v == s) {
        reservoir = e->getDest();
      }
    }
    else {
      cout << "print" << endl;
      f = std::min(f, e->getFlow());
      v = e->getDest();
      if(v == s) {
        reservoir = e->getOrig();
      }
    }
    
  }
  auto it = cities.find(city->getInfo());
  if((*Demand)[city] + f > it->second.getDemand()) {
    f = it->second.getDemand()- (*Demand)[city];
  }

  auto j = reservoirs.find(reservoir->getInfo());
  if((*maxDelivery)[reservoir] + f > j->second.get_Maximum_Delivery()) {
    f = j->second.get_Maximum_Delivery() - (*maxDelivery)[reservoir];
  }

  if(city->getInfo()[2] == '3') {
    cout << "Flow: " << f << endl;
  } 

  (*Demand)[city] += f;
  (*maxDelivery)[reservoir] += f;

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
double edmondsKarp(Graph<T> *g, std::string source, std::string city) {

  // Find source and target vertices in the graph
  Vertex<T>* s = g->findVertex(source);
  Vertex<T>* t = g->findVertex("T");


  Vertex<T>* c = g->findVertex(city);

  std::unordered_map<Vertex<T>*, double> maxDelivery;
  std::unordered_map<Vertex<T>*, double> Demand;

  for(auto v : g->getVertexSet()) {
    if(v->getInfo()[0] == 'C') {
      Demand[v] = 0.0;
    }
    else if(v->getInfo()[0] == 'R') {
      maxDelivery[v] = 0.0;
    }
  }

  // Validate source and target vertices
  if (s == nullptr)
   throw std::logic_error("Invalid source vertex");

  if(t == nullptr)
   throw std::logic_error("Invalid target vertex");

  if(s == t)
   throw std::logic_error("Invalid input, source equals target vertex");


  // Initialize flow on all edges to 0
  for (auto v : g->getVertexSet()) {
    for (auto e: v->getAdj()) {
      e->setFlow(0);
    }
  }
  while(findAugmentingPath(g, s, t, &maxDelivery, &Demand)) {
    double f = findMinResidualAlongPath(s, t, &maxDelivery, &Demand);
    augmentFlowAlongPath(s, t, f);
  }
  // While there is an augmenting path, augment the flow along the path

  double maxFlow = Demand[c];

  double totalFlow = 0.0;

  for(auto v: g->getVertexSet()) {
    if(v->getInfo()[0] == 'R') {
      cout << v->getInfo() << ":" << maxDelivery[v] << endl;
    }
  }

  for(auto v: g->getVertexSet()) {
    if(v->getInfo()[0] == 'C') {
      totalFlow += Demand[v];
    }
  }

  cout << "Total flow: " << totalFlow << endl;

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

/**
 * @brief Print detailed flow information for each edge in the graph.
 * 
 * @tparam T The datatype used for the vertices in the graph.
 * @param g A pointer to the graph of which flow details will be printed.
 */
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

/**
 * @struct FlowMetrics
 * @brief A structure to hold metrics related to flow in the network.
 */
struct FlowMetrics {
    double averageDifference = 0;
    double variance = 0;
    double maxDifference = 0;
};

/**
 * @brief Calculates flow metrics for the graph.
 * 
 * @tparam T The datatype used for the vertices in the graph.
 * @param g A constant pointer to the graph for which metrics are calculated.
 * @return FlowMetrics A struct containing the calculated metrics.
 */
template <class T>
FlowMetrics flowMetrics(const Graph<T>* g) {
    FlowMetrics metrics;
    double totalDifference = 0;
    int count = 0;

    //average difference between Capacity and Flow
    for (auto& vertex : g->getVertexSet()) {
        for (auto& edge : vertex->getAdj()) {
            if (edge->getOrig()->getInfo() != "S" && edge->getDest()->getInfo() != "T") {
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
                if (edge->getOrig()->getInfo() != "S" && edge->getDest()->getInfo() != "T") {
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
            if (edge->getOrig()->getInfo() != "S" && edge->getDest()->getInfo() != "T") {
                double difference = (edge->getWeight() - edge->getFlow());
                if (difference > metrics.maxDifference) {
                    metrics.maxDifference = difference;
                }
            }
        }
    }

    return metrics;
}

/**
 * @brief Print calculated flow metrics.
 * 
 * @param metrics A reference to the FlowMetrics struct containing the metrics to print.
 */
void printFlowMetrics(const FlowMetrics& metrics) {
    std::cout << "Average Difference: " << metrics.averageDifference << std::endl;
    std::cout << "Variance: " << metrics.variance << std::endl;
    std::cout << "Maximum Difference: " << metrics.maxDifference << std::endl;
}

/**
 * @brief Check for any improvement in flow metrics.
 * 
 * @param beforeMetrics The flow metrics before the rebalance algorithm.
 * @param afterMetrics The flow metrics after the rebalance algorithm.
 * @return true If any of the metrics have improved after the rebalance algorithm.
 * @return false If no improvement is found in the metrics after the rebalance algorithm.
 */
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

/**
 * @brief Performs a depth-first search (DFS) to find all paths from a source vertex to a destination vertex.
 * 
 * This recursive function explores all possible paths from 'current' to 'dest'. It considers only edges with available capacity.
 * The paths are stored in a vector of vectors, where each vector represents a path consisting of edges.
 * 
 * @tparam T The data type used for vertex identifiers.
 * @param current The current vertex identifier in the DFS.
 * @param dest The destination vertex to find a path for.
 * @param path A reference to the current path being explored.
 * @param paths A reference to the vector of vectors, where all found paths will be stored.
 */
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

/**
 * @brief Finds all paths from a source to a destination vertex using Depth First Search (DFS).
 * 
 * This function initializes the path-finding process by resetting the visited status of all vertices.
 * It then calls the recursive 'dfs_target' to explore all paths between source and dest.
 * 
 * @tparam T The data type used for vertex identifiers.
 * @param source The identifier of the source vertex where paths begin.
 * @param dest The identifier of the destination vertex where paths end.
 * @return A vector of vectors, with each inner vector representing a path from source to dest.
 */
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

/** Helper functions to print paths **/

/**
 * @brief Prints a single path represented by a vector of edges.
 * 
 * This function iterates through the given path vector and prints information
 * about each edge, including origin, destination, flow, and capacity.
 * 
 * @tparam T The data type used for the vertices in the graph.
 * @param path A constant reference to a vector of edges that represents a path.
 */
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

/**
 * @brief Prints all the paths contained in a vector of path vectors.
 * 
 * If there are no paths found, it prints a "No paths found." message
 * If there are paths, it prints the number of paths and iterates through the list of paths,
 * calling the printPath function for each one in order to print them.
 * 
 * @tparam T The data type used for the vertices in the graph.
 * @param paths A constant reference to a vector containing vectors of edges, 
 *              where each inner vector represents a different path.
 */
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
/**
 * @class EdgeComparator
 * @brief Comparator class to sort edges based on their capacity to flow ratio or by flow if the ratio is equal.
 *
 */
template <class T>
class EdgeComparator {
public:
    /**
     * @brief Compares two edges for sorting purposes.
     *
     * @param a Pointer to the first edge.
     * @param b Pointer to the second edge.
     * @return true If the first edge has a lower capacity to flow ratio than the second edge or,
     *         if the ratios are equal, true if the first edge has a greater flow.
     * @return false Otherwise.
     */
    bool operator()(const Edge<T>* a, const Edge<T>* b) const {
        double differenceA = (a->getWeight() - a->getFlow()) / a->getWeight();
        double differenceB = (b->getWeight() - b->getFlow()) / b->getWeight();
        return differenceA == differenceB ? a->getFlow() > b->getFlow() : differenceA < differenceB;
    }
};

/**
 * @brief Attempts to rebalance the flow in a graph to improve network efficiency.
 *
 * This function iterates through all edges in the graph, trying to find alternative paths
 * for edges with flow, to balance the network flow based on a specified metric. The process
 * continues until the number of iterations is equal to the number of edges or no further improvements can be made.
 *
 * @tparam T The data type used for the vertices in the graph.
 * @param g A pointer to the graph to rebalance.
 */
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

/**
 * @brief Function to execute Task 2.3. It takes the maximum flow as calculated in task 2.1, and then tries to rebalance the flow in the graph.
 *
 * This function finds the maximum flow using the Edmonds-Karp algorithm and after that it calls the rebalanceFlow function to improve
 * the balance of the flow across the network. It outputs
 * the flow metrics before and after rebalancing as well as printing the maximum flow of the graph.
 *
 * @tparam T The data type used for the vertices in the graph.
 * @param g A pointer to the graph where the flow is to be calculated and rebalanced.
 * @param source A string representing the identifier of the source vertex.
 * @param target A string representing the identifier of the target vertex.
 * @return The value of the maximum flow from source to target before rebalancing.
 */
template <class T>
double t2_3(Graph<T> *g, std::string source, std::string city) {

 // Find source and target vertices in the graph
  Vertex<T>* s = g->findVertex(source);
  Vertex<T>* t = g->findVertex("T");


  Vertex<T>* c = g->findVertex(city);

  std::unordered_map<Vertex<T>*, double> maxDelivery;
  std::unordered_map<Vertex<T>*, double> Demand;

  for(auto v : g->getVertexSet()) {
    if(v->getInfo()[0] == 'C') {
      Demand[v] = 0.0;
    }
    else if(v->getInfo()[0] == 'R') {
      maxDelivery[v] = 0.0;
    }
  }

  // Validate source and target vertices
  if (s == nullptr)
   throw std::logic_error("Invalid source vertex");

  if(t == nullptr)
   throw std::logic_error("Invalid target vertex");

  if(s == t)
   throw std::logic_error("Invalid input, source equals target vertex");


  // Initialize flow on all edges to 0
  for (auto v : g->getVertexSet()) {
    for (auto e: v->getAdj()) {
      e->setFlow(0);
    }
  }
  while(findAugmentingPath(g, s, t, &maxDelivery, &Demand)) {
    double f = findMinResidualAlongPath(s, t, &maxDelivery, &Demand);
    augmentFlowAlongPath(s, t, f);
  }
  // While there is an augmenting path, augment the flow along the path

  double maxFlow = Demand[c];

  double totalFlow = 0.0;

  for(auto v: g->getVertexSet()) {
    if(v->getInfo()[0] == 'R') {
      cout << v->getInfo() << ":" << maxDelivery[v] << endl;
    }
  }

  for(auto v: g->getVertexSet()) {
    if(v->getInfo()[0] == 'C') {
      totalFlow += Demand[v];
    }
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


template<class T>
void t3_1(Graph<T> *g, std::string reservoir) {
  unordered_map<string,City> cities;
  std::vector<Edge<T>*> savedEdges;
  cities = getCities("Project1LargeDataSet/Project1LargeDataSet/Cities.csv");


  unordered_map<string, double> deficits_before;
  for (auto v : g->getVertexSet()) {
    if ((v->getInfo()[0]) == 'C') {
      auto it = cities.find(v->getInfo());
      double max_flow = edmondsKarp(g, "S", v->getInfo());
      double demand = it->second.getDemand();
      deficits_before[v->getInfo()] = std::max(0.0, demand - max_flow);
    }
  }

  for (auto v : g->getVertexSet()) {
      if (v->getInfo() == reservoir) {
          for (auto edge : v->getAdj()) {
            savedEdges.push_back(edge);
          }
          break;
      }
  }
  unordered_map<string, double> savedValues;
   
  for (auto edge : savedEdges) {
    savedValues[edge->getDest()->getInfo()] =edge->getWeight();
  }

  g->removeEdge("S", reservoir);
  g->removeVertex(reservoir);
  cities = getCities("Project1LargeDataSet/Project1LargeDataSet/Cities.csv");

  for(auto v: g->getVertexSet()){
    if((v->getInfo()[0]) == 'C'){
        
      auto it = cities.find(v->getInfo());
        
      double max_flow = edmondsKarp(g,"S", v->getInfo());
      double demand = it->second.getDemand();
      double deficit_after = std::max(0.0, demand - max_flow);
      double deficit_before = deficits_before[v->getInfo()];

      if(deficit_after > 0 && deficit_before <= 0) {
        std::cout << "(City_code: " << v->getInfo()<< ",Deficit: " << it->second.getDemand()-max_flow << ")" << " <- Affected" << endl;  
      }
      else if(it->second.getDemand() > max_flow)
        std::cout << "(City_code: " << v->getInfo()<< ",Deficit: " << it->second.getDemand()-max_flow << ")" << endl;
      
    }
  }

  g->addVertex(reservoir);
  g->addEdge("S", reservoir, std::numeric_limits<int>::max());
  for (const auto& pair : savedValues) {
    g->addEdge(reservoir, pair.first, pair.second);
  }
}

template<class T>
void t3_2(Graph<T> *g, std::string pump_station){

  unordered_map<string,City> cities;
  cities = getCities("Project1LargeDataSet/Project1LargeDataSet/Cities.csv");

  std::vector<Edge<T>*> savedIngoingEdges;
  std::vector<Edge<T>*> savedOutgoingEdges;

  for (auto v : g->getVertexSet()) {
      if (v->getInfo() == pump_station) {
          for (auto edge : v->getAdj()) {
            savedOutgoingEdges.push_back(edge);
          }
          break;
      }
  }

  unordered_map<string, double> savedOutgoing;
   
  for (auto edge : savedOutgoingEdges) {
    savedOutgoing[edge->getDest()->getInfo()] =edge->getWeight();
  }

  auto v = g->findVertex(pump_station);

  savedIngoingEdges = v->getIncoming();
  
  unordered_map<string, double> savedIngoing;

  for (auto edge : savedIngoingEdges) {
    savedIngoing[edge->getOrig()->getInfo()] =edge->getWeight();
  }

  unordered_map<string, double> deficits_before;
  for (auto v : g->getVertexSet()) {
    if ((v->getInfo()[0]) == 'C') {
      auto it = cities.find(v->getInfo());
      double max_flow = edmondsKarp(g, "S", v->getInfo());
      double demand = it->second.getDemand();
      deficits_before[v->getInfo()] = std::max(0.0, demand - max_flow);
    }
  }
  

  double max_flow_total_before = edmondsKarp(g,"S", "T");

  
  g->removeVertex(pump_station);

  double max_flow_total_after = edmondsKarp(g,"S","T");

    cout << "max_before: " << max_flow_total_before << endl;
    cout << "max_after:" <<max_flow_total_after << endl;

    if(max_flow_total_before == max_flow_total_after){
      cout << "Pumping station"<<"("<< pump_station << ") "<< "can be removed without affeting the delivery capacity of all cities." << endl;
    }
    else{

      for(auto v: g->getVertexSet()){
       if((v->getInfo()[0]) == 'C'){
        
        auto it = cities.find(v->getInfo());
        
        double max_flow = edmondsKarp(g,"S", v->getInfo());
        double demand = it->second.getDemand();
        double deficit_after = std::max(0.0, demand - max_flow);
        double deficit_before = deficits_before[v->getInfo()];

         if(deficit_after > 0 && deficit_before <= 0) {
          std::cout << "(City_code: " << v->getInfo()<< ",Deficit: " << it->second.getDemand()-max_flow << ")" << " <- Affected" << endl;  
         }
         else if(it->second.getDemand() > max_flow)
          std::cout << "(City_code: " << v->getInfo()<< ",Deficit: " << it->second.getDemand()-max_flow << ")" << endl;
      
         }
       }
    }
   
  g->addVertex(pump_station);

 //Re-add ingoing edges 
  for (const auto& pair : savedIngoing) {
    g->addEdge(pair.first, pump_station, pair.second);
  }
//Re-add outgoing edges
  for (const auto& pair : savedOutgoing) {
    g->addEdge(pump_station, pair.first, pair.second);
  }

}

/*template<class T>
void t3_3_part1(Graph<T> *g, std::string city){
 
  unordered_map<string,City> cities;
  cities = getCities("Project1LargeDataSet/Project1LargeDataSet/Cities.csv");


   // nao contar com as arestas que tem como destino uma c_..(cidade) correr o algoritmo
}
*/

/*
template <class T>
std::vector<T> cityDfs(const T &source, Graph<T> *g) {
  std<vector<int> res;

  auto s = g->findVertex(source);
  if (s == nullptr) {
    return res;
  } 

  for(auto v : g->getVertexSet) {
    v->setVisited(false);
  }

  cityDfsVisit(s, res);

  return res;
}

template <class T>
void cityDfsVisit(Vertex<T> *v, std::vector<T> &res) {
  if(v->getInfo()[0] == 'C') {
    v->setVisited(true);
    res.pushback(v->getInfo());
  }
  else {
    for(auto &e : v ->getAdj()) {
      auto w = e->getDest();

      if(!w->isVisited()) {
        cityDfsVisit(w, res);
      }
    }
  }
}

*/

