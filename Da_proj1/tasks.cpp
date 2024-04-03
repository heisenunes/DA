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



