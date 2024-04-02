#include <iostream>
#include <vector>
#include "City.h"
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "Station.h"
#include "Reservoir.h"
#include "data_structures/Graph.h"

using namespace std;

const int INFINITY_INT = std::numeric_limits<int>::max(); 

unordered_map <string,City> getCities(const string &filename){

   unordered_map<string,City> CityMap;

    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return CityMap; // Return empty vector
    }

    string line;
    getline(file, line); // Skip header line

     while (getline(file, line)) {
        stringstream ss(line);

        string city_name, id, code, demand, population;
        getline(ss, city_name, ',');
        getline(ss, id, ',');
        getline(ss, code, ',');
        getline(ss, demand, ',');
        getline(ss, population, ',');

        // Convert strings to appropriate types
        int id_num = stoi(id);
        double demand_num = stod(demand);
        int population_num = stoi(population);

       City aCity(city_name, id_num, code, demand_num, population_num);
        CityMap[code] = aCity;
        
    }

    file.close();
    return CityMap;
}

unordered_map<string,Station> getStations(const string &filename){

    unordered_map<string,Station> StationMap;

    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return StationMap; // Return empty vector
    }

    string line;
    getline(file, line); // Skip header line

    while (getline(file, line)) {
        stringstream ss(line);

        string id, code;
        getline(ss, id, ',');
        getline(ss, code, ',');

        code.erase(0, code.find_first_not_of(" \t\r\n"));
    code.erase(code.find_last_not_of(" \t\r\n") + 1);

    
        // Convert strings to appropriate types
        int id_num = stoi(id);
        
       Station station(id_num, code);

        StationMap[code] = station;
        
    }

    file.close();
    return StationMap;

}

unordered_map<string,Reservoir> getReservoirs(const string &filename){

    unordered_map<string,Reservoir> ReservoirMap;

    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return ReservoirMap; // Return empty vector
    }

    string line;
    getline(file, line); // Skip header line

    while (getline(file, line)) {
        stringstream ss(line);

        string reservoir_name, municipality,id,code,maximum_delivery;
        getline(ss, reservoir_name, ',');
        getline(ss, municipality, ',');
        getline(ss, id, ',');
        getline(ss, code, ',');
        getline(ss, maximum_delivery, ',');

        
    
        // Convert strings to appropriate types
        int id_num = stoi(id);
        int maximum_delivery_num = stoi(maximum_delivery);
        
       Reservoir reservoir(reservoir_name, municipality,id_num,code,maximum_delivery_num);

        ReservoirMap[code] = reservoir;
        
    }

    file.close();
    return ReservoirMap;

}

Graph<string> construct_graph(const string &filename){
    Graph<string> g;
    //int edges = 0;
   
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return g; // Return empty vector
    }
  
  //Add SUPER SOURCE
   string Super_Source = "S";
// Add SUPER_SINK
   string Super_Target = "T";

   g.addVertex(Super_Source);
   g.addVertex(Super_Target);

    string line;
    getline(file, line); // Skip header line

     while (getline(file, line)) {
        stringstream ss(line);

        string service_point_a, service_point_b,capacity,direction;
        getline(ss, service_point_a, ',');
        getline(ss, service_point_b, ',');
        getline(ss, capacity, ',');
        getline(ss, direction, ',');

        if(g.findVertex(service_point_a) == nullptr){
             g.addVertex(service_point_a);

             if (service_point_a[0] == 'R')
             {
                 g.addEdge(Super_Source, service_point_a, INFINITY_INT);
             }
             
        }

        if(g.findVertex(service_point_b) == nullptr){
             g.addVertex(service_point_b);

             if (service_point_b[0] == 'C')
             {
                g.addEdge(service_point_b, Super_Target, INFINITY_INT);
             }
             
        }
        
       
        //cout << "EDGE:" << endl;
        //cout << service_point_a << endl;
        //cout << service_point_b << endl;
        
    
        // Convert strings to appropriate types
        double capacity_num = stod(capacity);
        int direction_num = stoi(direction);
        
        
        //cout << capacity_num << endl;
        //cout << direction_num << endl;

        if(direction_num == 0){
            g.addBidirectionalEdge(service_point_a,service_point_b,capacity_num);
            //cout << "BIDIRECTIONAL: ";
            //cout << "(" << service_point_a << "," << service_point_b << ")" << "with capacity: " << capacity_num << endl;
        }

        else{
            g.addEdge(service_point_a,service_point_b,capacity_num);
            //cout << "NORMAL: ";
            //cout << "(" << service_point_a << "," << service_point_b << ")" << "with capacity: " << capacity_num << endl;
        }

    }



   // cout << "NUMBER OF EDGES: " << edges << endl;
   
    
  return g;
}










/*int main(){
    Graph<string> graph = construct_graph("Project1DataSetSmall/Project1DataSetSmall/Pipes_Madeira.csv");

    cout << "Numero de vertices: " << graph.getNumVertex() << endl;

    vector<Vertex<string>*> v = graph.getVertexSet();

    for (auto vertexPtr : v) {
    std::cout << vertexPtr->getInfo() << std::endl; // Supondo que getInfo() retorna o conteúdo do vértice como uma string
}

    


    



    
    /*unordered_map<string,City> cities;

    cities = getCities("Project1DataSetSmall/Project1DataSetSmall/Cities_Madeira.csv");
    
   string searchCode = "C_4";
    auto it = cities.find(searchCode);
   
   
    if (it != cities.end()) {
        // Found the city
        City& city= it->second;
        cout << "City: " << city.getCity() << endl;
        cout << "Id: " << city.getId() << endl;
        
        cout << "Code: " << city.getCode() << endl;
        cout << "Demand: " << city.getDemand() << endl;
        cout << "Population: " << city.getPopulation() << endl;
        
    } else {
        cout << "City with code " << searchCode << " not found." << endl;
    }

  
  return 0;
    
}
*/
