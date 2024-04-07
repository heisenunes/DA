#include <iostream>
#include <vector>
#include "City.h"
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include "Station.h"
#include "Reservoir.h"
#include "Pipe.h"
#include "data_structures/Graph.h"


using namespace std;

const int INFINITY_INT = std::numeric_limits<int>::max();

/**
 * @brief Parses a CSV file containing city data and returns an unordered map of cities.
 *  
 * @param filename The name of the CSV file to read city data from.
 * @return An unordered map with city codes as keys and corresponding City objects as values.
 *         If the file cannot be opened or read, an empty unordered map is returned.
 * 
 * @note The CSV file is expected to have the following format:
 *       "city_name,id,code,demand,population"
 *       Each field is separated by a comma.
 */

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

/**
 * @brief Parses a CSV file containing station data and returns an unordered map of stations.
 *  
 * @param filename The name of the CSV file to read station data from.
 * @return An unordered map with station codes as keys and corresponding Station objects as values.
 *         If the file cannot be opened or read, an empty unordered map is returned.
 * 
 * @note The CSV file is expected to have the following format:
 *       "id,Code"
 *       Each field is separated by a comma.
 */

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

/**
 * @brief Parses a CSV file containing water reservation data and returns an unordered map of water reservations.
 *  
 * @param filename The name of the CSV file to read water reservation data from.
 * @return An unordered map with water reservation codes as keys and corresponding Water reservations objects as values.
 *         If the file cannot be opened or read, an empty unordered map is returned.
 * 
 * @note The CSV file is expected to have the following format:
 *       "reservoir_name,municipality,id,code,maximum_delivery"
 *       Each field is separated by a comma.
 */

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

/**
 * @brief Parses a CSV file containing pipes data and returns an unordered map of pipes.
 *  
 * @param filename The name of the CSV file to read pipe data from.
 * @return An unordered map with pipes ids as keys and corresponding Pipes objects as values.
 *         If the file cannot be opened or read, an empty unordered map is returned.
 * 
 * @note The CSV file is expected to have the following format:
 *       "service_point_a,service_point_b,capacity,code,direction"
 *       Each field is separated by a comma.
 */

unordered_map<int,Pipe> getPipes(const string &filename){
  unordered_map<int,Pipe> pipeMap;

    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return pipeMap; // Return empty vector
    }
    
    string line;
    getline(file, line); // Skip header line

    int numberId = 1;

    while (getline(file, line)) {
        stringstream ss(line);

        string service_point_a, service_point_b,capacity,direction;
        getline(ss, service_point_a, ',');
        getline(ss, service_point_b, ',');
        getline(ss, capacity, ',');
        getline(ss, direction, ',');
        
        // Convert strings to appropriate types
        int capacity_num = stoi(capacity);
        int direction_num = stoi(direction);

        if(direction_num == 1){
            Pipe pipe(numberId,service_point_a,service_point_b,capacity_num,direction_num);
            pipeMap[numberId] = pipe;
            numberId++;
        }
        else{
            
            Pipe pipe1(numberId,service_point_a,service_point_b,capacity_num,direction_num);
            pipeMap[numberId] = pipe1;

            numberId++;
            Pipe pipe2(numberId,service_point_b,service_point_a,capacity_num,direction_num);
            pipeMap[numberId] = pipe2;
            numberId++;
        }

    }

 file.close();

  return pipeMap;
} 

/**
 * @brief Constructs a graph from a CSV file containing service point connections and capacities.
 * 
 * This function reads data from a CSV file containing information about connections between service points
 * in a network, along with their capacities. It constructs a graph representation of this network.
 * 
 * @param filename The name of the CSV file to read service point data from.
 * @return A graph representing the service point network, with vertices representing service points
 *         and edges representing connections between them with corresponding capacities.
 *         If the file cannot be opened or read, an empty graph is returned.
 * 
 * @note The CSV file is expected to have the following format:
 *       "service_point_a,service_point_b,capacity,direction"
 *       Each field is separated by a comma. 'direction' field should be 0 for bidirectional edges
 *       and 1 for unidirectional edges.
 * 
 */

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

