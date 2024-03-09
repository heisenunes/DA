#include <iostream>
#include <vector>
#include "City.h"
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include "Station.h"
#include "Reservoir.h"

using namespace std;

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








int main()
{
    /*
    unordered_map<string,City> cities;

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
     */
}
