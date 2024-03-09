
#ifndef PROJ1_CITY_H
#define PROJ1_CITY_H


#include <iostream>
#include <string>

using namespace std;

class City{

private:
    string city;
    int id;
    string code;
    double demand;
    int population;

public: 
    City() : id(0), demand(0), population(0) {}

    City(const string& cityName, int cityId, const string& cityCode, double cityDemand, int cityPopulation)
        : city(cityName), id(cityId), code(cityCode), demand(cityDemand), population(cityPopulation) {}


    City(const City& other)
        : city(other.city), id(other.id), code(other.code), demand(other.demand), population(other.population) {}

    //Getters

    string getCity() const {
        return city;
    }

    int getId() const {
        return id;
    }
    
    string getCode() const {
        return code;
    }

    double getDemand() const {
        return demand;
    }

     int getPopulation() const {
        return population;
    }
    //Setters

     void setCity(const string& cityName) {
        city = cityName;
    }

    void setId(int cityId) {
        id = cityId;
    }

     void setCode(const string& cityCode) {
        code = cityCode;
    }

    void setDemand(double cityDemand) {
        demand = cityDemand;
    }

    void setPopulation(int cityPopulation) {
        population = cityPopulation;
    }
};

#endif //PROJ1_CITY_H
