//
// Created by francisco on 28-02-2024.
//

#ifndef PROJ1_RESERVOIR_H
#define PROJ1_RESERVOIR_H

#include <iostream>
#include <string>

using namespace std;

class Reservoir{

private:
    string reservoir;
    string municipality;
    int id;
    string code;
    int maximum_Delivery;

public:
 
      Reservoir() : id(0) , maximum_Delivery(0){}

      Reservoir(const std::string& res, const std::string& mun, int identifier, const std::string& cd, int max_del) 
        : reservoir(res), municipality(mun), id(identifier), code(cd), maximum_Delivery(max_del) {}

      Reservoir(const Reservoir& other)
        : reservoir(other.reservoir), municipality(other.municipality), id(other.id), code(other.code), maximum_Delivery(other.maximum_Delivery) {}



    //Getters
    string get_reservoir() const {
        return reservoir;
    }

    string get_municipality() const {
        return municipality;
    }

    int getId() const {
        return id;
    }

    string getCode() const {
        return code;
    }

    int get_Maximum_Delivery() const {
        return maximum_Delivery;
    }
    //Setters

    void setReservoir(const string& ReservoirName) {
        reservoir = ReservoirName;
    }

    void setMunicipality(const string& municipalityName) {
        municipality = municipalityName;
    }

    void setId(int ReservoirId) {
        id = ReservoirId;
    }

    void setCode(const string& cityCode) {
        code = cityCode;
    }

    void setMaximumDelivery(int MaximumDelivery) {
        maximum_Delivery = MaximumDelivery;
    }

};


#endif //PROJ1_RESERVOIR_H
