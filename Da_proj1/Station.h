//
// Created by francisco on 28-02-2024.
//

#ifndef PROJ1_STATION_H
#define PROJ1_STATION_H

#include <iostream>
#include <string>

using namespace std;

class Station{

private:

    int id;
    string code;

public:
    
    Station() : id(0){}
 // Constructor
    Station(int idValue, const std::string& codeValue) : id(idValue), code(codeValue) {}


    Station(const Station& other)
        : id(other.id), code(other.code){}

    //Getters

    int getId() const {
        return id;
    }

    string getCode() const {
        return code;
    }

    //Setters

    void setId(int cityId) {
        id = cityId;
    }

    void setCode(const string& cityCode) {
        code = cityCode;
    }

};

#endif //PROJ1_STATION_H
