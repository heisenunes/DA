//
// Created by francisco on 28-02-2024.
//

#ifndef PROJ1_PIPE_H
#define PROJ1_PIPE_H

#include <iostream>
#include <string>

using namespace std;

class Pipe{

private:
    string service_point_a;
    string service_point_b;
    int capacity;
    int direction;

public:
    //Getters

    string getService_point_a() const {
        return service_point_a;
    }

    string getService_point_b() const {
        return service_point_b;
    }

    int getCapacity() const {
        return capacity;
    }

    int getDirection() const {
        return direction;
    }

    //Setters

    void setService_point_a(const string& Service_point_a) {
        service_point_a = Service_point_a;
    }

    void setService_point_b(const string& Service_point_b) {
        service_point_b = Service_point_b;
    }

    void setCapacity(int Capacity) {
        capacity = Capacity;
    }

    void setDirection(int Direction) {
        direction = Direction;
    }
};




#endif //PROJ1_PIPE_H
