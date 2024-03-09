cmake_minimum_required(VERSION 3.27)
project(proj1)

set(CMAKE_CXX_STANDARD 17)

add_executable(proj1 main.cpp
        data_structures/Graph.h
        data_structures/MutablePriorityQueue.h
        data_structures/UFDS.h
        data_structures/UFDS.cpp
        City.h
        Pipe.h
        WaterReservoir.h
        Station.h
)
