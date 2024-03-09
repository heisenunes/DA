cmake_minimum_required(VERSION 3.27)
project(Da_proj1)

set(CMAKE_CXX_STANDARD 17)

add_executable(Da_proj1
        data_structures/Graph.h
        data_structures/MutablePriorityQueue.h
        data_structures/UFDS.h
        data_structures/UFDS.cpp
        Parser.cpp
        City.h
        Pipe.h
        Reservoir.h
        Station.h
)
