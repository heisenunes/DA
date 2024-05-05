cmake_minimum_required(VERSION 3.27)
project(da_proj2)

set(CMAKE_CXX_STANDARD 17)

add_executable(da_proj2 main.cpp
        src/ToyGraph.cpp
        src/ToyGraph.h
        src/DataParser.cpp
        src/DataParser.h
        src/RealWorldGraph.cpp
        src/RealWorldGraph.h
)
