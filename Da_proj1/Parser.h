#ifndef PARSER_H
#define PARSER_H

#include <unordered_map>
#include "City.h"
#include "Station.h"
#include "Reservoir.h"
#include "data_structures/Graph.h"

std::unordered_map<std::string, City> getCities(const std::string &filename);
std::unordered_map<std::string, Station> getStations(const std::string &filename);
std::unordered_map<std::string, Reservoir> getReservoirs(const std::string &filename);
Graph<std::string> construct_graph(const std::string &filename);

#endif // PARSER_H
