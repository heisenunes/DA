//
//

#ifndef DA_PROJ2_DATAPARSER_H
#define DA_PROJ2_DATAPARSER_H

#include <fstream>
#include <sstream>
#include <string>
#include "ToyGraph.h"
#include "RealWorldGraph.h"

class DataParser {
private:
    int nVertices;
    int nEdges;
    int index;
    double latitude;
    double longitude;
    int origin;
    int destination;
    float distance;
    string labelOrigin;
    string labelDestination;


    ToyGraph graph = ToyGraph();
    RealWorldGraph realGraph = RealWorldGraph();
public:

    void createToyGraph(int graphType); //0 - tourism.csv, 1 - stadiums.csv, 2 - shipping.csv
    ToyGraph& getToyGraph();
    void printToyGraph();

    void createRealWorldGraph(int graphType);

    RealWorldGraph& getRealWorldGraph();
    void printRealGraph();

};


#endif //DA_PROJ2_DATAPARSER_H
