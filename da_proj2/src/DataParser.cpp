//
//

#include "DataParser.h"

void DataParser::createToyGraph(int newGraphType) {

    graph.setGraphType(newGraphType);
    int gType = graph.getGraphType(); //0 - tourism.csv, 1 - stadiums.csv, 2 - shipping.csv
    std::string filename;
    if (gType == 0) {
        filename = "../Datasets/Toy-Graphs/tourism.csv";
    } else if (gType == 1) {
        filename = "../Datasets/Toy-Graphs/stadiums.csv";
    } else if (gType == 2){
        filename = "../Datasets/Toy-Graphs/shipping.csv";
    }

    std::ifstream file(filename);

    string line;
    getline(file, line); // skip header //

    while (std::getline(file, line)) {
        istringstream iss(line);

        //cout << line << endl;

        string token;

        getline(iss, token, ',');
        origin = stoi(token);
        getline(iss, token, ',');
        destination = stoi(token);
        getline(iss, token, ',');
        distance = stof(token);

        if(gType == 0){
            getline(iss, token, ',');
            labelOrigin = token;
            getline(iss, token, ',');
            labelDestination = token;
            graph.addVertexLabel(origin,labelOrigin);
            graph.addVertexLabel(destination,labelDestination);
        }else if (gType == 1 || gType == 2){
            graph.addVertex(origin);
            graph.addVertex(destination);
        }

        graph.addEdge(origin, destination, distance);
    }

    file.close();
}

ToyGraph& DataParser::getToyGraph() {
    return this->graph;
}

void DataParser::printToyGraph() {

    int gType = graph.getGraphType(); //0 - tourism.csv, 1 - stadiums.csv, 2 - shipping.csv
    cout << "Number of vertices: " << graph.getVertices().size() << endl;
    cout << "Number of edges: " << graph.numEdges() << endl;
    cout << "Graph Type: " << gType << endl;


    for (const auto& vertex : graph.vertices) {
        cout << "Node " << vertex.index << " connects to:\n";
        for (const auto& edge : vertex.adj) {
            cout   << "  Node " << edge.destination << " with distance " << edge.distance << "\n";
        }
    }


    /*
    cout << "node " << graph.getVertex(13).index << endl;
    for (const auto& edge : graph.getVertex(13).adj) {
        cout   << "  Node " << edge.destination << " with distance " << edge.distance << "\n";
    }
     */

}


