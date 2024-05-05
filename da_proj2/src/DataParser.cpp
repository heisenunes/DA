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

void DataParser::createRealWorldGraph(int newGraphType) {
    realGraph.setGraphType(newGraphType);
    int gType = realGraph.getGraphType(); //0 - edges_25.csv, 1 - edges_50.csv, 2 - edges_75.csv
    string filename;

    //** get nodes **//
    if(gType < 12){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/nodes.csv";
    } else if (gType == 12){
        filename = "../Datasets/Real-World-Graphs/graph1/nodes.csv";
    } else if (gType == 13){
        filename = "../Datasets/Real-World-Graphs/graph2/nodes.csv";
    } else if (gType == 14){
        filename = "../Datasets/Real-World-Graphs/graph3/nodes.csv";
    }

    std::ifstream file1(filename);

    string line1;
    getline(file1, line1); // skip header //

    while (getline(file1, line1)) {
        istringstream iss(line1);

        string token;

        getline(iss, token, ',');
        index = stoi(token);
        getline(iss, token, ',');
        longitude = stod(token);
        getline(iss, token, ',');
        latitude = stod(token);

        realGraph.addVertexLongLat(index,longitude,latitude);
    }

    file1.close();


    //** get edges **//
    if (gType == 0) {
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_25.csv";
    } else if (gType == 1) {
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_50.csv";
    } else if (gType == 2){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_75.csv";
    } else if (gType == 3){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_100.csv";
    } else if (gType == 4){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_200.csv";
    } else if (gType == 5){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_300.csv";
    } else if (gType == 6){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_400.csv";
    } else if (gType == 7){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_500.csv";
    } else if (gType == 8){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_600.csv";
    } else if (gType == 9){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_700.csv";
    } else if (gType == 10){
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_800.csv";
    } else if (gType == 11) {
        filename = "../Datasets/Extra-Fully-Connected-Graphs/edges_900.csv";
    }else if (gType == 12){
        filename = "../Datasets/Real-World-Graphs/graph1/edges.csv";
    } else if (gType == 13){
        filename = "../Datasets/Real-World-Graphs/graph2/edges.csv";
    } else if (gType == 14){
        filename = "../Datasets/Real-World-Graphs/graph3/edges.csv";
    }

    ifstream file(filename);

    string line;
    if(gType >11){
        getline(file, line); // skip header //
    }

    while (getline(file, line)) {
        istringstream iss(line);

        string token;

        getline(iss, token, ',');
        origin = stoi(token);
        getline(iss, token, ',');
        destination = stoi(token);
        getline(iss, token, ',');
        distance = stof(token);

        realGraph.addEdge(origin, destination, distance);
    }

    file.close();


}

RealWorldGraph& DataParser::getRealWorldGraph() {
    return this->realGraph;
}

void DataParser::printRealGraph() {

    int gType = realGraph.getGraphType();
    cout << "Number of vertices: " << realGraph.getVertices().size() << endl;
    cout << "Number of edges: " << realGraph.numEdges() << endl;
    cout << "Graph Type: " << gType << endl;

    for (const auto& vertex : realGraph.vertices) {
        cout << "Node " << vertex.index << " connects to:\n";
        for (const auto& edge : vertex.adj) {
            cout   << "  Node " << edge.destination << " with distance " << edge.distance << "\n";
        }
    }


}

