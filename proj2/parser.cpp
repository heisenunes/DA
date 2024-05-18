#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include "data_structures/Graph.h"

using namespace std;


Graph<int> construct_toygraph(const string &filename){
    Graph<int> g;
    //int edges = 0;
   
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return g; // Return empty vector
    }

    string line;
    getline(file, line); // Skip header line


     while (getline(file, line)) {
        stringstream ss(line);

        string origem,destino,distancia;

        getline(ss, origem, ',');
        getline(ss, destino, ',');
        getline(ss, distancia, ',');

        int origem_num = stoi(origem);
        int destino_num = stoi(destino);
       
        if(g.findVertex(origem_num) == nullptr){

             g.addVertex(origem_num);
 
        }

        if(g.findVertex(destino_num) == nullptr){
             g.addVertex(destino_num);

        }
        
      
        double distancia_num = stod(distancia);
       
      
        g.addBidirectionalEdge(origem_num,destino_num,distancia_num);
           
    }



    return g;
}

Graph<int> construct_extra_fully_connected(const string &filename){
    Graph<int> g;
    //int edges = 0;
   
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return g; // Return empty vector
    }

    string line;

     while (getline(file, line)) {
        stringstream ss(line);

        string origem,destino,distancia;

        getline(ss, origem, ',');
        getline(ss, destino, ',');
        getline(ss, distancia, ',');

        int origem_num = stoi(origem);
        int destino_num = stoi(destino);
       
        if(g.findVertex(origem_num) == nullptr){

             g.addVertex(origem_num);
 
        }

        if(g.findVertex(destino_num) == nullptr){
             g.addVertex(destino_num);

        }
        
      
        double distancia_num = stod(distancia);
       
      
        g.addBidirectionalEdge(origem_num,destino_num,distancia_num);

        
           
    }



    return g;
}
  