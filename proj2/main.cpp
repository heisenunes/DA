#include <iostream>
#include "tasks.cpp"
#include "parser.cpp"
#include <chrono>

using namespace std;

int main() {
    

    bool validChoice = false;
    int task_choice;
    int graph_choice;

    while(!validChoice){
        cout << "Choose a task: " << std::endl;
        cout << "T2.1 - Backtracking Algorithm (1)" << std::endl;
        cout << "T2.2 - Triangular Approximation (2)" << endl;
        cout << "T2.3 - Combination of nearest neighbor with 2-opt (3)" << endl;
        cout << "Choice: ";

        cin >> task_choice;
        cout << endl;

        string code;

        if (task_choice == 1){

            cout << "What Graph do You Want? " << endl;
            cout << "Toy Graph - Tourism (1)" << endl;
            cout << "Toy Graph - Stadiums (2)" << endl;
     
            cout << "Choice: ";

            cin >> graph_choice;
            cout << endl;


            Graph<int> g;
              

                switch(graph_choice) {
                  case 1:
                    g = construct_toygraph("Datasets/Toy-Graphs/tourism.csv");
                  break;
                  case 2:
                    g = construct_toygraph("Datasets/Toy-Graphs/stadiums.csv");
                  break;
                  default:
                  cout << "Invalid choice. Please run the program again and select either 1 or 2." << endl;
                  return 1; 
                }
            auto startTime = std::chrono::high_resolution_clock::now();

            float minDistance;
          
            minDistance = tsp(&g);
            
            cout << "- Minimum distance traveled is " << minDistance << std::endl;
            
             auto endTime = std::chrono::high_resolution_clock::now();
             std::chrono::duration<double> duration = endTime - startTime;


            std::cout << "Execution time: " << duration.count() << " s" << endl;


            validChoice = true;
        }

        else if(task_choice == 2 || task_choice == 3){
            
            cout << "What Graph do You Want? " << endl;
            cout << "Toy Graph - Tourism (1)" << endl;
            cout << "Toy Graph - Stadiums (2)" << endl;
            cout << "Extra fully connected Graph- edges_25(3)" << endl;
            cout << "Extra fully connected Graph- edges_50(4)" << endl;
            cout << "Extra fully connected Graph- edges_75(5)" << endl;
            cout << "Extra fully connected Graph- edges_100(6)" << endl;
            cout << "Extra fully connected Graph- edges_200(7)" << endl;
            cout << "Extra fully connected Graph- edges_300(8)" << endl;
            cout << "Extra fully connected Graph- edges_400(9)" << endl;
            cout << "Extra fully connected Graph- edges_500(10)" << endl;
            cout << "Extra fully connected Graph- edges_600(11)" << endl;
            cout << "Extra fully connected Graph- edges_700(12)" << endl;
            cout << "Extra fully connected Graph- edges_800(13)" << endl;
            cout << "Extra fully connected Graph- edges_900(14)" << endl;
            
            
            cout << "Graph Choice: ";
            cin >> graph_choice;
            cout << endl;

            Graph<int> g;

               switch(graph_choice) {
                  case 1:
                    g = construct_toygraph("Datasets/Toy-Graphs/tourism.csv");
                  break;
                  case 2:
                    g = construct_toygraph("Datasets/Toy-Graphs/stadiums.csv");
                  break;
                  case 3:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_25.csv");
                  break;
                   case 4:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_50.csv");
                  break;
                   case 5:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_75.csv");
                  break;
                  case 6:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_100.csv");
                  break;
                   case 7:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_200.csv");
                  break;
                   case 8:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_300.csv");
                  break;
                  case 9:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_400.csv");
                  break;
                  case 10:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_500.csv");
                  break;
                  case 11:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_600.csv");
                  break;
                  case 12:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_700.csv");
                  break;
                  case 13:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_800.csv");
                  break;
                  case 14:
                    g = construct_extra_fully_connected("Datasets/Extra-Fully-Connected-Graphs/edges_900.csv");
                  break;

                  default:
                  cout << "Invalid choice. Please run the program again and select either 1 or 2." << endl;
                  return 1; // Exit the program with an error code
                }

          if(task_choice == 2){
             auto startTime = std::chrono::high_resolution_clock::now();

            Graph<int> mst = primMST(&g);

            double min_distance;

            std::vector<int> result = pre_order(&mst, min_distance);

            int back = result.back();
            double final_edge_distance;

            Vertex<int>* origin = g.findVertex(0);
           
               
            for(auto e: origin->getAdj()){
               if(e->getDest()->getInfo() == back){
                final_edge_distance = e->getWeight();
             }
            
            } 

            min_distance = (min_distance/2) + final_edge_distance;

            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = endTime - startTime;

            cout << "Minimum distance traveled is: "<< min_distance << endl;

            std::cout << "Execution time: " << duration.count() << " s" << endl;

          

          cout << "Tour: ";
             for (int v : result) {
               std::cout << v << " ";
             }
            std::cout << "0" << endl;
          }
          else if(task_choice == 3){

             auto startTime = std::chrono::high_resolution_clock::now();
            vector<int> path = nearestNeighbor(&g);

            //vector<int> optimizedPath = twoOpt(path, &g);

            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = endTime - startTime;



            cout << "Path: ";
            for (int city : path) {
                cout << city << " ";
            }
            cout << endl;
           /* cout << "Optimized Path: ";
            for(int city:optimizedPath) {
                cout << city << " ";
            }
            */
            cout << endl;

            std::cout << "Execution time: " << duration.count() << " s" << endl;

          }

            validChoice = true;
        }

    }

    return 0;

}
