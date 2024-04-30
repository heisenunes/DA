#include <iostream>
#include "src/DataParser.h"
#include "src/ToyGraph.h"

int main() {

    bool validChoice = false;
    int task_choice;

    while(!validChoice){
        cout << "Choose a task: " << std::endl;
        cout << "T2.1 - Backtracking Algorithm (1)" << std::endl;
        cout << "Choice: ";

        cin >> task_choice;
        cout << endl;

        string code;

        if (task_choice == 1){

            cout << "What Graph do You Want? " << endl;
            cout << "Toy Graph - Tourism (1)" << endl;
            cout << "Toy Graph - Stadiums (2)" << endl;
            cout << "Toy Graph - Shipping (3)" << endl;
            cout << "Choice: ";

            cin >> task_choice;
            cout << endl;

            std::cout << "Starting Backtracking Algorithm" << std::endl;
            DataParser dataparser;
            dataparser.createToyGraph(task_choice - 1);

            cout << "Do you want to use debug mode to see discarded paths as well?" << endl;
            cout << "0 - No" << endl;
            cout << "1 - Yes" << endl;
            cout << "Choice: ";
            cin >> task_choice;
            cout << endl;

            //dataparser.createToyGraph(1);
            ToyGraph toyGraph = dataparser.getToyGraph();
            //dataparser.printToyGraph();

            float minDistance;
            if (task_choice == 1){minDistance = toyGraph.tspDebug();}
            else {minDistance = toyGraph.tsp();}

            std::cout << "Minimum distance traveled is " << minDistance << std::endl;


            validChoice = true;
        }
    }

    return 0;


}
