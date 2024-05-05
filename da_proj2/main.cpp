#include <iostream>
#include "src/DataParser.h"
#include "src/ToyGraph.h"

int main() {

    bool validChoice = false;
    int task_choice;

    while(!validChoice){
        cout << "Choose a task: " << endl;
        cout << "T2.1 - Backtracking Algorithm (1)" << endl;
        cout << "Real Graph (2)" << endl;
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
        } else if (task_choice ==2){
            cout << "What Graph do You Want? " << endl;
            cout << "Extra Fully Connected Graph - 25 Edges (0)" << endl;
            cout << "Extra Fully Connected Graph - 50 Edges (1)" << endl;
            cout << "Extra Fully Connected Graph - 75 Edges (2)" << endl;
            cout << "Extra Fully Connected Graph - 100 Edges (3)" << endl;
            cout << "Extra Fully Connected Graph - 200 Edges (4)" << endl;
            cout << "Extra Fully Connected Graph - 300 Edges (5)" << endl;
            cout << "Extra Fully Connected Graph - 400 Edges (6)" << endl;
            cout << "Extra Fully Connected Graph - 500 Edges (7)" << endl;
            cout << "Extra Fully Connected Graph - 600 Edges (8)" << endl;
            cout << "Extra Fully Connected Graph - 700 Edges (9)" << endl;
            cout << "Extra Fully Connected Graph - 800 Edges (10)" << endl;
            cout << "Extra Fully Connected Graph - 900 Edges (11)" << endl;

            cout << "Real World Graph - Graph 1 (12)" << endl;
            cout << "Real World Graph - Graph 2 (13)" << endl;
            cout << "Real World Graph - Graph 3 (14)" << endl;

            cout << "Choice: ";
            cin >> task_choice;
            cout << endl;

            DataParser dataparser;
            dataparser.createRealWorldGraph(task_choice);

            RealWorldGraph realGraph = dataparser.getRealWorldGraph();
            dataparser.printRealGraph();


        }

    }

    return 0;


}
