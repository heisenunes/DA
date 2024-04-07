#include "Parser.cpp"
#include "tasks.cpp"



int main(){

    cout << "-------------------------------" << endl;
    cout << "|                             |" << endl;
    cout << "|      Welcome to the         |" << endl;
    cout << "| Water supply network Project|" << endl;
    cout << "|                             |" << endl;
    cout << "|                             |" << endl;
    cout << "-------------------------------" << endl;


Graph<string> g = construct_graph("Project1DataSetSmall/Project1DataSetSmall/Pipes_Madeira.csv");

unordered_map<string, Reservoir> reservoirs = getReservoirs("Project1DataSetSmall/Project1DataSetSmall/Reservoirs_Madeira.csv");
unordered_map<string, Station> stations = getStations("Project1DataSetSmall/Project1DataSetSmall/Stations_Madeira.csv");
unordered_map<string, City> cities = getCities("Project1DataSetSmall/Project1DataSetSmall/Cities_Madeira.csv");
unordered_map<int,Pipe> pipes = getPipes("Project1DataSetSmall/Project1DataSetSmall/Pipes_Madeira.csv");


bool validChoice = false;

while(!validChoice){
 cout << "Task to choose: " << endl;
 cout << "T2.1 (1)" << endl;
 cout << "T2.2 (2)" << endl;
 cout << "T2.3 (3)" << endl;
 cout << "T3.1 (4)" << endl;
 cout << "T3.2 (5)" << endl;
 cout << "T3.3 (6)" << endl;
 cout << "Choice: ";

 int task_choice;
 cin >> task_choice;
 
 cout << endl;
 cout << endl;

 string code;

 if(task_choice == 1){

   for (const auto& pair : cities) {
     cout << pair.second.getCity() << " (" << pair.second.getCode() << ")" << endl;
   }
   cout << endl;
 
   cout << "Choose City: ";
   
   cin >> code;
    
   double maxFlow = edmondsKarp(&g, "S", code);
   cout << "(City_code: " << code << ", Max flow: " << maxFlow<< ")" << endl;
   
    validChoice = true;
 }
 else if(task_choice == 2){
   
   t2_2(&g);
     
   validChoice = true;
 }

 else if(task_choice == 3){

   for (const auto& pair : cities) {
     cout << pair.second.getCity() << " (" << pair.second.getCode() << ")" << endl;
   }
   cout << endl;

  cout << "Choose City: ";
    string code;
    cin >> code;

    t2_3(&g,"S",code);
   validChoice = true;
 }
  else if(task_choice == 4) {
   
    for (const auto& pair : reservoirs) {
      cout << pair.second.get_reservoir() << " (" << pair.second.getCode() << ")" << endl;
    }
    cout << endl;
    cout << "Choose the reservoir to be removed:" << endl;
    cin >> code;

    t3_1(&g, code);

    validChoice = true;
  }
  else if(task_choice == 5) {
    
    for (const auto& pair : stations) {
      cout << pair.second.getCode() << " (" << pair.second.getCode() << ")" << endl;
    }
    cout << endl;
    cout << "Choose the station to be removed:" << endl;
    cin >> code;

    t3_2(&g, code);

    validChoice = true;
  }
 else if(task_choice == 6){

  int numberId;

   cout << "Pipes: " << endl;
    for(const auto &pair: pipes){
      cout << "(" << pair.second.getService_point_a() << "," << pair.second.getService_point_b() << ") " << "(" << pair.first << ")" << endl; 
    }
    cout << endl;
    cout << "Choose the edge to be removed: ";
    cin >> numberId;

    t3_3(&g, numberId);

    validChoice = true;
 }
}
 
    return 0;
}