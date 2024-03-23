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


Graph<string> g = construct_graph("Project1LargeDataSet/Project1LargeDataSet/Pipes.csv");

bool validChoice = false;

while(!validChoice){
 cout << "Task to choose: " << endl;
 cout << "T2.1 (1)" << endl;
 cout << "T2.2 (2)" << endl;
 cout << "Choice: ";

 int task_choice;
 cin >> task_choice;
 
 cout << endl;
 cout << endl;
 if(task_choice == 1){

    cout << endl;
    cout << "Alcacer do Sal(C_1)" << endl;
    cout << "Aveiro(C_2)" << endl;
    cout << "Beja(C_3)" << endl;
    cout << "Braga(C_4)" << endl;
    cout << "Bragança(C_5)" << endl;
    cout << "Castelo Branco(C_6)" << endl;
    cout << "Coimbra(C_7)" << endl;
    cout << "Covilhã(C_8)" << endl;
    cout << "Estremoz(C_9)" << endl;
    cout << "Évora(C_10)" << endl;
    cout << "Faro(C_11)" << endl;
    cout << "Guarda(C_12)" << endl;
    cout << "Lagos(C_13)" << endl;

    cout << "Leiria(C_14)" << endl;
    cout << "Lisboa(C_15)" << endl;
    cout << "Portalegre(C_16)" << endl;
    cout << "Porto(C_17)" << endl;
    cout << "Santarém(C_18)" << endl;
    cout << "Setúbal(C_19)" << endl;
    cout << "Viana do Castelo(C_20)" << endl;
    cout << "Vila Real(C_21)" << endl;
    cout << "Viseu(C_22)" << endl;
    
    cout << "Choose City: ";
    string code;
    cin >> code;
    
    cout << "(City_code: " << code << ", Max flow: " << edmondsKarp(&g,"S",code) << ")" << endl;

    validChoice = true;
 }
 else if(task_choice == 2){
   
   t2_2(&g);
     
   validChoice = true;
 }
 else{
    cout << endl;
    cout << "Invalid Choice. Please try again!" << endl;
 }
}
 
    return 0;
}