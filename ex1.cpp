#include "exercises.h"

Labyrinth::Labyrinth(int values[10][10]) {
    for (int i = 0; i < 10; i++)
        for (int j = 0; j < 10; j++)
            labyrinth[i][j] = values[i][j];
}

#include <iostream>

void  Labyrinth::print() const {
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            std::cout << labyrinth[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

bool Labyrinth::findGoal(int x, int y) {

    if(x == 1 && y == 1 && visited[1][1] == false){
        initializeVisited();
    }

    if(x <= 0 || x >= 9 || y <= 0 || y >= 9 || visited[x][y] == true  || labyrinth[x][y] == 0){
        return false;
    }
  else {
        visited[x][y] = true;
    }
    if(labyrinth[x][y] == 2){
        return true;
    }
    return (findGoal(x+1,y) || findGoal(x-1,y) || findGoal(x,y+1) || findGoal(x,y-1));
}  // compleixade temporal no pior caso: O(n^2)

void Labyrinth::initializeVisited() {
    for(int i = 0; i < 10; i++)
        for(int j = 0; j < 10; j++)
            visited[i][j] = false;
}

/// TESTS ///
#include <gtest/gtest.h>

TEST(TP2_Ex1, testLabyrinthPossible) {
    // This test assumes that the mazes only have a possible path to the exit
    int lab1[10][10] ={
        {0,0,0,0,0,0,0,0,0,0},
        {0,1,1,1,1,1,0,1,0,0},
        {0,1,0,0,0,1,0,1,0,0},
        {0,1,1,0,1,1,1,1,1,0},
        {0,1,0,0,0,1,0,0,0,0},
        {0,1,0,1,0,1,1,1,1,0},
        {0,1,1,1,0,0,1,0,1,0},
        {0,1,0,0,0,0,1,0,1,0},
        {0,1,1,1,0,0,1,2,0,0},
        {0,0,0,0,0,0,0,0,0,0}};

    Labyrinth l1(lab1);
    EXPECT_EQ(l1.findGoal(1,1),true);
}

TEST(TP2_Ex1, testLabyrinthImpossible) {
    int lab1[10][10] ={
            {0,0,0,0,0,0,0,0,0,0},
            {0,1,1,1,1,1,0,1,0,0},
            {0,1,0,0,0,1,0,1,0,0},
            {0,1,1,0,1,1,1,1,1,0},
            {0,1,0,0,0,1,0,0,0,0},
            {0,1,0,1,0,1,1,1,1,0},
            {0,1,1,1,0,0,1,0,1,0},
            {0,1,0,0,0,0,1,0,1,0},
            {0,1,1,1,0,0,0,2,0,0},
            {0,0,0,0,0,0,0,0,0,0}};

    Labyrinth l1(lab1);
    EXPECT_EQ(l1.findGoal(1,1),false);
}