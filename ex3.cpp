// By: Gonçalo Leão
#include <algorithm>
#include "exercises.h"

bool changeMakingBF(unsigned int C[], unsigned int Stock[], unsigned int n, unsigned int T, unsigned int usedCoins[]) {
    //TODO...
    unsigned int CAux[n];
    unsigned int usedCoinsAux[n];
    unsigned int usedCoinsAnswer[n];
    unsigned int StockAux[n];
    int c = n;
    for(int i = 0; i < n; i++){
        usedCoins[i] = 0;
        CAux[i] = C[i];
        usedCoinsAux[i] = usedCoins[i];
        StockAux[i] = Stock[i];
    }
    unsigned int TAux = T;
    int minCoins = INT_MAX;
    bool found = false;
    while(n > 0){
        int nCoins = 0;
        for(int i = n - 1; i >= 0; i--){
            while(T >= C[i] && Stock[i] > 0){
                Stock[i] -= 1;
                T -= C[i];
                usedCoins[i] += 1;
                nCoins++;
            }
        }
        if(nCoins < minCoins && T == 0){
            for(int i = 0; i < c; i++){
                usedCoinsAnswer[i] = usedCoins[i];
            }
            minCoins = nCoins;
            found = true;
        }
        for(int i = 0; i < c; i++){
            Stock[i] = StockAux[i];
            usedCoins[i] = usedCoinsAux[i];
            C[i] = CAux[i];
        }
        T = TAux;
        n-=1;
    }
    if(found){
        for(int i = 0; i < c; i++){
            usedCoins[i] = usedCoinsAnswer[i];
        }
        return true;
    }
    for(int i = 0; i < c; i++){
        usedCoins[i] = usedCoinsAux[i];
    }
    return false;
}

/// TESTS ///
#include <gtest/gtest.h>

TEST(TP1_Ex3, hasBFChangeCanonical) {
    unsigned int C[] = {1,2,5,10};
    unsigned int Stock[] = {1,1,1,1};
    unsigned int n = 4;
    unsigned int usedCoins[4];

    EXPECT_EQ(changeMakingBF(C,Stock,n,13,usedCoins), true);
    EXPECT_EQ(usedCoins[0], 1);
    EXPECT_EQ(usedCoins[1], 1);
    EXPECT_EQ(usedCoins[2], 0);
    EXPECT_EQ(usedCoins[3], 1);

    unsigned int Stock2[] = {1,2,4,2};
    EXPECT_EQ(changeMakingBF(C,Stock2,n,38,usedCoins), true);
    EXPECT_EQ(usedCoins[0], 1);
    EXPECT_EQ(usedCoins[1], 1);
    EXPECT_EQ(usedCoins[2], 3);
    EXPECT_EQ(usedCoins[3], 2);
}

TEST(TP1_Ex3, hasBFChangeNonCanonical) {
    unsigned int C[] = {1,4,5};
    unsigned int Stock[] = {3,2,1};
    unsigned int n = 3;
    unsigned int usedCoins[3];

    EXPECT_EQ(changeMakingBF(C,Stock,n,6,usedCoins), true);
    EXPECT_EQ(usedCoins[0], 1);
    EXPECT_EQ(usedCoins[1], 0);
    EXPECT_EQ(usedCoins[2], 1);

    EXPECT_EQ(changeMakingBF(C,Stock,n,8,usedCoins), true);
    EXPECT_EQ(usedCoins[0], 0);
    EXPECT_EQ(usedCoins[1], 2);
    EXPECT_EQ(usedCoins[2], 0);
}

TEST(TP1_Ex3, hasNoBFChange) {
    unsigned int C[] = {1,2,5,10};
    unsigned int Stock[] = {0,1,1,1};
    unsigned int n = 4;
    unsigned int usedCoins[4];

    EXPECT_EQ(changeMakingBF(C,Stock,n,18,usedCoins), false);
    EXPECT_EQ(changeMakingBF(C,Stock,n,1,usedCoins), false);
}