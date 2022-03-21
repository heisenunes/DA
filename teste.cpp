#include <iostream>

bool changeMakingGreedy(unsigned int C[], unsigned int Stock[], unsigned int n, unsigned int T, unsigned int usedCoins[]) {
  // c = [1,2,5,10], stock = [1,2,4,2] n=4, T=38

    for(int i = n-1; i >= 0 && T > 0; i--){
        usedCoins[i] = 0;
        if(C[i] <= T && Stock[i] > 0){
            int n_moed = 0;
          if(Stock[i] < T/C[i]){
            n_moed = Stock[i];
          }
          else{
              n_moed = T/(C[i]);
          }
          usedCoins[i] += n_moed;
          T -= (n_moed * C[i]);
        }
      }

    if(T <= 0){
        return true;
    }

    return false;
}


int main(){

  unsigned int C[] = {1,2,5,10};
  unsigned int Stock[] = {3,5,2,1};
  unsigned int n = 4;
  unsigned int usedCoins[4];

  std::cout << "valor de verdade: " << changeMakingGreedy(C,Stock,n,8,usedCoins) << std::endl;

  for(int i = 0; i < 4; i++){
    std::cout << usedCoins[i] << std::endl;
  }



  return 0;
}
