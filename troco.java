import java.util.Arrays;

public class troco{

 static final int moedas[] = {1,2,5,10};

 public static int[] select(int montante,int [] stock){
   int [] sel = new int[moedas.length];


   for(int i = moedas.length-1; i >= 0 && montante > 0; i--)
     if(moedas[i] <= montante && stock[i] > 0){
       int n_moed;
       if(stock[i] < montante/moedas[i]){
         n_moed = stock[i];
       }
       else{
         n_moed = montante/moedas[i];
       }


       sel[i] += n_moed;
       montante -= n_moed * moedas[i];
     }

 if(montante > 0)
 return null;
 else
 return sel;
}

  public static void main(String[] args){

    int arr[] = new int[]{3,5,2,1};

    System.out.println(Arrays.toString(select(8,arr)));

  }
}
