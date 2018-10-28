extern int GET();
extern int * MALLOC(int);
extern void FREE(int *);
extern void PRINT(int);

int  f(int b) {
   return b+1;
}

int main() {
   int a;
   a = 5;

   while(a < 10) {
     a = f(a) + 1;
   }
   
   PRINT(a);
}
