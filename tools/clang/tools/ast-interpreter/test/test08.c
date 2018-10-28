extern int GET();
extern int * MALLOC(int);
extern void FREE(int *);
extern void PRINT(int);

int main() {
   int a = 0;
   int b = 0;

   while ( a < 10) {
      a = a + 1;
      b = b + 2;
   }
  PRINT(b);
}
