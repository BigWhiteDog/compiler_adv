extern int GET();
extern int * MALLOC(int);
extern void FREE(int *);
extern void PRINT(int);

int main() {
   int a = 0;

   for (a = 0; a < 10; a = a + 1) ;
 
   PRINT(a);
}
