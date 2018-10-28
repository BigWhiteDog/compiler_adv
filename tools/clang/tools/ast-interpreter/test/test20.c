extern int GET();
extern int * MALLOC(int);
extern void FREE(int *);
extern void PRINT(int);


int main() {
   int* a;
   int b;
   b = 10;
   
   a = MALLOC(sizeof(int));
   *a = b;
   PRINT(*a);
   FREE(a);
}
