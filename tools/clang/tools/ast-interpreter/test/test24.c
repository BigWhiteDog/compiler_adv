extern int GET();
extern int * MALLOC(int);
extern void FREE(int *);
extern void PRINT(int);

int main() {
   int* a;
   int **b;
   int *c;
   a = MALLOC(sizeof(int)*2);
   b = (int **)MALLOC(sizeof(int *));

   *b = a;
   *a = 10;
   *(a+1) = 20;

   c = *b;
   PRINT(*c);
   PRINT(*(c+1));
   FREE(a);
   FREE((int *)b);
}
