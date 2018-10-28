extern int GET();
extern int * MALLOC(int);
extern void FREE(int *);
extern void PRINT(int);

int main() {
   int a;
   int b;
   a = 10;
   b = 10;
   PRINT(a+b);
}
