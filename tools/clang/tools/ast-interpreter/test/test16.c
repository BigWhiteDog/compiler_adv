extern int GET();
extern int * MALLOC(int);
extern void FREE(int *);
extern void PRINT(int);

int f(int x) {
  int i = 0;
  while (i < x) {
     i = i + 2;
  }
  return i + 10;
}
int main() {
   int a;
   int b;
   a = 1;
   b = f(a);
   PRINT(b);
}
