#include <stdio.h>
#include <stdlib.h>

int main()
{
    int len = 20;
    char name[len];
    printf("Enter your name: ");
    fgets(name, len, stdin);
    printf("Your name is %s", name);
    return 0;
}