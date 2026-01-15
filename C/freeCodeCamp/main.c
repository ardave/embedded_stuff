#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main()
{
    char s1[] = "abcdef";
    s1[0] = 'X';
    printf("s1: %s\n", s1);

    char *s2 = "abcdef";
    printf("s2: %s\n", s2);

    return 0;
}