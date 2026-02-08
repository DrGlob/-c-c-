#include <stdio.h>
#include <stdint.h>
#include "allocator.h"

int main(void) {
    void* a = my_malloc(15);
    void* b = my_malloc(15);
    void* c = my_malloc(180);

    printf("a=%p b=%p c=%p\n", a, b, c);

    int x;
    my_free(&x);                 // не из heap -> проигнорируется


    my_free(b);
    my_free(a);
    my_free(a);
    my_free(c);

    void* d = my_malloc(15);
    my_free((char*)d + 1);       // сдвинутый -> проигнорируется
    printf("d=%p\n", d);

    return 0;
}