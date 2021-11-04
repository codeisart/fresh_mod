#include <stdio.h>
#include "util.h"

bool loadPlm(void* mem, int lenth)
{
    return false;
}

bool loadPlm(const char* filename)
{
    printf("loading plm... ");
    return loadWholeFile(filename, [](void* mem, size_t size) -> bool { return loadPlm(mem,size); });
}
