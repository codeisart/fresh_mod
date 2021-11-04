#include <stdio.h>
#include "util.h"
#include <stdio.h>
#include <memory.h>

bool loadPlm(void* mem, int len)
{
    uint8_t* readhead = (uint8_t*)mem;
    if(len < 4) return false;

    // ID.
    uint32_t ID = *(uint32_t*)readhead; 
    uint8_t PLM[] { 0x50, 0x4C, 0x4D, 0x1A };    
    if(ID != *(uint32_t*)PLM) 
        return false;
    readhead+=4;

    // Version.
    uint8_t headersize = *readhead++;
    uint8_t version = *readhead++;

    // Song name. 48 chars, null terminated.
    std::string name( (const char*)readhead);
    readhead+=48;

    int channels = *readhead++;
    uint32_t flags = *readhead++;
    int maxvol = *readhead++;
    int amplify = *readhead++;
    int bpm = *readhead++;
    int speed = *readhead++;
   
    uint8_t pan[32];
    memcpy(pan, readhead,32);
    readhead+=32;
   
    int nSamples = *readhead++;
    int nPatters = *readhead++;
    int nOrderes = *(uint16_t*)readhead;
    readhead+=2;


    return true;
}

bool loadPlm(const char* filename)
{
    printf("loading plm... ");
    return loadWholeFile(filename, [](void* mem, size_t size) -> bool { return loadPlm(mem,size); });
}
