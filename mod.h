#pragma once

#include <assert.h>
#include <thread>         // std::thread
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <string>
#include <vector>
#include <functional>
#include <map>
#include "util.h"

extern const uint16_t gNotes[];

struct Sample
{
   std::string name;
   size_t length = 0;
   int fine_tune = 0;
   int volume = 0;
   int loop_start = 0;
   int loop_length = 0; 
   int8_t* data = nullptr;
};

struct Channel
{
    float freq = 0.f;   // actual freq in hz to play sound.
    float samplePos = 0; // float for now.
    int freqOffset = 0; // offset in note table
    int vol = 0;
    int ft = 0;
    Sample* sample = nullptr;
    Sample* lastSample = nullptr;
    Channel() {}

    // playback
    void setSample(Sample* s) 
    {
        lastSample = s;
    } 

    void play(int offset=0) { samplePos = offset; }
    void setFineTune(int ftune) { ft = ftune; }
    void setVol(int v) { vol = v; }
    void setFreq(int foffset) 
    { 
        freq = getFreqHz(foffset,ft);
    }

    static float getFreqHz(int foffset, int finetune)
    {
        int ftoff = 0;//12*3*7;//finetune;
        static uint32_t max = 12*3*16;
        int offset = foffset + ftoff;
        assert(offset < max);
        assert(offset >= 0);
        float amigaval = gNotes[offset];
        assert(amigaval > 0);
        return (float)7159090.5 / (amigaval * 2);
    }

    // render thread.
    int makeAudio(float* dst, float dstSize, float sampleRate );
};

struct Note
{
    uint16_t noteOffset:11;  // 0-??  = 11 bits = 0-2048 should be plenty for your needs.
    uint8_t sampleNumber:5;  // 0-31  = 5 bits
    uint8_t effect;    // 0-15  = 4 bits, but use 8 to keep things even
    uint8_t eparm;     // 0-255 = 8 bits
};

struct Mod
{
    int nPatterns = 0;
    int nChannels = 0;
    int songLength = 0;
    std::string name;
    std::vector<Sample> samples;
    std::vector<int> order;
    std::vector<Channel> channels;
    Note* patternData = nullptr;
    std::mutex cs;

    //playback
    int speed = 6; // default
    int currentOrder = 0; // index into order table.
    int currentRow = 0;
    int currentTick = 0;

    void tick();
    void updateRow(); 
    void updateEffects() {}

    size_t makeAudio(RingBuffer<float>* stereoOutput, float sampleRate, int frameSize);
}; 
