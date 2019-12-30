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
    float samplePos = -1; // float for now.
    int vol = 0;
    int ft = 0;
    int pan = 0;
    int amigaPeriod = 0;
    Sample* sample = nullptr;
    int volRamp = 0;
    int volRampTicksLeft = 0;
    int portaSpeed = 0;
    int portaToNoteOffset = 0;
    Channel() {}

    static float amigaPeriodToHz(int amigaFreq)
    {
        assert(amigaFreq > 0);
        return (float)7159090.5f / (amigaFreq * 2);
    }
    static int getAmigaFreq(uint16_t offsetInNoteTable, int finetune = 0)
    {
        if( offsetInNoteTable != 0 && offsetInNoteTable >= 12*3*16) return -1;
        offsetInNoteTable--; // subtract 1 as 0 is an error. 
        int finetuneoffset = 12*3 * finetune; // can be negative.
        // clamp fine tune in range.
        for( ; offsetInNoteTable+finetuneoffset < 0 || offsetInNoteTable+finetuneoffset > 12*3*16
             ; finetuneoffset = 12*3 * finetune )
        {
            if( finetune > 0) finetune--;
            else if (finetune < 0) finetune++;
        }
        return gNotes[offsetInNoteTable + finetuneoffset];
    }

    // playback
    void setSample(Sample* s);
    void setFineTune(int ftune) { ft = ftune; }
    void setVol(int v) { vol = v; }
    // render thread.
    int makeAudio(
        std::vector<float>& leftMix,
        std::vector<float>& rightMix,
        int frameSize,
        float sampleRate );
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
    void updateEffects();

    size_t makeAudio(
        std::vector<float>& mixLeft,
        std::vector<float>& mixRight,
        float sampleRate, int frameSize);
}; 
