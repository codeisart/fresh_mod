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

enum class Effect : uint8_t
{
    Arpeggio = 0,
    Porta_Up = 1,
    Porta_Down = 2,
    Porta_To_Note = 3,
    Vibrato = 4,
    Porta_Vol_Slide = 5,
    Vibrato_Vol_Slide = 6,
    Tremolo = 7,
    Pan = 8,
    Sample_Offset = 9,
    Volume_Slide = 0xa,
    Jump_To_Pattern = 0xb,
    Set_Volume = 0xc,
    Pattern_Break = 0xd,
    Sub_Effect = 0xe,
    Set_Speed = 0xf,
};
enum class EffectSubType
{
    Set_Filter,
    Fine_Porta_Up,
    Fine_Porta_Down,
    Glissando_Control,
    Set_Vibrato_Waveform,
    Set_Finetune,
    Pattern_Loop,
    Set_Tremolo_WaveForm,
    Unused,
    Retrig_Note,
    Fine_Volume_Slide_Up,
    Fine_Volume_Slide_Down,
    Cut_Note,
    Delay_Note,
    Pattern_Delay,
    Invert_Loop,
};

struct Sample
{
   std::string name;
   size_t length = 0;
   int fine_tune = 0;
   int volume = 0;
   int loop_start = 0;
   int loop_length = 0; 
   std::vector<float> data;
};

struct Note
{
    uint16_t noteOffset:11;         // 0-??  = 11 bits = 0-2048 should be plenty for your needs.
    uint8_t sampleNumber:5;         // 0-31  = 5 bits
    Effect effect;                  // 0-15  = 4 bits, but use 8 to keep things even
    uint8_t eparm;                  // 0-255 = 8 bits
};

struct Channel
{
    float samplePos = 0; 
    int vol = 0;
    int ft = 0;
    int pan = 0;
    bool bSurround = false;
    int amigaPeriod = 0;
    Sample* sample = nullptr;
    float vuDb = -96.f;
    int voiceHandle = -1;
    
    // volume slide
    int volSlideSpeed = 0;
    int volSlideTicksLeft = 0;
    
    // portamento.
    int portaSpeed = 0;
    int portaToNoteOffset = 0;
    int portaAmmount = 0;
    int portaTicksLeft = 0;
    
    // vibrato 
    int vibrPos = 0;
    int vibrSpeed = 0;
    bool vibrInv = false;
    int vibrDepth = 0;
    int vibrTicksLeft = 0;

    // temello 
    int tremePos = 0;
    int tremeSpeed = 0;
    bool tremeInv = false;
    int tremeDepth = 0;
    int tremeTicksLeft = 0;

    int delayNoteTicksLeft = 0;
    Note delayNote;

    Channel() {}

    static uint32_t amigaPeriodToHz(int amigaFreq)
    {
        assert(amigaFreq > 0);
        //return (float)7159090.5f / (amigaFreq * 2);
        return 7159090 / (amigaFreq*2); 

    }
    static int getAmigaFreq(uint16_t offsetInNoteTable, int finetune = 0)
    {
        if( offsetInNoteTable != 0 && offsetInNoteTable >= 12*3*16) return -1;
        offsetInNoteTable--; // subtract 1 as 0 is an error. 
        int finetuneoffset = 12*3 * finetune; // can be negative.
        // clamp fine tune in range.
        //for( ; offsetInNoteTable+finetuneoffset < 0 || offsetInNoteTable+finetuneoffset > 12*3*16
        //     ; finetuneoffset = 12*3 * finetune )
        //{
        //    if( finetune > 0) finetune--;
        //    else if (finetune < 0) finetune++;
        //}
        return gNotes[offsetInNoteTable + finetuneoffset];
    }

    // playback
    void setSample(Sample* s);
    void setFineTune(int ftune);
    void setAmigaPeriod(int ap);
    void setVol(int v);

    // render thread.
    int makeAudio(
        std::vector<float>& leftMix,
        std::vector<float>& rightMix,
        int frameSize,
        uint32_t sampleRate );
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
    int currentSpeed = 6; // default
    int currentOrder = 0; // index into order table.
    int currentRow = 0;
    int currentTick = 0;
    int patternDelay = 0;

    void tick();
    void updateRow(); 
    void updateEffects();

    void startVolSlide(Channel& channel, Note& note);
    void startVibrato(Channel& channel, Note& note, bool bKeepOld);
    void startPortamento(Channel& channel, Note& note, bool bTargetNote, bool bKeepOld );
    void startTremello(Channel& channel, Note& note);

    size_t makeAudio(
        std::vector<float>& mixLeft,
        std::vector<float>& mixRight,
        uint32_t sampleRate, int frameSize);
}; 
