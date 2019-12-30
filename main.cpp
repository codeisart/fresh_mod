#include "portaudio.h"
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <assert.h>
#include <thread>         // std::thread
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <string>
#include <vector>
#include <functional>
#include <map>
#include <sstream>
#include "util.h"
#include "mod.h"

static const char* notestr[] = {"C-", "C#", "D-", "D#","E-", "F-", "F#", "G-","G#","A-", "A#", "B-" };
extern const uint16_t gNotes[];
#define SAMPLE_RATE (48000)
#define NUM_SECONDS (60)

static const int kFrameSize = 700;// SAMPLE_RATE /20.f;
Mod gMod;

uint16_t findOffsetFromPeriod(int amigaFreq)
{    
    if( amigaFreq <= 0) return 0;
    const uint16_t* normal = gNotes+(12*3*8); // offset to normal range.
    for(int i = 0; i < 12*3; ++i)
    {
        if(amigaFreq >= normal[i])
            return i+1+(12*3*8);
    }
    return 0;
}

void offsetToFancyNote(int offset, int& semi, int& oct, int& ft)
{
    offset--; 
    ft = offset / (12*3);
    semi = offset % 12;
    oct = (offset - semi - (36*ft)) / 12;
    ft -=8;
    oct+=3;
}
void prettyPrintNote(Note* note, int channel, int nChannels, int row)
{
    // pretty print.
    int semi, oct, ft;
    offsetToFancyNote(note->noteOffset, semi, oct, ft);

    if (channel == 0)
    {
        printf("\n");
        printf("%02x    ", row);
    }
    // note.
    if (note->noteOffset != 0)
        printf("%s%d %02d ", notestr[semi], oct, ft);
    else
        printf("...... ");
    // sample #
    if (note->sampleNumber)
        printf("%02x ", note->sampleNumber);
    else
        printf(".. ");
    // effect.
    printf("%x%02x", note->effect, note->eparm);
    printf("     ");
}

void timer_start(std::function<void(void)> func, unsigned int interval)
{
  std::thread([func, interval]()
  { 
    while (true)
    { 
      auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);
      func();
      std::this_thread::sleep_until(x);
    }
  }).detach();
}

void Channel::setSample(Sample* s)
{
    sample = s;
    samplePos = 0;
    ft = sample->fine_tune;
}

// render thread.
int Channel::makeAudio(
    std::vector<float>& leftMix,
    std::vector<float>& rightMix,
    int frameSize,
    float sampleRate)
{
    if( freq <= 0)
        return 0;
    if( !sample )
        return 0;
    if( samplePos < 0)
        return 0;

    float srcStep = freq / (double)sampleRate;

    float end = sample->loop_length > 0 ?
                sample->loop_start + sample->loop_length :
                sample->length;

    float* left = leftMix.data();
    float* right = rightMix.data();
    static const float s8ToFloatRecp = 1.f / 127.f;
    static const float vol64FloatRecp = 1.f / 64.f;
    float fltvol = (float)vol * vol64FloatRecp;

    // too sleepy, think about this properly.
    float fltLeftPan  = Abs(Min(0, pan)) * vol64FloatRecp;
    float fltRightPan = Max(0, pan) * vol64FloatRecp; 

    int i = 0; 
    while( i < frameSize )
    {
        // handle end of sample.
        if( samplePos >= end )
        {
            // looping?
            if( sample->loop_length > 0)
                samplePos = sample->loop_start;
            else 
            {
                samplePos = -1.f;
                break;
            }
        }

        float s = (float) sample->data[(int)samplePos];
        s*= s8ToFloatRecp;
        s*= fltvol;
        (*left++) += s;
        (*right++) += s;
        
        samplePos+=srcStep;
        ++i;
    }
    return i;
}

void Mod::tick()
{
    currentTick++;
    if (currentTick >= speed)
    {
        std::unique_lock<std::mutex> lk(cs);
        updateRow();
        currentTick = 0;
        currentRow++;
        if( currentRow>= 64)
        {
            currentRow = 0;
            currentOrder++;
            if( currentOrder >= songLength )
            {
                currentOrder = 0;
            }
        }
    }
    else
    {
        updateEffects();
    }
}

void Mod::updateRow() 
{
    //std::unique_lock<std::mutex> lk(cs);

    int patternIdx = order[currentOrder];
    //printf("updaterow... ptn=%d, ord=%d, row=%d\n", patternIdx, currentOrder, currentRow );
    for(int channelIdx = 0; channelIdx < nChannels; ++channelIdx)
    //int channelIdx = 2;
    {        
        Note& note = patternData[(64*nChannels*patternIdx)+(nChannels*currentRow)+channelIdx];
        Channel& channel = channels[channelIdx];

        //prettyPrintNote(&note, channelIdx, nChannels, currentRow);
        if( note.sampleNumber )
        {
            Sample* smpl = &samples[note.sampleNumber];
            channel.setSample(smpl);
            channel.setVol(smpl->volume);
            //printf( "chn %d, set sample %d '%s'\n", channelIdx, note.sampleNumber, smpl->name.c_str());  
        }
        if( note.noteOffset != 0 ) 
        {
           if (note.effect != 3 && note.effect != 5) 
           {
                if( channel.sample)
                    channel.ft = channel.sample->fine_tune;
                
                int period = Channel::getAmigaFreq(note.noteOffset, channel.ft);
                if( period > 0 )
                {
                    channel.amigaPeriod = period;
                    channel.freq = Channel::amigaPeriodToHz(period);
                }
           }
        }
        if( note.effect == 0 && note.eparm == 0)
            int nop = 0;
        else if (note.effect == 0xf)
            speed = note.eparm;
        else if (note.effect == 0xc) // set volume
            channel.setVol(note.eparm);
        else if (note.effect == 0xd) // pattern break.
        {
            if (note.eparm > 64)
                currentRow = 0;
            else
                currentRow = note.eparm;
            currentOrder++;
            currentRow--; // We are about to increment this, so make it -1 that we want it.
        }
        else if (note.effect == 0xe && (note.eparm >> 4) == 0x8) // panning
        {
            int pan = note.eparm & 0xf;
            pan -= 8;
            channel.pan = pan * 8;
        }
        else if (note.effect == 0x9) // sample offset.
            channel.samplePos = (note.eparm * 0x100);
        else if (note.effect == 0xa)
        {
            int8_t lowNib = -(note.eparm & 0xf);
            int8_t hiNib = (note.eparm >> 4) & 0xf;
            channel.volRamp = lowNib ? lowNib : hiNib;
            channel.volRampTicksLeft = speed - 1;
        }
        else if (note.effect == 0x3) // porta to note.
        {
            // new target?
            if (note.noteOffset)
                channel.portaToNoteOffset = note.noteOffset;
            if (note.eparm)
                channel.portaSpeed = note.eparm;
        }
        else if(note.effect == 0x5) // porta to note and volume slide combo.
        {
            int8_t lowNib = -(note.eparm & 0xf);
            int8_t hiNib = (note.eparm >> 4) & 0xf;
            channel.volRamp = lowNib ? lowNib : hiNib;
            channel.volRampTicksLeft = speed - 1;
            // new target?
            if (note.noteOffset)
                channel.portaToNoteOffset = note.noteOffset;
            if (channel.portaSpeed == 0 && channel.prevPortaSpeed)
                channel.portaSpeed = channel.prevPortaSpeed;
        }
        else
        {
            printf("effect unhandled %x%02x, row=%d, channel=%d\n", 
                note.effect, note.eparm, currentRow, channelIdx);
        }
    }
}

void Mod::updateEffects()
{
    int patternIdx = order[currentOrder];
    for(int channelIdx = 0; channelIdx < nChannels; ++channelIdx)
    {
        Channel& c = channels[channelIdx];
        Note& note = patternData[(64*nChannels*patternIdx)+(nChannels*currentRow)+channelIdx];

        if( c.volRampTicksLeft >0 )
        {
            c.vol = Clamp(0, 64, c.vol+c.volRamp);
            c.volRampTicksLeft--;
        }
        if(c.portaSpeed > 0)  
        {
            // going up or down?
            uint16_t targetFreqAmiga = Channel::getAmigaFreq(c.portaToNoteOffset, c.ft);
            if( targetFreqAmiga > c.amigaPeriod ) 
                c.amigaPeriod = Min<int>(c.amigaPeriod + c.portaSpeed, targetFreqAmiga );
            else if( targetFreqAmiga < c.amigaPeriod )
                c.amigaPeriod = Max<int>(c.amigaPeriod - c.portaSpeed, targetFreqAmiga);
            else 
            {
                // we are finished moving, so stop and stash our speed incase we are retriggered.
                c.prevPortaSpeed = c.portaSpeed;
                c.portaSpeed = 0; // we are done.
            }

            // update date the freq.
            c.freq = Channel::amigaPeriodToHz(c.amigaPeriod);
        }
    }
}

 size_t Mod::makeAudio(
    std::vector<float>& leftMix,
    std::vector<float>& rightMix,
    float sampleRate, 
    int frameSize)
{
    std::unique_lock<std::mutex> lk(cs);
    int o,f,n;
    int made = 0;
    for(int i = 0; i < nChannels; ++i)
    {
        Channel& c = channels[i];
        made = Max(made, c.makeAudio(leftMix, rightMix, frameSize, sampleRate));
    }

    // Apply master volume.
    float masterVolRecp = (float)1.f / nChannels;
    float* l = leftMix.data();
    float* r = rightMix.data();
    for( int i = 0; i < frameSize; ++i)
    {
        *l++ *= masterVolRecp;
        *r++ *= masterVolRecp;
    }

    return made;
}

static int patestCallback( const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo* timeInfo,
                           PaStreamCallbackFlags statusFlags,
                           void *userData )
{
    /* Cast data passed through stream to our structure. */
    float *out = (float*)outputBuffer;
    (void) inputBuffer; /* Prevent unused variable warning. */

    std::vector<float> leftMix;
    std::vector<float> rightMix;
    leftMix.resize(framesPerBuffer);
    rightMix.resize(framesPerBuffer);
    gMod.makeAudio(leftMix,rightMix, SAMPLE_RATE, framesPerBuffer);
    for( int i=0; i<framesPerBuffer; i++ )
    {
        *out++ = leftMix[i];
        *out++ = rightMix[i];
    }
    return 0;
}

int getChannelCount(const void* mem, size_t size)
{
    if( size <= 1084 ) return 0;
    uint32_t id = *(const uint32_t*)((uint8_t*)mem + 1080);
    static const uint32_t k4Chan = '.K.M';
    static const uint32_t k6Chan = 'NHC6';
    static const uint32_t k8Chan = 'NHC8';
    
    switch(id)
    {
        case k4Chan: return 4;
        case k6Chan: return 6;
        case k8Chan: return 8;
    }
    return 0;
}

bool loadMod(void* mem, size_t size)
{
    // channels.
    int nChannels = getChannelCount(mem,size);
    if(!nChannels) return false; // valid?
    gMod.nChannels = nChannels;
    for(int i = 0; i < gMod.nChannels; i++)
        gMod.channels.push_back(std::move(Channel()));

    // name.    
    char name[21] = {0};
    memcpy(name, mem, 20);
    gMod.name = name;
    printf("loading '%d' channel mod '%s'\n", nChannels, name);

    // sample info.
    uint8_t* readPtr8 = (uint8_t*)mem+20; 
    gMod.samples.resize(32);
    for(int i = 1; i <= 31; ++i)
    {
        Sample s;

        // name
        char name[23] = {0};
        memcpy(name, readPtr8, 22);
        s.name = name;
        readPtr8 += 22;

        // length.
        uint16_t packed = *(uint16_t*)readPtr8;        
        s.length = (((packed & 0xff) * 0x100) + (packed>>8)) * 2;
        readPtr8+=2;

        // fine tune.
        s.fine_tune = *(uint8_t*)readPtr8;
        if(s.fine_tune > 7) s.fine_tune -= 16;
        readPtr8+=1;

        // volume.
        s.volume = *(uint8_t*)readPtr8;
        readPtr8+=1; 

        // loop start.
        packed = *(uint16_t*)readPtr8;   
        s.loop_start = (((packed & 0xff) * 0x100) + (packed>>8)) * 2;
        readPtr8+=2;

        // loop length
        packed = *(uint16_t*)readPtr8; 
        uint8_t byte1 = (packed >> 0) & 0xff;
        uint8_t byte2 = (packed >> 8) & 0xff;
        s.loop_length = ((int)(byte1*0x100) + byte2);
        if( s.loop_length == 1) s.loop_length = 0; // 1 means non-looping.
        else s.loop_length *= 2;
        readPtr8+=2; 
        
        if( s.length )
        {
            printf("%d, name='%s', length=0x%x, finetune=%d, vol=%d, loopstart=%x, looplength=%x\n", 
                i, name, (int)s.length, s.fine_tune, s.volume, s.loop_start, s.loop_length);
        }
        gMod.samples[i] = std::move(s);
    }

    // song length
    gMod.songLength = *(uint8_t*)readPtr8;
    readPtr8+=1;
    
    // Skip unused.
    readPtr8+=1;
    
    // Order table.
    gMod.order.resize(gMod.songLength);
    for(int i = 0; i < 128; ++i)
    {
       uint8_t order = *(uint8_t*)readPtr8;
       if( order > gMod.nPatterns) gMod.nPatterns = order;
       readPtr8++;
       if( i < gMod.songLength )
       {
           gMod.order[i] = order;
           printf("%02x, ",order);
       }
    }
    printf("\nLength=%02x, nPatterns=%02x\n", gMod.songLength, gMod.nPatterns);
    assert((uintptr_t)readPtr8 - (uintptr_t)mem == 1080);

    //- read 4 bytes, discard them (we are at position 1080 again, this is M.K. etc!)
    readPtr8+=4;

    size_t patternDataSize = 64 * gMod.nChannels * sizeof(Note) * (gMod.nPatterns+1);
    gMod.patternData = (Note*)malloc(patternDataSize);

    for(int i = 0; i< gMod.nPatterns; ++i)
    {
        Note* note = &gMod.patternData[(gMod.nChannels * 64 * i)];
        printf("\npattern=%02x", i);
        for(int j = 0; j < gMod.nChannels * 64; j++)
        {
            uint16_t byte0 = *readPtr8++;
            uint16_t byte1 = *readPtr8++;
            uint16_t byte2 = *readPtr8++;
            uint16_t byte3 = *readPtr8++;

            // period to note
            int period = ((byte0 & 0x0F) << 8) | byte1;
            note->noteOffset = findOffsetFromPeriod(period);  
            note->sampleNumber = (byte0 & 0xF0) | (byte2 >> 4);
            note->effect = byte2 & 0x0F;
            note->eparm = byte3;

            prettyPrintNote(note, j % nChannels, nChannels, j / nChannels );
            note++;  
        }
    }
    printf("\n");

    int8_t* samples = (int8_t*)mem+size;

    // Load sample data.
    for(int i = 31; i >=1; --i)
    {
        Sample& s = gMod.samples[i];
        if( s.length > 0 )
        {
            samples -= s.length;

            s.data = (int8_t*)malloc(s.length);
            memcpy(s.data, samples, s.length);
/*
            std::stringstream ss;
            ss << "smp_" << i << ".raw";
            if( FILE* fp = fopen(ss.str().c_str(), "wb"))
            {
                fwrite(s.data, s.length, 1, fp);
                fclose(fp);
            }
            */
        }
    }

    // good.
    return true;
}

bool loadMod(const char* filename)
{
    if( FILE* fp = fopen(filename, "rb") )
    {
        fseek(fp,  0, SEEK_END);
        size_t size = ftell(fp);
        void* mem = malloc(size);
        fseek(fp,0, SEEK_SET);
        fread(mem, size, 1, fp);
        bool bSuccess = loadMod(mem, size);
        free(mem);
        return bSuccess;
    }
    return false;
}

int main(int argc, char** argv)
{
    if( argc <= 1 )
    {
        printf("please supply a mod filename.\n");
        return -1;
    }
    if( !loadMod(argv[1]))
    {
        printf("failed to load mod file '%s'\n", argv[1]);
        return -1;
    }

    PaError err,result;
    int line;
  
    PaStream *stream = 0;

    

    ERR_WRAP(Pa_Initialize());

    /* Open an audio I/O stream. */
    ERR_WRAP(Pa_OpenDefaultStream( &stream,
                                0,          /* no input channels */
                                2,          /* stereo output */
                                paFloat32,  /* 32 bit floating point output */
                                SAMPLE_RATE,
                                //1024,
                                paFramesPerBufferUnspecified,        
                                                    /* frames per buffer, i.e. the number
                                                   of sample frames that PortAudio will
                                                   request from the callback. Many apps
                                                   may want to use
                                                   paFramesPerBufferUnspecified, which
                                                   tells PortAudio to pick the best,
                                                   possibly changing, buffer size.*/
                                patestCallback, /* this is your callback function */
                                &gMod )); /*This is a pointer that will be passed to
                                                   your callback*/

    ERR_WRAP(Pa_StartStream( stream ));
    timer_start([&]{gMod.tick();}, 18);

    Pa_Sleep(NUM_SECONDS*100000);

    ERR_WRAP(Pa_StopStream( stream ));
    ERR_WRAP(Pa_Terminate());
    return 0;

error:
    if( stream )
    {
        if( !Pa_IsStreamStopped (stream)) 
            Pa_StopStream(stream);

        Pa_CloseStream(stream);
        stream = nullptr;
    }
    printf(  "PortAudio error: %s\n", Pa_GetErrorText( err ) );
    return -1;
}

//
const uint16_t gNotes[12*3*16] = 
    {
        // Tuning -8
        907,856,808,762,720,678,640,604,570,538,508,480
        ,453,428,404,381,360,339,320,302,285,269,254,240
        ,226,214,202,190,180,170,160,151,143,135,127,120
        // Tuning -7
        ,900,850,802,757,715,675,636,601,567,535,505,477
        ,450,425,401,379,357,337,318,300,284,268,253,238
        ,225,212,200,189,179,169,159,150,142,134,126,119
        //; Tuning -6
        ,894,844,796,752,709,670,632,597,563,532,502,474
        ,447,422,398,376,355,335,316,298,282,266,251,237
        ,223,211,199,188,177,167,158,149,141,133,125,118
        // Tuning -5
        ,887,838,791,746,704,665,628,592,559,528,498,470
        ,444,419,395,373,352,332,314,296,280,264,249,235
        ,222,209,198,187,176,166,157,148,140,132,125,118
        // Tuning -4
        ,881,832,785,741,699,660,623,588,555,524,494,467
        ,441,416,392,370,350,330,312,294,278,262,247,233
        ,220,208,196,185,175,165,156,147,139,131,123,117
        // Tuning -3
        ,875,826,779,736,694,655,619,584,551,520,491,463
        ,437,413,390,368,347,328,309,292,276,260,245,232
        ,219,206,195,184,174,164,155,146,138,130,123,116
        // Tuning -2
        ,868,820,774,730,689,651,614,580,547,516,487,460
        ,434,410,387,365,345,325,307,290,274,258,244,230
        ,217,205,193,183,172,163,154,145,137,129,122,115
        // Tuning -1
        ,862,814,768,725,684,646,610,575,543,513,484,457
        ,431,407,384,363,342,323,305,288,272,256,242,228
        ,216,203,192,181,171,161,152,144,136,128,121,114
        // normal tuning.
        ,856,808,762,720,678,640,604,570,538,508,480,453 //; C-1 to B-1
        ,428,404,381,360,339,320,302,285,269,254,240,226 // ; C-2 to B-2
        ,214,202,190,180,170,160,151,143,135,127,120,113 // ; C-3 to B-3
        // Tuning 1
        ,850,802,757,715,674,637,601,567,535,505,477,450 //; same as above
        ,425,401,379,357,337,318,300,284,268,253,239,225 //; but with 
        ,213,201,189,179,169,159,150,142,134,126,119,113 //; finetune +1
        // Tuning 2
        ,844,796,752,709,670,632,597,563,532,502,474,447 //; etc, 
        ,422,398,376,355,335,316,298,282,266,251,237,224 //; finetune +2
        ,211,199,188,177,167,158,149,141,133,125,118,112 //
        // Tuning 3
        ,838,791,746,704,665,628,592,559,528,498,470,444
        ,419,395,373,352,332,314,296,280,264,249,235,222
        ,209,198,187,176,166,157,148,140,132,125,118,111
        // Tuning 4
        ,832,785,741,699,660,623,588,555,524,495,467,441
        ,416,392,370,350,330,312,294,278,262,247,233,220
        ,208,196,185,175,165,156,147,139,131,124,117,110
        // Tuning 5
        ,826,779,736,694,655,619,584,551,520,491,463,437
        ,413,390,368,347,328,309,292,276,260,245,232,219
        ,206,195,184,174,164,155,146,138,130,123,116,109
        // Tuning 6
        ,820,774,730,689,651,614,580,547,516,487,460,434
        ,410,387,365,345,325,307,290,274,258,244,230,217
        ,205,193,183,172,163,154,145,137,129,122,115,109
        // Tuning 7
        ,814,768,725,684,646,610,575,543,513,484,457,431
        ,407,384,363,342,323,305,288,272,256,242,228,216
        ,204,192,181,171,161,152,144,136,128,121,114,108
    };
