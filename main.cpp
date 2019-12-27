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
#include <map>

uint16_t findNoteFromFreq(uint16_t amigaFreq, int& note, int& oct, int& ft, int tolerance=1);

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
struct Note
{
    uint16_t note:11;  // 0-??  = 11 bits = 0-2048 should be plenty for your needs.
    uint8_t number:5;  // 0-31  = 5 bits
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
    Note* patternData = nullptr;
} gMod;

template<typename T> T Max(T a,T b) { return a > b ? a : b; }
template<typename T> T Min(T a,T b) { return a < b ? a : b; }
template<typename T> T Abs(T v) { return v < 0 ? -v : v; }

template<typename T>
struct RingBuffer
{
    int head, tail, size;
    T* buff;
    RingBuffer(int InSize) : buff(new T[InSize]), head(0), tail(0), size(InSize) {}
    ~RingBuffer() { delete[] buff; }
    int increment(int i, int amnt=1) { return (i+amnt)%size; }  
    bool push(const T& v) 
    { 
        int next = increment(head);
        if( next == tail) return false; // overflow
        buff[head] = v; 
        head=next;
        return true;
    }  
    T pop() { T v = buff[tail]; tail=increment(tail); return v; }
    int num() const { return head >= tail ? head-tail : (head+size)-tail; }
    int space() const { return size - num(); }
};

void testCircbuffer()
{
    // test 1. write 
    {
        //CircBuff a(100 * sizeof(float));
        RingBuffer<float> a(2);
        assert(a.num() == 0);
        a.push(0.12345f); 
        assert(a.num() == 1);
        float tmp = a.pop();
        assert(tmp == 0.12345f);
    }
    // test 2. write a bunch.
    {
       RingBuffer<float> a(8);
       for(int i = 0; i < 200; ++i)
        {
            assert(a.num() == 0);
            a.push(0.12345f);
            assert(a.num() == 1);
            float tmp = a.pop();
            assert(tmp == 0.12345f);
        } 
    }

    // test 3. 
    {
        RingBuffer<float> a(128);
        while(a.space())
        {
            a.push(0.12345f); 
        } 
        while(a.num())
        {
            float tmp = a.pop();
            assert(tmp == 0.12345f); 
        }
    }

     // test 4. 
    {
        RingBuffer<float> a(128);
        for( int i = 0; i < 64; ++i )
        {
            a.push((float)i);    
            assert(a.num() == i);
        }
        for( int i=0; i < 64; ++i)
        {
            float tmp = a.pop();
            assert(tmp == (float)i); 
        }
    }
}

static std::atomic<bool> gExit;
static std::atomic<bool> gWake;
static std::condition_variable gWakeCv;
static std::mutex m;
RingBuffer<float> gOutput(1024*2);

void producerThread()
{
    printf("Starting worker thread.\n");

    int currentSample = 0;
    int samplePos = 0;
    float leftPhase = 0.0f;
    float rightPhase = 0.0f;
    while(!gExit)
    {
        {
            std::unique_lock<std::mutex> lk(m);
            auto now = std::chrono::system_clock::now();
            if( !gWakeCv.wait_until(lk,now + std::chrono::milliseconds(200), []() -> bool { return gWake; } ) )
                printf("Wait timeout...\n");
        }
        //printf("Waking up...\n");
        if(gMod.samples.size() < 31) continue;

        int samplesProduced = 0;
        while(gOutput.space() > 0)
        {
            Sample& s = gMod.samples[currentSample];

            for( ; samplePos < s.length; ++samplePos)
            {
                float val = (float)(s.data[samplePos] / 256.f);
                if( !gOutput.push(val))
                    break;
                if( !gOutput.push(val))
                    break;
                samplesProduced+=2;
            }
            if( samplePos >= s.length)
            {
                samplePos = 0;
                currentSample = (currentSample +1) % 31;
                printf("sample=%d\n", currentSample);
            }
        }

        // produce some samples...
        /*
        for( int i = 0; i < 1024; ++i)
        {
            leftPhase += 0.03f;
            if( leftPhase >= 1.0f ) leftPhase -= 2.0f;
            rightPhase += 0.01f;
            if( rightPhase >= 1.0f ) rightPhase -= 2.0f;
                
            if(!gOutput.push(leftPhase)) break;
            if(!gOutput.push(rightPhase)) break;
        }*/
        gWake = false;
    }

    printf("Exiting worker thread.\n");
}


/* This routine will be called by the PortAudio engine when audio is needed.
   It may called at interrupt level on some machines so don't do anything
   that could mess up the system like calling malloc() or free().
*/ 
static int patestCallback( const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo* timeInfo,
                           PaStreamCallbackFlags statusFlags,
                           void *userData )
{
    /* Cast data passed through stream to our structure. */
    //CircBuff *buff = (CircBuff*)userData; 
    RingBuffer<float>* buff = (RingBuffer<float>*)userData;
    float *out = (float*)outputBuffer;
    unsigned int i;
    (void) inputBuffer; /* Prevent unused variable warning. */
    
    int starvedSamples = 0;
    for( i=0; i<framesPerBuffer; i++ )
    {
        if( buff->num() >= 2)
        {
            *out++ = buff->pop();
            *out++ = buff->pop(); 
        }
        else
        {
            starvedSamples+=2;
            *out++ = 0.0f;
            *out++ = 0.0f; 
        }
    }

    if( starvedSamples)
    {
        printf("We starved for %d samples\n", starvedSamples);
    }

    // Wake producer thread.
    gWake = true;
    gWakeCv.notify_all();
    return 0;
}

#define ERR_WRAP(mac_err) do { result = mac_err ; line = __LINE__ ; if ( result != paNoError ) goto error; } while(0)
#define SAMPLE_RATE (22000)
#define NUM_SECONDS (10)


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

    // name.    
    char name[21] = {0};
    memcpy(name, mem, 20);
    gMod.name = name;
    printf("loading '%d' channel mod '%s'\n", nChannels, name);

    // sample info.
    uint8_t* sampleInfoState = (uint8_t*)mem+20; 
    for(int i = 0; i < 31; ++i)
    {
        Sample s;

        // name
        char name[23] = {0};
        memcpy(name, sampleInfoState, 22);
        s.name = name;
        sampleInfoState += 22;

        // length.
        uint16_t packed = *(uint16_t*)sampleInfoState;        
        s.length = (((packed & 0xff) * 0x100) + (packed>>8)) * 2;
        sampleInfoState+=2;

        // fine tune.
        s.fine_tune = *(uint8_t*)sampleInfoState;
        if(s.fine_tune > 7) s.fine_tune -= 16;
        sampleInfoState+=1;

        // volume.
        s.volume = *(uint8_t*)sampleInfoState;
        sampleInfoState+=1; 

        // loop start.
        packed = *(uint16_t*)sampleInfoState;   
        s.loop_start = (((packed & 0xff) * 0x100) + (packed>>8)) * 2;
        sampleInfoState+=2;

        // loop length
        packed = *(uint16_t*)sampleInfoState;   
        s.loop_length = (((packed & 0xff) * 0x100) + (packed>>8)) * 2;
        sampleInfoState+=2; 
        
        if( s.length )
        {
            printf("%d, name='%s', length=0x%x, finetune=%d, vol=%d, loopstart=%d, looplength=%d\n", 
                i, name, (int)s.length, s.fine_tune, s.volume, s.loop_start, s.loop_length);
        }
        gMod.samples.push_back(std::move(s));
    }

    // song length
    gMod.songLength = *(uint8_t*)sampleInfoState;
    sampleInfoState+=1;
    
    // Skip unused.
    sampleInfoState+=1;

    // Order table.
    for(int i = 0; i < 128; ++i)
    {
       uint8_t order = *(uint8_t*)sampleInfoState;
       if( order > gMod.nPatterns) gMod.nPatterns = order;
       gMod.order.push_back(order);
       sampleInfoState++;
       printf("order=%d -> %u\n", i, order);
    }
    printf("nPatterns=%d\n", gMod.nPatterns);
    assert((uintptr_t)sampleInfoState - (uintptr_t)mem == 1080);

    //- read 4 bytes, discard them (we are at position 1080 again, this is M.K. etc!)
    sampleInfoState+=4;

    gMod.patternData = (Note*)malloc(64 * gMod.nChannels * sizeof(Note) * (gMod.nPatterns+1));

    uint32_t* patterns = (uint32_t*)sampleInfoState;
    for(int i = 0; i< gMod.nPatterns; ++i)
    {
        Note* note = &gMod.patternData[(gMod.nChannels * 64 * i)];
        for(int j = 0; j < gMod.nChannels * 64; j++)
        {
            uint32_t packed = *patterns++;
            //store SAMPLE_NUMBER as    (byte0 AND 0F0h) + (byte2 SHR 4)
            uint8_t byte0 = packed & 0xff;
            uint8_t byte2 = (packed >> 16) & 0xff;
            note->number = (byte0 & 0xf0) + (byte2 >> 4);

            //- store PERIOD_FREQUENCY as ((byte0 AND 0Fh) SHL 8) + byte1;
            uint32_t freq = ((packed & 0xf) << 8) + ((packed >> 8) & 0xff);
            int oct, semi, ft;
            note->note = findNoteFromFreq(freq, semi, oct, ft);
            
            //- store EFFECT_NUMBER as    byte2 AND 0Fh
            note->effect = (packed >> 16) & 0xf;

            //- store EFFECT_PARAMETER as byte 3
            note->eparm = (packed >> 24) & 0xff;
            const char* notestr[] = {"C-", "C#", "D-", "D#","E-", "F-", "F#", "G-","G#","A-", "A#", "B-" };

            if( j % gMod.nChannels == 0 )
            {
                printf("\n");
                printf("%02x    ", j / 4);
            }
            // note.
            if( note->note) printf("%s%d ", notestr[semi],oct);
            else printf("... ");
            // sample #
            if( note->number ) printf("%02x ", note->number);
            else printf(".. ");
            // effect.
            printf("%x%02x", note->effect, note->eparm);
            printf("     ");

            note++;  
        }
    }

    // Load sample data.
    uint8_t* sampleData = (uint8_t*)patterns;
    for(int i = 0; i < 31; ++i)
    {
        Sample& s = gMod.samples[i];
        if( s.length > 0 )
        {
            s.data = (int8_t*)malloc(s.length);
            memcpy(s.data, sampleData, s.length);
            sampleData += s.length;
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

    std::thread r(producerThread);
    r.detach();

    //testCircbuffer();
    //return 0;

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
                                paFramesPerBufferUnspecified,        
                                                    /* frames per buffer, i.e. the number
                                                   of sample frames that PortAudio will
                                                   request from the callback. Many apps
                                                   may want to use
                                                   paFramesPerBufferUnspecified, which
                                                   tells PortAudio to pick the best,
                                                   possibly changing, buffer size.*/
                                patestCallback, /* this is your callback function */
                                &gOutput )); /*This is a pointer that will be passed to
                                                   your callback*/

    ERR_WRAP(Pa_StartStream( stream ));

    Pa_Sleep(NUM_SECONDS*1000);

    ERR_WRAP(Pa_StopStream( stream ));

    ERR_WRAP(Pa_Terminate());
    {
        gWake = true;
        gExit = true;
        gWakeCv.notify_all();
    }
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
static uint16_t gNotes[12*3*16] = 
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

uint16_t findNoteFromFreq(uint16_t amigaFreq, int& note, int& oct, int& ft, int tolerance)
{    
    for(oct=0; oct < 3; ++oct)
    {
        for(note=0; note < 12; note++ )
        {
            for(ft=0; ft < 16; ft++)
            {
                uint16_t off = (12*3*ft)+(12*oct)+note;
                uint16_t val = gNotes[off];
                int diff = (int)amigaFreq-val;
                if( Abs(diff) <= tolerance)
                {
                    oct+=3;
                    ft -=8;
                    return off;
                }
            }
        }
    }
    return 0; // fail.
}