#include <assert.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fftw3.h>
#include <functional>
#include <immintrin.h>
#include <map>
#include <memory.h>
#include <ncurses.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <tgmath.h>
#include <thread>         // std::thread
#include <unistd.h>
#include <vector>

#include "plm.h"
#include "mod.h"
#include "portaudio.h"
#include "util.h"

#define WITH_DISPLAY (1)

extern const uint16_t gNotes[];
extern const uint8_t sine_table[32];
Mod gMod;

// mix buffers.
std::mutex gMixCs;
std::vector<float> gLeftMix;
std::vector<float> gRightMix;

// fftw
double* gInBuff=nullptr;
fftw_complex* gOutBuff=nullptr;
float* gPowerSpectrum = nullptr;
fftw_plan gPlan;

void initFft(int bufSize)
{
    gInBuff = (double*)fftw_malloc(sizeof(double)*bufSize);
    gOutBuff = fftw_alloc_complex(bufSize/2+1);;
    gPlan = fftw_plan_dft_r2c_1d(bufSize, gInBuff, gOutBuff, FFTW_ESTIMATE);
    if(!gPlan){
	printf("fuck\n");
	exit(0);
    }
    gPowerSpectrum = (float*)fftw_malloc(sizeof(float)*bufSize);
}
void deinitFft()
{   
    fftw_destroy_plan(gPlan);
    fftw_free(gInBuff); gInBuff=nullptr;
    fftw_free(gOutBuff); gOutBuff=nullptr;
    fftw_free(gPowerSpectrum); gPowerSpectrum=nullptr;
}


#define SAMPLE_RATE (48000)
#define NUM_SECONDS (60)

static const uint32_t kFrameSize=512;



static int patestCallback( const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo* timeInfo,
                           PaStreamCallbackFlags statusFlags,
                           void *userData )
{
    std::unique_lock<std::mutex> lk(gMixCs);
    /* Cast data passed through stream to our structure. */
    float *out = (float*)outputBuffer;
    (void) inputBuffer; /* Prevent unused variable warning. */

    gLeftMix.resize(framesPerBuffer);
    gRightMix.resize(framesPerBuffer);
    gMod.makeAudio(gLeftMix,gRightMix, SAMPLE_RATE, framesPerBuffer);
    float summedPow = 0;
    const float* leftMix = gLeftMix.data();
    const float* rightMix = gRightMix.data();
    for( int i=0; i<framesPerBuffer; i++ )
    {
        *out++ = *leftMix++;
        *out++ = *rightMix++;
    }

#if WITH_DISPLAY

    // fft.
    for( int i=0;i < framesPerBuffer; ++i)
    {
    	gInBuff[i] = gLeftMix[i];
    }
    // han window.
    for( int i=0;i < framesPerBuffer; ++i)
    {
    	float multiplier = 0.5f * (1.f - cosf((float)2.f*M_PI*i/framesPerBuffer));
    	gInBuff[i]*=multiplier;
    }
    
    fftw_execute(gPlan);

    gPowerSpectrum[0] = gOutBuff[0][0]*gOutBuff[0][0];
    for( int i=1; i< (framesPerBuffer+1)/2; i++)
    {
        float rl =gOutBuff[i][0];
    	float im =gOutBuff[i][1];
    	gPowerSpectrum[i] = (rl*rl) + (im*im);
    }
    gPowerSpectrum[framesPerBuffer/2] = gOutBuff[framesPerBuffer/2][0]*gOutBuff[framesPerBuffer][0];

#endif //WITH_DISPLAY

    return 0;
}



void initGfx()
{
    initscr();
    cbreak();
    nodelay(stdscr,TRUE);
    nonl();
    intrflush(stdscr,TRUE);
    curs_set(FALSE);
}
void deinitGfx()
{
    endwin();    
}



void inputLoop()
{
    bool bQuit = false;
    while(!bQuit)
    {
        int c = getch();
        if (c == 'q')
            bQuit = true;
        else if (c >= '0' && c <= '9')
            devSoloChannel = c - '0';
        else if (c == 's')
            devSoloChannel = -1;

        // draw vu
        clear();
        int y, x;
        getmaxyx(stdscr, y, x);

        std::vector<float> l, r;
        {
            std::unique_lock<std::mutex> lk(gMixCs);
            l = std::move(gLeftMix);
            r = std::move(gRightMix);
        }
        if (r.size() > 0)
        {
            int mid = y / 2;
            float pos = 0;
            float step = x > 0 ? (float)(l.size() - 1) / x : 0;
            for (int i = 0; i < x; ++i, pos += step)
            {
                //fprintf(stderr, "pos=%d, %f\n", (int)pos, pos);
                pos = clamp(0.0f, (float)l.size() - 1, pos);
                float s = r[(int)pos];
                s = fabs(s);
                int starty = (float)mid * s;
                //mvvline(mid-starty,i, 0, starty*2);
            }

            for (int i = 0; i < gMod.nChannels; ++i)
            {
                float nvu = fabs(norm(-96.f, 0, gMod.channels[i].vuDb));
                float wid = (float)(1.f - nvu) * x;
                mvhline(i, 0, 0, wid);
                //float wid = (float) (1.f - nvu) * y;
                //mvvline(y-wid, i, 0, wid);
            }

            for (int i = 0; i < x; ++i)
            {
                float posx = ((float)i / x) * (float)(kFrameSize / 4 + 1);
                float p = gPowerSpectrum[(int)posx];
                float pl = linearToDb(p);
                //float nvu = norm(-96,0,pl);
                float nvu = fabs(norm(0, 100, pl));
                //float wid = (float) (1.f-nvu) * (y/4);
                float wid = (float)(nvu)*y;
                mvvline(y - wid, i, 0, wid);
            }
        }

        usleep(30000);
        refresh();
    }
}

bool loadModule(const std::string& filename)
{   
    if(ends_with(filename, ".mod"))
        return loadMod(filename.c_str());
    else if(ends_with(filename, ".plm"))
        return loadPlm(filename.c_str());
    
    return false;
}

int main(int argc, char** argv)
{
    if( argc <= 1 )
    {
        printf("please supply a mod filename.\n");
        return -1;
    }
    if( !loadModule(argv[1]))
    {
        printf("failed to load file '%s'\n", argv[1]);
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
                                kFrameSize,
                                //paFramesPerBufferUnspecified,        
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

#if WITH_DISPLAY
    initFft(kFrameSize);
    initGfx();
    clear();
    refresh();
#endif //WITH_DISPLAY

    inputLoop();
    
    //Pa_Sleep(NUM_SECONDS*100001);

    ERR_WRAP(Pa_StopStream( stream ));
    ERR_WRAP(Pa_Terminate());

#if WITH_DISPLAY
    deinitGfx();
    deinitFft();
#endif //WITH_DISPLAY

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