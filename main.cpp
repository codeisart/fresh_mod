#include "portaudio.h"
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <assert.h>
#include <thread>         // std::thread
#include <condition_variable>
#include <atomic>
#include <chrono>

template<typename T> T Max(T a,T b) { return a > b ? a : b; }
template<typename T> T Min(T a,T b) { return a < b ? a : b; }

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

void producerThread()
{
    printf("Starting worker thread.\n");

    while(!gExit)
    {
        {
            std::unique_lock<std::mutex> lk(m);
            auto now = std::chrono::system_clock::now();
            gWakeCv.wait_until(lk,now + std::chrono::milliseconds(1000), []() -> bool { return gWake; } );
        }
        printf("Waking up..\n");
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
    
    for( i=0; i<framesPerBuffer; i++ )
    {
        if( buff->num() >= 2)
        {
            *out++ = buff->pop();
            *out++ = buff->pop(); 
        }
        else
        {
            *out++ = 0.0f;
            *out++ = 0.0f; 
        }
    }
    return 0;
}

#define ERR_WRAP(mac_err) do { result = mac_err ; line = __LINE__ ; if ( result != paNoError ) goto error; } while(0)
#define SAMPLE_RATE (44100)
#define NUM_SECONDS (10)

int main(int argc, char** argv)
{
    std::thread r(producerThread);
    r.detach();

    //testCircbuffer();
    //return 0;

    PaError err,result;
    int line;
    int sampleCount = 44000*10;
    RingBuffer<float> output(sampleCount);
    //CircBuff output(sampleCount * sizeof(float) * 2);    // 512 samples * 2 channels * sizeof(float).
    PaStream *stream = 0;

    float phase = 0.0f;
    for( int i = 0; i < sampleCount; ++i)
    {
        phase += 0.03f;
        if( phase >= 1.0f ) 
            phase -= 2.0f;
               
        output.push(phase);
        output.push(phase);
    }

    ERR_WRAP(Pa_Initialize());

    /* Open an audio I/O stream. */
    ERR_WRAP(Pa_OpenDefaultStream( &stream,
                                0,          /* no input channels */
                                2,          /* stereo output */
                                paFloat32,  /* 32 bit floating point output */
                                SAMPLE_RATE,
                                256,        /* frames per buffer, i.e. the number
                                                   of sample frames that PortAudio will
                                                   request from the callback. Many apps
                                                   may want to use
                                                   paFramesPerBufferUnspecified, which
                                                   tells PortAudio to pick the best,
                                                   possibly changing, buffer size.*/
                                patestCallback, /* this is your callback function */
                                &output )); /*This is a pointer that will be passed to
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