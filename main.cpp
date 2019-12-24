#include "portaudio.h"
#include <stdio.h>

int Callback( const void *input, void *output,
    unsigned long frameCount,
    const PaStreamCallbackTimeInfo* timeInfo,
    PaStreamCallbackFlags statusFlags,
    void *userData )
{
    return 0;
}

#define ERR_WRAP(mac_err) do { result = mac_err ; line = __LINE__ ; if ( result != paNoError ) goto error; } while(0)
#define SAMPLE_RATE (44100)
#define NUM_SECONDS (10)

typedef struct
{
    float left_phase;
    float right_phase;
}   
paTestData;

int main(int argc, char** argv)
{
    PaError err,result;
    int line;
    paTestData data = {0};
    PaStream *stream = 0;

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
                                Callback, /* this is your callback function */
                                &data )); /*This is a pointer that will be passed to
                                                   your callback*/

    ERR_WRAP(Pa_StartStream( stream ));

    Pa_Sleep(NUM_SECONDS*1000);

    ERR_WRAP(Pa_StopStream( stream ));

    ERR_WRAP(Pa_Terminate());
    return 0;

error:
    if( stream )
    {
        if( !Pa_IsStreamStopped (stream)) 
            Pa_StopStream(stream);

        Pa_CloseStream(stream);
        stream = 0;
    }
    printf(  "PortAudio error: %s\n", Pa_GetErrorText( err ) );
    return -1;
}