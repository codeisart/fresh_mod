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

#include "mod.h"
#include "portaudio.h"
#include "util.h"

#define WITH_DISPLAY (0)

static const float s8ToFloatRecp = 1.f / 128.f;
static const float vol64FloatRecp = 1.f / 64.f;
static const char* notestr[] = {"C-", "C#", "D-", "D#","E-", "F-", "F#", "G-","G#","A-", "A#", "B-" };
extern const uint16_t gNotes[];
extern const uint8_t sine_table[32];
Mod gMod;

// mix buffers.
std::mutex gMixCs;
std::vector<float> gLeftMix;
std::vector<float> gRightMix;

// solo for debugging.
int devSoloChannel = -1; 
int devSoloPattern = -1; 

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

// This is a fast approximation to log2()
// Y = C[0]*F*F*F + C[1]*F*F + C[2]*F + C[3] + E;
float log2f_approx(float X) {
  float Y, F;
  int E;
  F = frexpf(fabsf(X), &E);
  Y = 1.23149591368684f;
  Y *= F;
  Y += -4.11852516267426f;
  Y *= F;
  Y += 6.02197014179219f;
  Y *= F;
  Y += -3.13396450166353f;
  Y += E;
  return(Y);
}

//log10f is exactly log2(x)/log2(10.0f)
#define log10f_fast(x)  (log2f_approx(x)*0.3010299956639812f)

float linearToDb(float f)
{
    static const float kSmallNumber = 1.e-8f;
    if( f > kSmallNumber )
    	return 20.f * log10f_fast(f); //log10f(f);
    
    return -96.f;
}
template<typename T> T clamp(T mn, T mx, T v) { return v <  mn ? mn : v > mx ? mx : v; }

float lerp( float s1, float s2, float t )
{
    float d = s2-s1;
    float l = s1 + (d*t);    
    return l;
}

int32_t lerpFixed( int32_t s1, int32_t s2, uint32_t t)
{
    int32_t d = (s2-s1) << 16;
    int32_t p = ((int64_t)(d*t) / (1 << 16)); 
    int32_t l = s1 + (p / (1 << 16));
    return l;
    // fp_fract 0...65535 (0...100)
    //uint32_t (65535 - fp_fract);
}

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
    printf("%x%02x", (int)note->effect, note->eparm);
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

void Channel::setFineTune(int ftune)
{
    ft = ftune;
}
void Channel::setAmigaPeriod(int ap)
{
    amigaPeriod = ap;
}
void Channel::setVol(int v)
{
    vol = v;
}

void Channel::setSample(Sample* s)
{
    sample = s;
    samplePos = 0;
    ft = sample->fine_tune;

    if( amigaPeriod <= 0)
        return; 
}

// render thread.
int Channel::makeAudio(
    std::vector<float>& leftMix,
    std::vector<float>& rightMix,
    int blockSize,
    uint32_t sampleRate)
{
    // testing voice mgr.
    //return 0;

    if( amigaPeriod <= 0)
        return 0;
    if( !sample )
        return 0;
    if( samplePos == -1.f)
        return 0;

    uint32_t freq = amigaPeriodToHz(amigaPeriod);
    //uint32_t srcStep = (uint64_t)(freq << 16) / sampleRate;
    float srcStep = freq / (double)sampleRate;

    float end = sample->loop_length > 0 ?
                sample->loop_start + sample->loop_length :
                sample->length;

    float* left = leftMix.data();
    float* right = rightMix.data();
    float fltVol = (float)vol * vol64FloatRecp;
    float fltPan = ((float)(pan + 128)/256.f) - 0.5f;
    bool bLooping = sample->loop_length > 0; 

    // square law panning.
    float sqrPowLeft = fltPan+1.f;
    float gainLeft = 1.f - 0.25f * (sqrPowLeft*sqrPowLeft);
    float sqrPowRight = 0.5f*fltPan-0.5f;
    float gainRight = 1.f - (sqrPowRight*sqrPowRight);

    int i = 0; 
    float sum = 0.f;
    while( i < blockSize )
    {
        // handle end of sample.
        if( samplePos >= end )
        {
            // looping?
            if( bLooping )
                samplePos = sample->loop_start;
            else 
            {
                samplePos = -1;
                break;
            }
        }

        // Break out fractional+int parts
        double intPos = 0.f;
        double fractPos = modf(samplePos, &intPos);
        int samplePosInt = (int)intPos;
        int nextSamplePos = samplePosInt+1;

        // calculate next sample.
        if( nextSamplePos >= end)
        {
            if(bLooping)
                nextSamplePos = sample->loop_start;   
            else
                nextSamplePos = samplePosInt;
        }

        float s1 = (float) sample->data[samplePosInt];
        float s2 = (float) sample->data[nextSamplePos];
        float s = lerp(s1, s2, fractPos);
    
        float ls = s1 * gainLeft * fltVol;
        float rs = s1 * gainRight * fltVol;

        (*left++) += ls;
        (*right++) += rs;
        
	    sum +=s*s;
        samplePos += srcStep;  
        ++i;
    }

    float avgPow = (float)sum / blockSize;
    vuDb = linearToDb(avgPow);
    return i;
}

void Mod::tick()
{
    currentTick++;
    if (currentTick >= currentSpeed)
    {
        std::unique_lock<std::mutex> lk(cs);
        if( patternDelay <= 0)
        {
            updateRow();
            currentTick = 0;
            currentRow++;
            if (currentRow >= 64)
            {
                currentRow = 0;
                currentOrder++;
                if (currentOrder >= songLength)
                {
                    currentOrder = 0;
                }
            }
        }
        else 
            patternDelay--;
    }
    else
    {
        updateEffects();
    }
}

void Mod::startVibrato(Channel& channel, Note& note, bool bKeepOld)
{
    if( !bKeepOld )
    {
        int depth = note.eparm & 0xf;
        int speed = (note.eparm >> 4) & 0xf;
        if( depth ) channel.vibrDepth = depth;
        if( speed ) channel.vibrSpeed = speed;
    }
    channel.vibrTicksLeft = currentSpeed - 1;
}
void Mod::startTremello(Channel& channel, Note& note)
{
    int depth = note.eparm & 0xf;
    int speed = (note.eparm >> 4) & 0xf;
    if (depth)
        channel.tremeDepth = depth;
    if (speed)
        channel.tremeSpeed = speed;
    channel.tremeTicksLeft = currentSpeed - 1;
}
void Mod::startVolSlide(Channel& channel, Note& note)
{
    // vol slide.
    int8_t lowNib = -(note.eparm & 0xf);
    int8_t hiNib = (note.eparm >> 4) & 0xf;
    int speed =  lowNib ? lowNib : hiNib;
    if( speed ) channel.volSlideSpeed = speed;
    channel.volSlideTicksLeft = currentSpeed - 1;
}
void Mod::startPortamento(Channel& channel, Note& note, bool bTargetNote, bool bKeepOld )
{
    if( bTargetNote )
    {
        // new target?
        if (note.noteOffset)
            channel.portaToNoteOffset = note.noteOffset;
    }else
    {
        channel.portaToNoteOffset = 0;
    }
    
    if( !bKeepOld )
    {
        if (note.eparm)
            channel.portaSpeed = note.effect == Effect::Porta_Down ? -note.eparm : note.eparm;
    }
    channel.portaTicksLeft = currentSpeed - 1;
}

void Mod::updateRow() 
{
    int patternIdx = devSoloPattern >= 0 ? 
        devSoloPattern : 
        order[currentOrder];

    //printf("updaterow... ptn=%d, ord=%d, row=%d\n", patternIdx, currentOrder, currentRow );
    for(int channelIdx = 0; channelIdx < nChannels; ++channelIdx)
    {        
        Note& note = patternData[(64*nChannels*patternIdx)+(nChannels*currentRow)+channelIdx];
        Channel& channel = channels[channelIdx];

        bool bDelayNote = note.effect == Effect::Sub_Effect && static_cast<EffectSubType>(note.eparm >> 4) == EffectSubType::Delay_Note;

        prettyPrintNote(&note, channelIdx, nChannels, currentRow);
        if( note.sampleNumber && !bDelayNote)
        {
            Sample* smpl = &samples[note.sampleNumber];
            channel.setSample(smpl);
            channel.setVol(smpl->volume);
            //printf( "chn %d, set sample %d '%s'\n", channelIdx, note.sampleNumber, smpl->name.c_str());  
        }

        if( note.noteOffset != 0 && !bDelayNote) 
        {
           if (note.effect != Effect::Porta_To_Note && 
               note.effect != Effect::Porta_Vol_Slide)  
           {
                if( channel.sample)
                    channel.setFineTune(channel.sample->fine_tune);
                
                int period = Channel::getAmigaFreq(note.noteOffset, channel.ft);
                if( period > 0 )
                {
                    channel.setAmigaPeriod(period);
                    //channel.amigaPeriod = period;

                    // these all reset when a new note is played.
                    channel.vibrInv = false;
                    channel.vibrPos = 0;
                    channel.tremeInv = false;
                    channel.tremePos = 0;
                }
           }
        }

        switch(static_cast<Effect>(note.effect))
        {
        case Effect::Arpeggio:
            if( note.eparm == 0 ) break;
        default:
            printf("effect unhandled %x%02x, row=%d, channel=%d, pattern=%d\n",
                   (int)note.effect, (int)note.eparm, currentRow, channelIdx, patternIdx);
            break;
        case Effect::Tremolo:
            startTremello(channel, note);
            break;
        case Effect::Pan:
            if( note.eparm == 0 )           channel.pan = -128;
            else if( note.eparm == 0x40)    channel.pan = 0;
            else if( note.eparm == 0x80)    channel.pan = 127;
            else if( note.eparm == 0xa4)    channel.bSurround = true;
            break;
        case Effect::Porta_Up: // fall-through.
        case Effect::Porta_Down:
            startPortamento(channel, note, false, false);
            break;
        case Effect::Porta_To_Note:
            startPortamento(channel, note, true, false);
            break;
        case Effect::Porta_Vol_Slide:
        {
            startPortamento(channel, note, true, true );
            startVolSlide(channel, note);    
            break;
        }
        case Effect::Vibrato:
            startVibrato(channel, note, false);
            break;
        case Effect::Vibrato_Vol_Slide:
        {
            startVibrato(channel, note, true);
            startVolSlide(channel, note);    
            break;
        }
        case Effect::Sample_Offset:
            channel.samplePos = (float)((uint32_t)note.eparm * 0x100);
            break;
        case Effect::Volume_Slide:
            startVolSlide(channel, note);
            break;
        case Effect::Jump_To_Pattern:
            currentOrder = note.eparm - 1;
            currentRow = -1;
            break;
        case Effect::Set_Volume:
            channel.setVol(note.eparm);
            break;
        case Effect::Pattern_Break:
        {
            if (note.eparm > 64)
                currentRow = 0;
            else
                currentRow = note.eparm;
            currentOrder++;
            currentRow--; // We are about to increment this, so make it -1 that we want it.
            break;
        }
        case Effect::Sub_Effect:
        {
            int param = note.eparm & 0xff;
            switch(static_cast<EffectSubType>(note.eparm>>4))
            {
            default:
                printf("effect unhandled %x%02x, row=%d, channel=%d, pattern=%d\n", 
                (int)note.effect, note.eparm, currentRow, channelIdx, patternIdx );
                break;
            case EffectSubType::Delay_Note:
                channel.delayNote = note;
                channel.delayNoteTicksLeft = param;
                break;
            case EffectSubType::Pattern_Delay:
                patternDelay = param;
                break;
            case EffectSubType::Fine_Volume_Slide_Down:
                channel.vol = Clamp(0,64,channel.vol-param); 
                break;
            case EffectSubType::Fine_Volume_Slide_Up:          
                channel.vol = Clamp(0,64,channel.vol+param); 
                break;
            case EffectSubType::Fine_Porta_Down:
                channel.amigaPeriod = Clamp(0,1000, channel.amigaPeriod - param);
                break;
            case EffectSubType::Fine_Porta_Up:
                channel.amigaPeriod = Clamp(0,1000, channel.amigaPeriod + param);
                break;
            }
            break;
        }

        case Effect::Set_Speed:
            currentSpeed = note.eparm;
            break;
        }
        // mute other channels. testing.
        if( devSoloChannel >= 0 && devSoloChannel != channelIdx)  channel.vol = 0;
    }
}

void Mod::updateEffects()
{
    int patternIdx = devSoloPattern >= 0 ? 
        devSoloPattern : 
        order[currentOrder];

    for(int channelIdx = 0; channelIdx < nChannels; ++channelIdx)
    {
        Channel& c = channels[channelIdx];
        Note& note = patternData[(64*nChannels*patternIdx)+(nChannels*currentRow)+channelIdx];

        if( c.volSlideTicksLeft >0 )
        {
            c.vol = Clamp(0, 64, c.vol+c.volSlideSpeed);
            c.volSlideTicksLeft--;
            //printf("Chn=%d, Vol slide speed %d, vol=%d\n", channelIdx, c.volSlideSpeed, c.vol);
        }
        if(c.portaTicksLeft > 0)  
        {
            if( c.portaToNoteOffset != 0)
            {
                // targetting a note or just an offset from where we are?
                uint16_t targetFreqAmiga = Channel::getAmigaFreq(c.portaToNoteOffset, c.ft);
                if (targetFreqAmiga > c.amigaPeriod)
                    c.amigaPeriod = Min<int>(c.amigaPeriod + c.portaSpeed, targetFreqAmiga);
                else if (targetFreqAmiga < c.amigaPeriod)
                    c.amigaPeriod = Max<int>(c.amigaPeriod - c.portaSpeed, targetFreqAmiga);
            }
            else 
            {
                c.amigaPeriod = Clamp(0, 1000, c.portaSpeed + c.amigaPeriod);
            }
            c.portaTicksLeft--;
        }
        if( c.tremeTicksLeft > 0)
        {
            c.tremePos += c.tremeSpeed;
            if (c.tremePos >= 32) 
            {
                c.tremePos %= 32;
                c.tremeInv = !c.tremeInv;
            }
            int period = c.amigaPeriod;
            int tre = sine_table[c.vibrPos];
            tre *= c.tremeDepth;
            tre /= 64;
            if( !c.tremeInv ) tre = -tre; 
            period += tre;
            
            c.tremeTicksLeft--;

            c.amigaPeriod = period;
            if( channelIdx== devSoloChannel)
            {
                //printf("chn=%d vibrato: ticksleft=%d, pos=%d, depth=%d, speed=%d vib=%d, inv=%d, period=%d\n",
                //    channelIdx, c.vibrTicksLeft, c.vibrPos, c.vibrDepth, c.vibrSpeed, vib, c.vibrInv, c.amigaPeriod );
            }

            c.tremeTicksLeft--;
        }
        if( c.vibrTicksLeft > 0)
        {
            c.vibrPos += c.vibrSpeed;
            if (c.vibrPos >= 32) 
            {
                c.vibrPos %= 32;
                c.vibrInv = !c.vibrInv;
            }
            int period = c.amigaPeriod;
            int vib = sine_table[c.vibrPos];
            vib *= c.vibrDepth;
            vib /= 128;
            if( !c.vibrInv ) vib = -vib; 
            period += vib;
            
            c.vibrTicksLeft--;

            c.amigaPeriod = period;
            if( channelIdx== devSoloChannel)
            {
                //printf("chn=%d vibrato: ticksleft=%d, pos=%d, depth=%d, speed=%d vib=%d, inv=%d, period=%d\n",
                //    channelIdx, c.vibrTicksLeft, c.vibrPos, c.vibrDepth, c.vibrSpeed, vib, c.vibrInv, c.amigaPeriod );
            }
        }

        if( c.delayNoteTicksLeft > 0)
        {
            c.delayNoteTicksLeft--;
            // time to trigger?
            if( c.delayNoteTicksLeft == 0 && c.delayNote.sampleNumber > 0)
            {
                if( Sample* s = &samples[c.delayNote.sampleNumber] )
                    c.setSample(s);
                if( int period = Channel::getAmigaFreq(c.delayNote.noteOffset, c.ft) )
                {
                    c.amigaPeriod = period;                    
                }
            }
        }

        // mute other channels. testing.
        if( devSoloChannel >= 0 && devSoloChannel != channelIdx)  c.vol = 0;
    }
}


 size_t Mod::makeAudio(
    std::vector<float>& leftMix,
    std::vector<float>& rightMix,
    uint32_t sampleRate, 
    int frameSize)
{
    std::unique_lock<std::mutex> lk(cs);

    
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
            //printf("%d, name='%s', length=0x%x, finetune=%d, vol=%d, loopstart=%x, looplength=%x\n", 
            //    i, name, (int)s.length, s.fine_tune, s.volume, s.loop_start, s.loop_length);
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
           //printf("%02x, ",order);
       }
    }
    //printf("\nLength=%02x, nPatterns=%02x\n", gMod.songLength, gMod.nPatterns);
    assert((uintptr_t)readPtr8 - (uintptr_t)mem == 1080);

    //- read 4 bytes, discard them (we are at position 1080 again, this is M.K. etc!)
    readPtr8+=4;

    size_t patternDataSize = 64 * gMod.nChannels * sizeof(Note) * (gMod.nPatterns+1);
    gMod.patternData = (Note*)malloc(patternDataSize);

    for(int i = 0; i< gMod.nPatterns; ++i)
    {
        Note* note = &gMod.patternData[(gMod.nChannels * 64 * i)];
        //printf("\npattern=%02x", i);
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
            note->effect = static_cast<Effect>(byte2 & 0x0F);
            note->eparm = byte3;

            //prettyPrintNote(note, j % nChannels, nChannels, j / nChannels );
            note++;  
        }
    }
    //printf("\n");

    int8_t* samples = (int8_t*)mem+size;

    // Load sample data.
    for(int i = 31; i >=1; --i)
    {
        Sample& s = gMod.samples[i];
        if( s.length > 0 )
        {
            samples -= s.length;
	    s.data.resize(s.length);
            //s.data = (int8_t*)malloc(s.length);
            //memcpy(s.data, samples, s.length);
	    for(int j=0; j<s.length; ++j)
		s.data[j]=(float)s8ToFloatRecp*samples[j];

	    /*
            std::stringstream ss;
            ss << "smp_" << i << ".raw";
            if( FILE* fp = fopen(ss.str().c_str(), "wb"))
            {
                fwrite(s.data.data(), s.length, sizeof(float), fp);
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

float norm(float min, float max, float v)
{
	float c = v < min ? min : v > max ? max : v;
	float recp = 1.f / (max-min);
	return c*recp;
}

int min(int a, int b) { return a < b ? a : b; }

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

//
const uint8_t sine_table[32] = 
{  0, 24, 49, 74, 97,120,141,161,
	 180,197,212,224,235,244,250,253,
	 255,253,250,244,235,224,212,197,
	 180,161,141,120, 97, 74, 49, 24 };

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
        //{"C-", "C#", "D-", "D#","E-", "F-", "F#", "G-","G#","A-", "A#", "B-" };
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
