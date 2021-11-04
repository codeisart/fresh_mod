#pragma once

#include <math.h>
#include <functional>
#include <chrono>
#include <thread>

inline float norm(float min, float max, float v)
{
	float c = v < min ? min : v > max ? max : v;
	float recp = 1.f / (max-min);
	return c*recp;
}

inline int min(int a, int b) { return a < b ? a : b; }

// This is a fast approximation to log2()
// Y = C[0]*F*F*F + C[1]*F*F + C[2]*F + C[3] + E;
inline float log2f_approx(float X) {
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

inline float linearToDb(float f)
{
    static const float kSmallNumber = 1.e-8f;
    if( f > kSmallNumber )
    	return 20.f * log10f_fast(f); //log10f(f);
    
    return -96.f;
}
template<typename T> inline T clamp(T mn, T mx, T v) { return v <  mn ? mn : v > mx ? mx : v; }

inline float lerp( float s1, float s2, float t )
{
    float d = s2-s1;
    float l = s1 + (d*t);    
    return l;
}

inline int32_t lerpFixed( int32_t s1, int32_t s2, uint32_t t)
{
    int32_t d = (s2-s1) << 16;
    int32_t p = ((int64_t)(d*t) / (1 << 16)); 
    int32_t l = s1 + (p / (1 << 16));
    return l;
    // fp_fract 0...65535 (0...100)
    //uint32_t (65535 - fp_fract);
}

inline void timer_start(std::function<void(void)> func, unsigned int interval)
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

template<typename T> T Max(T a,T b) { return a > b ? a : b; }
template<typename T> T Min(T a,T b) { return a < b ? a : b; }
template<typename T> T Abs(T v) { return v < 0 ? -v : v; }
template<typename T> T Clamp(T min, T max, T v) { return v < min ? min : v > max ? max : v; }

#define ERR_WRAP(mac_err) do { result = mac_err ; line = __LINE__ ; if ( result != paNoError ) goto error; } while(0)

template<typename T>
struct RingBuffer
{
    int head, tail, size;
    T* buff;
    RingBuffer(RingBuffer&) = delete;
    RingBuffer(RingBuffer&& rhs) : buff(rhs.buff), head(rhs.head), tail(rhs.tail), size(rhs.size) { rhs.buff = nullptr; }
    RingBuffer(int InSize) : buff(new T[InSize]), head(0), tail(0), size(InSize) {}
    ~RingBuffer() { delete[] buff; }
    
    int increment(int i) const { return (i+1)%size; }  
    int decrement(int i) const { return (i-1+size)%size; }

    bool push(const T& v) 
    { 
        int next = increment(head);
        if( next == tail) return false; // overflow
        buff[head] = v; 
        head=next;
        return true;
    }  
    bool pop(T& out) 
    {
         if(isEmpty()) return false;  // underflow
         out = buff[tail]; 
         tail=increment(tail); 
         return true; 
    }

    int num() const { return head >= tail ? head-tail : (head+size)-tail; }
    int space() const { return size-1 - num(); }
    bool isEmpty() const { return head == tail; }
    bool isFull() const { return increment(head) == tail; }
};

#if TESTING_MOFO
void testCircbuffer()
{
    // test 1. write 
    {
        //CircBuff a(100 * sizeof(float));
        RingBuffer<float> a(2);
        assert(a.num() == 0);
        a.push(0.12345f); 
        assert(a.num() == 1);
        float tmp;
        assert(a.pop(tmp));
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
            float tmp;
            assert(a.pop(tmp));
            assert(tmp == 0.12345f);
        } 
    }

    // test 3. 
    {
        RingBuffer<float> a(128);
        while(a.space()>0)
        {
            a.push(0.12345f); 
        } 
        while(a.num())
        {
            assert(a.num() > 0);
            float tmp;
            assert(a.pop(tmp));
            assert(tmp == 0.12345f); 
        }
    }

     // test 4. 
    {
        RingBuffer<float> a(128);
        for( int i = 0; i < 64; ++i )
        {
            assert(a.num() == i);
            a.push((float)i);    
        }
        for( int i=0; i < 64; ++i)
        {
            float tmp;
            assert(a.num() == 64-i);
            assert(a.pop(tmp));
            assert(tmp == (float)i); 
        }
    }

     // test 5. 
    {
        RingBuffer<float> a(128);
        assert(a.space() == 127);
        for( int i = 0; i < 128; ++i )
        {
            assert(a.num() == i);
            a.push((float)i);    
        }
        assert(a.space()==0);
        assert(a.isFull());
        assert(!a.isEmpty());

        for( int i=0; i < 128; ++i)
        {
            float tmp;
            assert(a.pop(tmp) || i == 127);
            assert(tmp == (float)i || i == 127); 
        }
    }
}
#endif //TESTING_MOFO