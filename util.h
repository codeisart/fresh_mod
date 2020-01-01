#pragma once


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