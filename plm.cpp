#include <stdio.h>
#include "util.h"
#include <stdio.h>
#include <memory.h>
#include <vector>

template<typename T> bool readX(T& X, void *&Ptr, const void* End)
{
    T* TypedPtr = static_cast<T*>(Ptr);
    T* NextPtr = TypedPtr+1;
    if(TypedPtr < End)
    {
        X = *TypedPtr;
        Ptr = NextPtr;
        return true;
    }
    return false;
}
template<typename T>
bool readX(std::vector<T>& Out, void*& Ptr,const void* End)
{
    size_t N = Out.size();
    for(uint32_t i = 0; i < N; ++i)
        if(!readX(Out[i], Ptr, End))
            return false;
    
    return true;
}
template<typename T, uint32_t N>
bool readX(T (&Out)[N], void*& Ptr,const void* End)
{
    for(uint32_t i = 0; i < N; ++i)
        if(!readX(Out[i], Ptr, End))
            return false;
    
    return true;
}

template<uint32_t MaxLength>
std::string readNonTerminatedString(void*& Ptr, const void* End)
{
    char Buff[MaxLength] = {0};
    if(readX(Buff, Ptr, End))
        return { Buff, Buff+MaxLength };   
    return {};
}

struct OrderFormat
{
    uint16_t x;
    uint8_t y;
    uint8_t pattern; 
};

struct NoteFormat
{
    uint8_t pitch;      // hi nib is octave, low is note. 0=blank
    uint8_t sample;     // sample index, 0 if blank.
    uint8_t vol;        // 0xff = blank.
    uint8_t cmd;
    uint8_t info;
};

struct PatternFormat
{
    uint32_t size;
    uint8_t len;
    uint8_t wid;
    uint8_t color;
    std::string name;
    std::vector<NoteFormat> notes;
};

struct SampleFormat
{
    uint32_t id;        // 4 bytes         always "PLS" then character 26
    uint8_t headersize; // 1 byte          size of header in bytes, including ID etc
    uint8_t version;    // 1 byte
    std::string fullname;  // 32 byte            ; NOT asciiz
    std::string filename;  // 12 byte            ; ditto
    uint8_t pan;        // byte               ; default pan, 0..f, >f=none
    uint8_t vol;        // byte             //; default vol 0..40h
    uint8_t flags;      // byte             //; 1 = 16 bit , 0=8 bit
    uint16_t c4spd;     // word             //; c4spd (as for S3M)
    uint32_t gusloc;    // dword            //  ; posn in gusram (not used in file)
    uint32_t loopst;    // dword            //  ; loopstart
    uint32_t loopen;    // dword            //  ; loopend
    uint32_t len;       // dword            //  ; data size IN BYTES
    //data            lots of bytes   //  ; unsigned data
    std::vector<uint8_t> data;
};

bool loadPlm(void* mem, int len)
{
    uint8_t* start = (uint8_t*)mem;
    uint8_t* end = (uint8_t*)mem+len;

    // ID.
    uint32_t ID;
    if(!readX(ID,mem,end))
    {
        printf("Failed to read ID from from PLM file");
        return false;
    }
    static const uint8_t PLM[] { 0x50, 0x4C, 0x4D, 0x1A };    
    if(ID != *(const uint32_t*)PLM) 
    {
        printf("ID is not a PLM file. ID=0x%x\n", ID);
        return false;
    }

    // Header size
    uint8_t HeaderSize = 0;
    if(!readX(HeaderSize,mem,end))
    {
        printf("failed to read header size\n");
        return false;
    }    

    // Version.
    uint8_t version = 0;
    static const uint8_t ExpectedVersion = 0x10;
    if( !readX(version,mem,end) || version != ExpectedVersion)
    {
        printf("Failed to validate version. Expecting=%d, Ver=%d\n", ExpectedVersion, (int)version);
        return false;
    }

    // Name.
    char Name[48];
    if(!readX(Name,mem,end))
    {
        printf("Failed to read name from header.\n");
        return false;
    }
   
    // Number of Channels
    uint8_t NumChannels = 0;
    if(!readX(NumChannels, mem,end) || NumChannels ==0)
    {
        printf("Failed to read channel count from header\n");
        return false;
    }

    // Flags.
    uint8_t Flags = 0;
    if(!readX(Flags, mem,end))
    {
        printf("Failed to read flags from header\n");
        return false;
    }

    uint8_t MaxVol = 0;
    if(!readX(MaxVol, mem,end))
    {
        printf("Failed to read Max Vol from header\n");
        return false;
    }
    
    uint8_t Amplify = 0;
    if(!readX(Amplify, mem,end))
    {
        printf("Failed to read Amplify from header\n");
        return false;
    }

    uint8_t Bpm = 0;
    if(!readX(Bpm, mem,end))
    {
        printf("Failed to read BPM from header\n");
        return false;
    }

    uint8_t Speed = 0;
    if(!readX(Speed, mem,end))
    {
        printf("Failed to read Speed from header\n");
        return false;
    }

    uint8_t pan[32];
    if(!readX(pan,mem,end))
    {
        printf("Failed to read Pan Array\n");
        return false;
    }
    
    uint8_t NumSamples = 0;
    if(!readX(NumSamples,mem,end))
    {
        printf("Failed to read Sample Count\n");
        return false;
    }
    uint8_t NumPatterns = 0;
    if(!readX(NumPatterns,mem,end))
    {
        printf("Failed to read Pattern Count\n");
        return false;
    }
    uint16_t NumOrders = 0;
    if(!readX(NumOrders,mem,end))
    {
        printf("Failed to read Order Count\n");
        return false;
    }

    printf("Successfully parsed PLM header. NumSamples=%d, NumOrders=%d, NumPatterns=%d\n",
        NumSamples, NumOrders, NumPatterns
    );

    // There's some padding and what not, so use the header size from start.
    void* afterHeader = start+HeaderSize;

    // Order list.
    std::vector<OrderFormat> OrderList;
    OrderList.resize(NumOrders);
    readX(OrderList,afterHeader,end);

    // Pattern offsets.
    std::vector<uint32_t> PatternOffsets;
    PatternOffsets.resize(NumPatterns);
    readX(PatternOffsets, afterHeader,end);

    // Sample offsets.
    std::vector<uint32_t> SampleOffsets;
    SampleOffsets.resize(NumSamples);
    readX(SampleOffsets,afterHeader,end);

    // Read patterns
    for(uint32_t i = 0;  i <PatternOffsets.size(); ++i)
    {
        uint32_t Offset = PatternOffsets[i];
        if(Offset==0) 
            continue;
        void* PatternStart = start+Offset;
        PatternFormat Pat;
        if(!readX(Pat.size,PatternStart,end))
        {
            printf("Failed to read Pattern length\n");
            return false;
        }
        void* PatternEnd = (uint8_t*)PatternStart+Pat.size;
        if(!readX(Pat.len,PatternStart,PatternEnd))
        {
            printf("Failed to read Pattern length\n");
            return false;
        }
        if(!readX(Pat.wid,PatternStart,PatternEnd))
        {
            printf("Failed to read Pattern width\n");
            return false;
        }
        if(!readX(Pat.color,PatternStart,PatternEnd))
        {
            printf("Failed to read Pattern Color\n");
            return false;
        }
        char Name[25] = {0};
        if(!readX(Name,PatternStart,PatternEnd))
        {
            printf("Failed to read Pattern Name\n");
            return false;
        }
        Pat.name = Name;

        // Make enough room for the notes.
        Pat.notes.resize(Pat.wid*Pat.len);
        
        // Read row at a time.
        for(uint32_t y = 0; y < Pat.len; y++)
        {
            for (uint32_t x = 0; x < Pat.wid; x++)
            {
                NoteFormat &Note = Pat.notes[(y*Pat.wid)+x];
                readX(Note.pitch, PatternStart, PatternEnd);
                readX(Note.sample, PatternStart, PatternEnd);
                readX(Note.vol, PatternStart, PatternEnd);
                readX(Note.cmd, PatternStart, PatternEnd);
                readX(Note.info, PatternStart, PatternEnd);
            }
        }
        
        printf("Read Pattern: %u, NChannels=%d, Rows=%d, Name=%s\n", 
            i, Pat.wid, Pat.len, Pat.name.c_str());
    }

    std::vector<SampleFormat> Samples;
    Samples.resize(SampleOffsets.size());
    for( uint32_t i = 0; i < SampleOffsets.size(); ++i)
    {
        SampleFormat& Sample = Samples[i];
        uint32_t Offset = SampleOffsets[i];
        if(Offset==0) 
            continue;
        void* SampleStart = start+Offset;
        readX(Sample.id, SampleStart, end);
        readX(Sample.headersize, SampleStart, end);
        void* SampleEnd = (uint8_t*)SampleStart + Sample.headersize;
        readX(Sample.version, SampleStart, SampleEnd);
        Sample.fullname = readNonTerminatedString<32>(SampleStart, SampleEnd);
        Sample.filename = readNonTerminatedString<12>(SampleStart, SampleEnd);
        readX(Sample.pan, SampleStart, SampleEnd);
        readX(Sample.vol, SampleStart, SampleEnd);
        readX(Sample.flags, SampleStart, SampleEnd);
        readX(Sample.c4spd, SampleStart, SampleEnd);
        readX(Sample.gusloc, SampleStart, SampleEnd);
        readX(Sample.loopst, SampleStart, SampleEnd);
        readX(Sample.loopen, SampleStart, SampleEnd);
        readX(Sample.len, SampleStart, SampleEnd);
        if( Sample.len > 0)
        {
            Sample.data.resize(Sample.len);
            readX(Sample.data, SampleStart, (uint8_t*)SampleEnd+Sample.len);
        }

        std::string Filename = Sample.filename;
        if(Filename.empty()) Filename = Sample.fullname;
        char spbuf[32];
        sprintf(spbuf, "%d.pls",i); 
        if(Filename.empty()) Filename = spbuf; 
        if(FILE* fp = fopen(Filename.c_str(), "wb"))
        {
            fwrite(Sample.data.data(), Sample.data.size(), 1, fp);
            fclose(fp);
        }

        printf("Read Sample: %u, Name=%s Filename=%s\n", 
            i, Sample.fullname.c_str(), Sample.filename.c_str());
    }

    return true;
}

bool loadPlm(const char* filename)
{
    printf("loading plm... ");
    return loadWholeFile(filename, [](void* mem, size_t size) -> bool { return loadPlm(mem,size); });
}
