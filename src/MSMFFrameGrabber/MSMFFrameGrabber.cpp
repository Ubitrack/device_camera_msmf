/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
#include "opencv2/videoio.hpp"

#include "opencv2/core/utility.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgcodecs/imgcodecs_c.h"
#include "opencv2/videoio/videoio_c.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <assert.h>

#if defined WIN32 || defined WINCE
    #if !defined _WIN32_WINNT
        #ifdef HAVE_MSMF
            #define _WIN32_WINNT 0x0600 // Windows Vista
        #else
            #define _WIN32_WINNT 0x0500 // Windows 2000
        #endif
    #endif

    #include <windows.h>
    #undef small
    #undef min
    #undef max
    #undef abs
#endif

#if (defined WIN32 || defined _WIN32) && defined HAVE_MSMF
/*
   Media Foundation-based Video Capturing module is based on
   OpenCV VideoIO Module cap_msmf.* https://www.github.com/opencv/opencv
   and 
   videoInput library by Evgeny Pereguda:
   http://www.codeproject.com/Articles/559437/Capturing-of-video-from-web-camera-on-Windows-7-an
   Originaly licensed under The Code Project Open License (CPOL) 1.02:
   http://www.codeproject.com/info/cpol10.aspx
*/
//require Windows 8 for some of the formats defined otherwise could baseline on lower version
#if WINVER < _WIN32_WINNT_WIN7
#undef WINVER
#define WINVER _WIN32_WINNT_WIN7
#endif
#if defined _MSC_VER && _MSC_VER >= 1600
    #define HAVE_CONCURRENCY
#endif
#include <windows.h>
#include <guiddef.h>
#include <mfidl.h>
#include <Mfapi.h>
#include <mfplay.h>
#include <mfobjects.h>
#include <tchar.h>
#include <strsafe.h>
#include <Mfreadwrite.h>
#include <new>
#include <map>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// includes for Ubitrack integration
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>
#include <opencv/cv.h>
#include <utVision/Image.h>
#include <utVision/Undistortion.h>
#include <utVision/OpenCLManager.h>


#include <opencv/highgui.h>
#include <opencv2/core/ocl.hpp>

#ifdef _MSC_VER
#pragma warning(disable:4503)
#pragma comment(lib, "mfplat")
#pragma comment(lib, "mf")
#pragma comment(lib, "mfuuid")
#pragma comment(lib, "Strmiids")
#pragma comment(lib, "Mfreadwrite")
#if (WINVER >= 0x0602) // Available since Win 8
#pragma comment(lib, "MinCore_Downlevel")
#endif
#endif

#include <mferror.h>
#include <comdef.h>

struct IMFMediaType;
struct IMFActivate;
struct IMFMediaSource;
struct IMFAttributes;

namespace
{

template <class T> void SafeRelease(T **ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

#ifdef _DEBUG
/// Class for printing info into console
class DPO
{
public:
    ~DPO(void);
    static DPO& getInstance();
    void printOut(const wchar_t *format, ...);
    void setVerbose(bool state);
    bool verbose;
private:
    DPO(void);
};
#define DebugPrintOut(...) DPO::getInstance().printOut(__VA_ARGS__)
#else
#define DebugPrintOut(...) void()
#endif

#include "MSMFFrameGrabber.hpp"

// Structure for collecting info about types of video, which are supported by current video device
struct MediaType
{
    unsigned int MF_MT_FRAME_SIZE;
    unsigned int height;
    unsigned int width;
    unsigned int MF_MT_YUV_MATRIX;
    unsigned int MF_MT_VIDEO_LIGHTING;
    int MF_MT_DEFAULT_STRIDE; // stride is negative if image is bottom-up
    unsigned int MF_MT_VIDEO_CHROMA_SITING;
    GUID MF_MT_AM_FORMAT_TYPE;
    wchar_t *pMF_MT_AM_FORMAT_TYPEName;
    unsigned int MF_MT_FIXED_SIZE_SAMPLES;
    unsigned int MF_MT_VIDEO_NOMINAL_RANGE;
    unsigned int MF_MT_FRAME_RATE_NUMERATOR;
    unsigned int MF_MT_FRAME_RATE_DENOMINATOR;
    unsigned int MF_MT_PIXEL_ASPECT_RATIO;
    unsigned int MF_MT_PIXEL_ASPECT_RATIO_low;
    unsigned int MF_MT_ALL_SAMPLES_INDEPENDENT;
    unsigned int MF_MT_FRAME_RATE_RANGE_MIN;
    unsigned int MF_MT_FRAME_RATE_RANGE_MIN_low;
    unsigned int MF_MT_SAMPLE_SIZE;
    unsigned int MF_MT_VIDEO_PRIMARIES;
    unsigned int MF_MT_INTERLACE_MODE;
    unsigned int MF_MT_FRAME_RATE_RANGE_MAX;
    unsigned int MF_MT_FRAME_RATE_RANGE_MAX_low;
    GUID MF_MT_MAJOR_TYPE;
    GUID MF_MT_SUBTYPE;
    wchar_t *pMF_MT_MAJOR_TYPEName;
    wchar_t *pMF_MT_SUBTYPEName;
    MediaType();
    ~MediaType();
    void Clear();
};

/// Class for parsing info from IMFMediaType into the local MediaType
class FormatReader
{
public:
    static MediaType Read(IMFMediaType *pType);
    ~FormatReader(void);
private:
    FormatReader(void);
};

DWORD WINAPI MainThreadFunction( LPVOID lpParam );
typedef void(*emergensyStopEventCallback)(int, void *);
typedef void(*newFrameAvailableCallback)(int, void *, LONGLONG);

class RawImage
{
public:
    ~RawImage(void);
    // Function of creation of the instance of the class
    static long CreateInstance(RawImage **ppRImage,unsigned int size);
    void setCopy(const BYTE * pSampleBuffer);
    void fastCopy(const BYTE * pSampleBuffer);
    unsigned char * getpPixels();
    bool isNew();
    unsigned int getSize();
private:
    bool ri_new;
    unsigned int ri_size;
    unsigned char *ri_pixels;
    RawImage(unsigned int size);
};

class ImageGrabberCallback : public IMFSampleGrabberSinkCallback
{
public:
    void pauseGrabbing();
    void resumeGrabbing();
    RawImage *getRawImage();
    // IMFClockStateSink methods
    STDMETHODIMP OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset);
    STDMETHODIMP OnClockStop(MFTIME hnsSystemTime);
    STDMETHODIMP OnClockPause(MFTIME hnsSystemTime);
    STDMETHODIMP OnClockRestart(MFTIME hnsSystemTime);
    STDMETHODIMP OnClockSetRate(MFTIME hnsSystemTime, float flRate);
    // IMFSampleGrabberSinkCallback methods
    STDMETHODIMP OnSetPresentationClock(IMFPresentationClock* pClock);
    STDMETHODIMP OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
        LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE * pSampleBuffer,
        DWORD dwSampleSize);
    STDMETHODIMP OnShutdown();

	// method added to handle received frames without extra thread
	virtual void onNewFrameAvailable(LONGLONG ts);

    const HANDLE ig_hFrameReady;
    const HANDLE ig_hFrameGrabbed;
    const HANDLE ig_hFinish;
protected:
    ImageGrabberCallback(bool synchronous);
    bool ig_RIE;
    bool ig_Close;
    bool ig_Synchronous;
    long m_cRef;

    RawImage *ig_RIFirst;
    RawImage *ig_RISecond;
    RawImage *ig_RIOut;
private:
    ImageGrabberCallback& operator=(const ImageGrabberCallback&);   // Declared to fix compilation warning.
 };


// Class for grabbing image from video stream
class ImageGrabber : public ImageGrabberCallback
{
public:
    ~ImageGrabber(void);
    HRESULT initImageGrabber(IMFMediaSource *pSource);
    HRESULT startGrabbing(void);
    void stopGrabbing();
    // IUnknown methods
    STDMETHODIMP QueryInterface(REFIID iid, void** ppv);
    STDMETHODIMP_(ULONG) AddRef();
    STDMETHODIMP_(ULONG) Release();
    // Function of creation of the instance of the class
    static HRESULT CreateInstance(ImageGrabber **ppIG, unsigned int deviceID, bool synchronous = false);

	// method added to handle received frames without extra thread
	void setNewFrameAvailableEvent(void *userData, void(*func)(int, void *, LONGLONG));
	virtual void onNewFrameAvailable(LONGLONG ts);


private:
    unsigned int ig_DeviceID;

    IMFMediaSource *ig_pSource;
    IMFMediaSession *ig_pSession;
    IMFTopology *ig_pTopology;
    ImageGrabber(unsigned int deviceID, bool synchronous);
    HRESULT CreateTopology(IMFMediaSource *pSource, IMFActivate *pSinkActivate, IMFTopology **ppTopo);
    HRESULT AddSourceNode(IMFTopology *pTopology, IMFMediaSource *pSource,
        IMFPresentationDescriptor *pPD, IMFStreamDescriptor *pSD, IMFTopologyNode **ppNode);
    HRESULT AddOutputNode(IMFTopology *pTopology, IMFActivate *pActivate, DWORD dwId, IMFTopologyNode **ppNode);

    ImageGrabber& operator=(const ImageGrabber&);   // Declared to fix comiplation error.

	newFrameAvailableCallback ig_funcNewFrame;
	void *ig_userDataNewFrame;


};


/// Class for controlling of thread of the grabbing raw data from video device
class ImageGrabberThread
{
    friend DWORD WINAPI MainThreadFunction( LPVOID lpParam );
public:
    ~ImageGrabberThread(void);
    static HRESULT CreateInstance(ImageGrabberThread **ppIGT, IMFMediaSource *pSource, unsigned int deviceID, bool synchronious = false);
    void start();
    void stop();
    void setEmergencyStopEvent(void *userData, void(*func)(int, void *));
    ImageGrabber *getImageGrabber();
protected:
    virtual void run();
private:
    ImageGrabberThread(IMFMediaSource *pSource, unsigned int deviceID, bool synchronious);
    HANDLE igt_Handle;
    DWORD   igt_ThreadIdArray;
    ImageGrabber *igt_pImageGrabber;
    emergensyStopEventCallback igt_func;
    void *igt_userData;
    bool igt_stop;
    unsigned int igt_DeviceID;
};





// Structure for collecting info about one parametr of current video device
struct Parametr
{
    long CurrentValue;
    long Min;
    long Max;
    long Step;
    long Default;
    long Flag;
    Parametr();
};

// Structure for collecting info about 17 parametrs of current video device
struct CamParametrs
{
        Parametr Brightness;
        Parametr Contrast;
        Parametr Hue;
        Parametr Saturation;
        Parametr Sharpness;
        Parametr Gamma;
        Parametr ColorEnable;
        Parametr WhiteBalance;
        Parametr BacklightCompensation;
        Parametr Gain;
        Parametr Pan;
        Parametr Tilt;
        Parametr Roll;
        Parametr Zoom;
        Parametr Exposure;
        Parametr Iris;
        Parametr Focus;
};

typedef std::wstring String;
typedef std::vector<int> vectorNum;
typedef std::map<String, vectorNum> SUBTYPEMap;
typedef std::map<UINT64, SUBTYPEMap> FrameRateMap;
typedef void(*emergensyStopEventCallback)(int, void *);

/// Class for controlling of video device
class videoDevice
{
public:
    videoDevice(void);
    ~videoDevice(void);
    void closeDevice();
    CamParametrs getParametrs();
    void setParametrs(CamParametrs parametrs);
    void setEmergencyStopEvent(void *userData, void(*func)(int, void *));
	void setNewFrameAvailableEvent(void *userData, void(*func)(int, void *, LONGLONG));
    long readInfoOfDevice(IMFActivate *pActivate, unsigned int Num);
    wchar_t *getName();
    int getCountFormats();
    unsigned int getWidth();
    unsigned int getHeight();
    unsigned int getFrameRate() const;
    MediaType getFormat(unsigned int id);
    bool setupDevice(unsigned int w, unsigned int h, unsigned int idealFramerate = 0);
    bool setupDevice(unsigned int id);
    bool isDeviceSetup();
    bool isDeviceMediaSource();
	ImageGrabber *getImageGrabber();
    bool isDeviceRawDataSource();
    bool isFrameNew();
    IMFMediaSource *getMediaSource();
    RawImage *getRawImageOut();
private:
    enum typeLock
    {
        MediaSourceLock,
        RawDataLock,
        OpenLock
    } vd_LockOut;
    wchar_t *vd_pFriendlyName;
    ImageGrabberThread *vd_pImGrTh;
    CamParametrs vd_PrevParametrs;
    unsigned int vd_Width;
    unsigned int vd_Height;
    unsigned int vd_FrameRate;
    unsigned int vd_CurrentNumber;
    bool vd_IsSetuped;
    std::map<UINT64, FrameRateMap> vd_CaptureFormats;
    std::vector<MediaType> vd_CurrentFormats;
    IMFMediaSource *vd_pSource;
    emergensyStopEventCallback vd_func;
	void *vd_userData;
    HRESULT enumerateCaptureFormats(IMFMediaSource *pSource);
    long setDeviceFormat(IMFMediaSource *pSource, unsigned long dwFormatIndex);
    void buildLibraryofTypes();
    int findType(unsigned int size, unsigned int frameRate = 0);
    long resetDevice(IMFActivate *pActivate);
    long checkDevice(IMFAttributes *pAttributes, IMFActivate **pDevice);
    long initDevice();
};

/// Class for managing of list of video devices
class videoDevices
{
public:
    ~videoDevices(void);
    long initDevices(IMFAttributes *pAttributes);
    static videoDevices& getInstance();
    videoDevice *getDevice(unsigned int i);
	int getDeviceIdForUUID(const std::string& uuid);
    unsigned int getCount();
    void clearDevices();
private:
    UINT32 count;
    std::vector<videoDevice *> vds_Devices;
    videoDevices(void);
};

// Class for creating of Media Foundation context
class Media_Foundation
{
public:
    virtual ~Media_Foundation(void);
    static Media_Foundation& getInstance();
    bool buildListOfDevices();
private:
    Media_Foundation(void);
};

/// The only visiable class for controlling of video devices in format singelton
class videoInput
{
public:
    virtual ~videoInput(void);
    // Getting of static instance of videoInput class
    static videoInput& getInstance();
    // Closing video device with deviceID
    void closeDevice(int deviceID);
    // Setting callback function for emergency events(for example: removing video device with deviceID) with userData
    void setEmergencyStopEvent(int deviceID, void *userData, void(*func)(int, void *));
	// Setting callback function for notification on new frame with userData
	void setNewFrameAvailableEvent(int deviceID, void *userData, void(*func)(int, void *, LONGLONG));
    // Closing all devices
    void closeAllDevices();
    // Getting of parametrs of video device with deviceID
    CamParametrs getParametrs(int deviceID);
    // Setting of parametrs of video device with deviceID
    void setParametrs(int deviceID, CamParametrs parametrs);
    // Getting numbers of existence videodevices with listing in consol
    unsigned int listDevices(bool silent = false);
    // Getting numbers of formats, which are supported by videodevice with deviceID
    unsigned int getCountFormats(int deviceID) const;
    // Getting width of image, which is getting from videodevice with deviceID
    unsigned int getWidth(int deviceID) const;
    // Getting height of image, which is getting from videodevice with deviceID
    unsigned int getHeight(int deviceID) const;
    // Getting frame rate, which is getting from videodevice with deviceID
    unsigned int getFrameRate(int deviceID) const;
    // Getting name of videodevice with deviceID
    wchar_t *getNameVideoDevice(int deviceID);
    // Getting interface MediaSource for Media Foundation from videodevice with deviceID
    IMFMediaSource *getMediaSource(int deviceID);
    // Getting format with id, which is supported by videodevice with deviceID
    MediaType getFormat(int deviceID, int unsigned id);
    // Checking of existence of the suitable video devices
    bool isDevicesAcceable();
    // Checking of using the videodevice with deviceID
    bool isDeviceSetup(int deviceID);
    // Checking of using MediaSource from videodevice with deviceID
    bool isDeviceMediaSource(int deviceID);
    // Checking of using Raw Data of pixels from videodevice with deviceID
    bool isDeviceRawDataSource(int deviceID);
#ifdef _DEBUG
    // Setting of the state of outprinting info in console
    static void setVerbose(bool state);
#endif
    // Initialization of video device with deviceID by media type with id
    bool setupDevice(int deviceID, unsigned int id = 0);
    // Initialization of video device with deviceID by wisth w, height h and fps idealFramerate
    bool setupDevice(int deviceID, unsigned int w, unsigned int h, unsigned int idealFramerate = 30);
    // Checking of recivig of new frame from video device with deviceID
    bool isFrameNew(int deviceID);
    // Writing of Raw Data pixels from video device with deviceID with correction of RedAndBlue flipping flipRedAndBlue and vertical flipping flipImage
    bool getPixels(int deviceID, unsigned char * pixels, bool flipRedAndBlue = false, bool flipImage = false);
    static void processPixels(unsigned char * src, unsigned char * dst, unsigned int width, unsigned int height, unsigned int bpp, bool bRGB, bool bFlip);
private:
    bool accessToDevices;
    videoInput(void);
    void updateListOfDevices();
};

#ifdef _DEBUG
DPO::DPO(void):verbose(true)
{
}

DPO::~DPO(void)
{
}

DPO& DPO::getInstance()
{
    static DPO instance;
    return instance;
}

void DPO::printOut(const wchar_t *format, ...)
{
    if(verbose)
    {
        int i = 0;
        wchar_t *p = NULL;
        va_list args;
        va_start(args, format);
        if( ::IsDebuggerPresent() )
        {
            WCHAR szMsg[512];
            ::StringCchVPrintfW(szMsg, sizeof(szMsg)/sizeof(szMsg[0]), format, args);
            ::OutputDebugStringW(szMsg);
        }
        else
        {
            if(wcscmp(format, L"%i"))
            {
                i = va_arg (args, int);
            }
            if(wcscmp(format, L"%s"))
            {
                p = va_arg (args, wchar_t *);
            }
            wprintf(format, i,p);
        }
        va_end (args);
    }
}

void DPO::setVerbose(bool state)
{
    verbose = state;
}
#endif

LPCWSTR GetGUIDNameConstNew(const GUID& guid);
HRESULT GetGUIDNameNew(const GUID& guid, WCHAR **ppwsz);
HRESULT LogAttributeValueByIndexNew(IMFAttributes *pAttr, DWORD index);
HRESULT SpecialCaseAttributeValueNew(GUID guid, const PROPVARIANT& var, MediaType &out);

unsigned int *GetParametr(GUID guid, MediaType &out)
{
    if(guid == MF_MT_YUV_MATRIX)
        return &(out.MF_MT_YUV_MATRIX);
    if(guid == MF_MT_VIDEO_LIGHTING)
        return &(out.MF_MT_VIDEO_LIGHTING);
    if(guid == MF_MT_DEFAULT_STRIDE)
        return (unsigned int*)&(out.MF_MT_DEFAULT_STRIDE);
    if(guid == MF_MT_VIDEO_CHROMA_SITING)
        return &(out.MF_MT_VIDEO_CHROMA_SITING);
    if(guid == MF_MT_VIDEO_NOMINAL_RANGE)
        return &(out.MF_MT_VIDEO_NOMINAL_RANGE);
    if(guid == MF_MT_ALL_SAMPLES_INDEPENDENT)
        return &(out.MF_MT_ALL_SAMPLES_INDEPENDENT);
    if(guid == MF_MT_FIXED_SIZE_SAMPLES)
        return &(out.MF_MT_FIXED_SIZE_SAMPLES);
    if(guid == MF_MT_SAMPLE_SIZE)
        return &(out.MF_MT_SAMPLE_SIZE);
    if(guid == MF_MT_VIDEO_PRIMARIES)
        return &(out.MF_MT_VIDEO_PRIMARIES);
    if(guid == MF_MT_INTERLACE_MODE)
        return &(out.MF_MT_INTERLACE_MODE);
    return NULL;
}

HRESULT LogAttributeValueByIndexNew(IMFAttributes *pAttr, DWORD index, MediaType &out)
{
    WCHAR *pGuidName = NULL;
    WCHAR *pGuidValName = NULL;
    GUID guid = { 0 };
    PROPVARIANT var;
    PropVariantInit(&var);
    HRESULT hr = pAttr->GetItemByIndex(index, &guid, &var);
    if (FAILED(hr))
    {
        goto done;
    }
    hr = GetGUIDNameNew(guid, &pGuidName);
    if (FAILED(hr))
    {
        goto done;
    }
    hr = SpecialCaseAttributeValueNew(guid, var, out);
    unsigned int *p;
    if (FAILED(hr))
    {
        goto done;
    }
    if (hr == S_FALSE)
    {
        switch (var.vt)
        {
        case VT_UI4:
            p = GetParametr(guid, out);
            if(p)
            {
                *p = var.ulVal;
            }
            break;
        case VT_UI8:
            break;
        case VT_R8:
            break;
        case VT_CLSID:
            if(guid == MF_MT_AM_FORMAT_TYPE)
            {
                hr = GetGUIDNameNew(*var.puuid, &pGuidValName);
                if (SUCCEEDED(hr))
                {
                    out.MF_MT_AM_FORMAT_TYPE = *var.puuid;
                    out.pMF_MT_AM_FORMAT_TYPEName = pGuidValName;
                    pGuidValName = NULL;
                }
            }
            if(guid == MF_MT_MAJOR_TYPE)
            {
                hr = GetGUIDNameNew(*var.puuid, &pGuidValName);
                if (SUCCEEDED(hr))
                {
                    out.MF_MT_MAJOR_TYPE = *var.puuid;
                    out.pMF_MT_MAJOR_TYPEName = pGuidValName;
                    pGuidValName = NULL;
                }
            }
            if(guid == MF_MT_SUBTYPE)
            {
                hr = GetGUIDNameNew(*var.puuid, &pGuidValName);
                if (SUCCEEDED(hr))
                {
                    out.MF_MT_SUBTYPE = *var.puuid;
                    out.pMF_MT_SUBTYPEName = pGuidValName;
                    pGuidValName = NULL;
                }
            }
            break;
        case VT_LPWSTR:
            break;
        case VT_VECTOR | VT_UI1:
            break;
        case VT_UNKNOWN:
            break;
        default:
            break;
        }
    }
done:
    CoTaskMemFree(pGuidName);
    CoTaskMemFree(pGuidValName);
    PropVariantClear(&var);
    return hr;
}

HRESULT GetGUIDNameNew(const GUID& guid, WCHAR **ppwsz)
{
    HRESULT hr = S_OK;
    WCHAR *pName = NULL;
    LPCWSTR pcwsz = GetGUIDNameConstNew(guid);
    if (pcwsz)
    {
        size_t cchLength = 0;
        hr = StringCchLengthW(pcwsz, STRSAFE_MAX_CCH, &cchLength);
        if (FAILED(hr))
        {
            goto done;
        }
        pName = (WCHAR*)CoTaskMemAlloc((cchLength + 1) * sizeof(WCHAR));
        if (pName == NULL)
        {
            hr = E_OUTOFMEMORY;
            goto done;
        }
        hr = StringCchCopyW(pName, cchLength + 1, pcwsz);
        if (FAILED(hr))
        {
            goto done;
        }
    }
    else
    {
        hr = StringFromCLSID(guid, &pName);
    }
done:
    if (FAILED(hr))
    {
        *ppwsz = NULL;
        CoTaskMemFree(pName);
    }
    else
    {
        *ppwsz = pName;
    }
    return hr;
}

void LogUINT32AsUINT64New(const PROPVARIANT& var, UINT32 &uHigh, UINT32 &uLow)
{
    Unpack2UINT32AsUINT64(var.uhVal.QuadPart, &uHigh, &uLow);
}

float OffsetToFloatNew(const MFOffset& offset)
{
    return offset.value + (static_cast<float>(offset.fract) / 65536.0f);
}

HRESULT LogVideoAreaNew(const PROPVARIANT& var)
{
    if (var.caub.cElems < sizeof(MFVideoArea))
    {
        return S_OK;
    }
    return S_OK;
}

HRESULT SpecialCaseAttributeValueNew(GUID guid, const PROPVARIANT& var, MediaType &out)
{
    if (guid == MF_MT_DEFAULT_STRIDE)
    {
        out.MF_MT_DEFAULT_STRIDE = var.intVal;
    } else
    if (guid == MF_MT_FRAME_SIZE)
    {
        UINT32 uHigh = 0, uLow = 0;
        LogUINT32AsUINT64New(var, uHigh, uLow);
        out.width = uHigh;
        out.height = uLow;
        out.MF_MT_FRAME_SIZE = out.width * out.height;
    }
    else
    if (guid == MF_MT_FRAME_RATE)
    {
        UINT32 uHigh = 0, uLow = 0;
        LogUINT32AsUINT64New(var, uHigh, uLow);
        out.MF_MT_FRAME_RATE_NUMERATOR = uHigh;
        out.MF_MT_FRAME_RATE_DENOMINATOR = uLow;
    }
    else
    if (guid == MF_MT_FRAME_RATE_RANGE_MAX)
    {
        UINT32 uHigh = 0, uLow = 0;
        LogUINT32AsUINT64New(var, uHigh, uLow);
        out.MF_MT_FRAME_RATE_RANGE_MAX = uHigh;
        out.MF_MT_FRAME_RATE_RANGE_MAX_low = uLow;
    }
    else
    if (guid == MF_MT_FRAME_RATE_RANGE_MIN)
    {
        UINT32 uHigh = 0, uLow = 0;
        LogUINT32AsUINT64New(var, uHigh, uLow);
        out.MF_MT_FRAME_RATE_RANGE_MIN = uHigh;
        out.MF_MT_FRAME_RATE_RANGE_MIN_low = uLow;
    }
    else
    if (guid == MF_MT_PIXEL_ASPECT_RATIO)
    {
        UINT32 uHigh = 0, uLow = 0;
        LogUINT32AsUINT64New(var, uHigh, uLow);
        out.MF_MT_PIXEL_ASPECT_RATIO = uHigh;
        out.MF_MT_PIXEL_ASPECT_RATIO_low = uLow;
    }
    else
    {
        return S_FALSE;
    }
    return S_OK;
}

#ifndef IF_EQUAL_RETURN
#define IF_EQUAL_RETURN(param, val) if(val == param) return L#val
#endif

LPCWSTR GetGUIDNameConstNew(const GUID& guid)
{
    IF_EQUAL_RETURN(guid, MF_MT_MAJOR_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_MAJOR_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_SUBTYPE);
    IF_EQUAL_RETURN(guid, MF_MT_ALL_SAMPLES_INDEPENDENT);
    IF_EQUAL_RETURN(guid, MF_MT_FIXED_SIZE_SAMPLES);
    IF_EQUAL_RETURN(guid, MF_MT_COMPRESSED);
    IF_EQUAL_RETURN(guid, MF_MT_SAMPLE_SIZE);
    IF_EQUAL_RETURN(guid, MF_MT_WRAPPED_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_NUM_CHANNELS);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_SAMPLES_PER_SECOND);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_FLOAT_SAMPLES_PER_SECOND);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_AVG_BYTES_PER_SECOND);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_BLOCK_ALIGNMENT);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_BITS_PER_SAMPLE);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_VALID_BITS_PER_SAMPLE);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_SAMPLES_PER_BLOCK);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_CHANNEL_MASK);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_FOLDDOWN_MATRIX);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_WMADRC_PEAKREF);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_WMADRC_PEAKTARGET);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_WMADRC_AVGREF);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_WMADRC_AVGTARGET);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_PREFER_WAVEFORMATEX);
    IF_EQUAL_RETURN(guid, MF_MT_AAC_PAYLOAD_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_AAC_AUDIO_PROFILE_LEVEL_INDICATION);
    IF_EQUAL_RETURN(guid, MF_MT_FRAME_SIZE);
    IF_EQUAL_RETURN(guid, MF_MT_FRAME_RATE);
    IF_EQUAL_RETURN(guid, MF_MT_FRAME_RATE_RANGE_MAX);
    IF_EQUAL_RETURN(guid, MF_MT_FRAME_RATE_RANGE_MIN);
    IF_EQUAL_RETURN(guid, MF_MT_PIXEL_ASPECT_RATIO);
    IF_EQUAL_RETURN(guid, MF_MT_DRM_FLAGS);
    IF_EQUAL_RETURN(guid, MF_MT_PAD_CONTROL_FLAGS);
    IF_EQUAL_RETURN(guid, MF_MT_SOURCE_CONTENT_HINT);
    IF_EQUAL_RETURN(guid, MF_MT_VIDEO_CHROMA_SITING);
    IF_EQUAL_RETURN(guid, MF_MT_INTERLACE_MODE);
    IF_EQUAL_RETURN(guid, MF_MT_TRANSFER_FUNCTION);
    IF_EQUAL_RETURN(guid, MF_MT_VIDEO_PRIMARIES);
    IF_EQUAL_RETURN(guid, MF_MT_CUSTOM_VIDEO_PRIMARIES);
    IF_EQUAL_RETURN(guid, MF_MT_YUV_MATRIX);
    IF_EQUAL_RETURN(guid, MF_MT_VIDEO_LIGHTING);
    IF_EQUAL_RETURN(guid, MF_MT_VIDEO_NOMINAL_RANGE);
    IF_EQUAL_RETURN(guid, MF_MT_GEOMETRIC_APERTURE);
    IF_EQUAL_RETURN(guid, MF_MT_MINIMUM_DISPLAY_APERTURE);
    IF_EQUAL_RETURN(guid, MF_MT_PAN_SCAN_APERTURE);
    IF_EQUAL_RETURN(guid, MF_MT_PAN_SCAN_ENABLED);
    IF_EQUAL_RETURN(guid, MF_MT_AVG_BITRATE);
    IF_EQUAL_RETURN(guid, MF_MT_AVG_BIT_ERROR_RATE);
    IF_EQUAL_RETURN(guid, MF_MT_MAX_KEYFRAME_SPACING);
    IF_EQUAL_RETURN(guid, MF_MT_DEFAULT_STRIDE);
    IF_EQUAL_RETURN(guid, MF_MT_PALETTE);
    IF_EQUAL_RETURN(guid, MF_MT_USER_DATA);
    IF_EQUAL_RETURN(guid, MF_MT_AM_FORMAT_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG_START_TIME_CODE);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG2_PROFILE);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG2_LEVEL);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG2_FLAGS);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG_SEQUENCE_HEADER);
    IF_EQUAL_RETURN(guid, MF_MT_DV_AAUX_SRC_PACK_0);
    IF_EQUAL_RETURN(guid, MF_MT_DV_AAUX_CTRL_PACK_0);
    IF_EQUAL_RETURN(guid, MF_MT_DV_AAUX_SRC_PACK_1);
    IF_EQUAL_RETURN(guid, MF_MT_DV_AAUX_CTRL_PACK_1);
    IF_EQUAL_RETURN(guid, MF_MT_DV_VAUX_SRC_PACK);
    IF_EQUAL_RETURN(guid, MF_MT_DV_VAUX_CTRL_PACK);
    IF_EQUAL_RETURN(guid, MF_MT_ARBITRARY_HEADER);
    IF_EQUAL_RETURN(guid, MF_MT_ARBITRARY_FORMAT);
    IF_EQUAL_RETURN(guid, MF_MT_IMAGE_LOSS_TOLERANT);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG4_SAMPLE_DESCRIPTION);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG4_CURRENT_SAMPLE_ENTRY);
    IF_EQUAL_RETURN(guid, MF_MT_ORIGINAL_4CC);
    IF_EQUAL_RETURN(guid, MF_MT_ORIGINAL_WAVE_FORMAT_TAG);
    // Media types
    IF_EQUAL_RETURN(guid, MFMediaType_Audio);
    IF_EQUAL_RETURN(guid, MFMediaType_Video);
    IF_EQUAL_RETURN(guid, MFMediaType_Protected);
    IF_EQUAL_RETURN(guid, MFMediaType_SAMI);
    IF_EQUAL_RETURN(guid, MFMediaType_Script);
    IF_EQUAL_RETURN(guid, MFMediaType_Image);
    IF_EQUAL_RETURN(guid, MFMediaType_HTML);
    IF_EQUAL_RETURN(guid, MFMediaType_Binary);
    IF_EQUAL_RETURN(guid, MFMediaType_FileTransfer);
    IF_EQUAL_RETURN(guid, MFVideoFormat_AI44); //     FCC('AI44')
    IF_EQUAL_RETURN(guid, MFVideoFormat_ARGB32); //   D3DFMT_A8R8G8B8
    IF_EQUAL_RETURN(guid, MFVideoFormat_AYUV); //     FCC('AYUV')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DV25); //     FCC('dv25')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DV50); //     FCC('dv50')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DVH1); //     FCC('dvh1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DVSD); //     FCC('dvsd')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DVSL); //     FCC('dvsl')
    IF_EQUAL_RETURN(guid, MFVideoFormat_H264); //     FCC('H264')
    IF_EQUAL_RETURN(guid, MFVideoFormat_I420); //     FCC('I420')
    IF_EQUAL_RETURN(guid, MFVideoFormat_IYUV); //     FCC('IYUV')
    IF_EQUAL_RETURN(guid, MFVideoFormat_M4S2); //     FCC('M4S2')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MJPG);
    IF_EQUAL_RETURN(guid, MFVideoFormat_MP43); //     FCC('MP43')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MP4S); //     FCC('MP4S')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MP4V); //     FCC('MP4V')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MPG1); //     FCC('MPG1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MSS1); //     FCC('MSS1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MSS2); //     FCC('MSS2')
    IF_EQUAL_RETURN(guid, MFVideoFormat_NV11); //     FCC('NV11')
    IF_EQUAL_RETURN(guid, MFVideoFormat_NV12); //     FCC('NV12')
    IF_EQUAL_RETURN(guid, MFVideoFormat_P010); //     FCC('P010')
    IF_EQUAL_RETURN(guid, MFVideoFormat_P016); //     FCC('P016')
    IF_EQUAL_RETURN(guid, MFVideoFormat_P210); //     FCC('P210')
    IF_EQUAL_RETURN(guid, MFVideoFormat_P216); //     FCC('P216')
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB24); //    D3DFMT_R8G8B8
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB32); //    D3DFMT_X8R8G8B8
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB555); //   D3DFMT_X1R5G5B5
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB565); //   D3DFMT_R5G6B5
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB8);
    IF_EQUAL_RETURN(guid, MFVideoFormat_UYVY); //     FCC('UYVY')
    IF_EQUAL_RETURN(guid, MFVideoFormat_v210); //     FCC('v210')
    IF_EQUAL_RETURN(guid, MFVideoFormat_v410); //     FCC('v410')
    IF_EQUAL_RETURN(guid, MFVideoFormat_WMV1); //     FCC('WMV1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_WMV2); //     FCC('WMV2')
    IF_EQUAL_RETURN(guid, MFVideoFormat_WMV3); //     FCC('WMV3')
    IF_EQUAL_RETURN(guid, MFVideoFormat_WVC1); //     FCC('WVC1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y210); //     FCC('Y210')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y216); //     FCC('Y216')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y410); //     FCC('Y410')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y416); //     FCC('Y416')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y41P);
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y41T);
    IF_EQUAL_RETURN(guid, MFVideoFormat_YUY2); //     FCC('YUY2')
    IF_EQUAL_RETURN(guid, MFVideoFormat_YV12); //     FCC('YV12')
    IF_EQUAL_RETURN(guid, MFVideoFormat_YVYU);
    IF_EQUAL_RETURN(guid, MFAudioFormat_PCM); //              WAVE_FORMAT_PCM
    IF_EQUAL_RETURN(guid, MFAudioFormat_Float); //            WAVE_FORMAT_IEEE_FLOAT
    IF_EQUAL_RETURN(guid, MFAudioFormat_DTS); //              WAVE_FORMAT_DTS
    IF_EQUAL_RETURN(guid, MFAudioFormat_Dolby_AC3_SPDIF); //  WAVE_FORMAT_DOLBY_AC3_SPDIF
    IF_EQUAL_RETURN(guid, MFAudioFormat_DRM); //              WAVE_FORMAT_DRM
    IF_EQUAL_RETURN(guid, MFAudioFormat_WMAudioV8); //        WAVE_FORMAT_WMAUDIO2
    IF_EQUAL_RETURN(guid, MFAudioFormat_WMAudioV9); //        WAVE_FORMAT_WMAUDIO3
    IF_EQUAL_RETURN(guid, MFAudioFormat_WMAudio_Lossless); // WAVE_FORMAT_WMAUDIO_LOSSLESS
    IF_EQUAL_RETURN(guid, MFAudioFormat_WMASPDIF); //         WAVE_FORMAT_WMASPDIF
    IF_EQUAL_RETURN(guid, MFAudioFormat_MSP1); //             WAVE_FORMAT_WMAVOICE9
    IF_EQUAL_RETURN(guid, MFAudioFormat_MP3); //              WAVE_FORMAT_MPEGLAYER3
    IF_EQUAL_RETURN(guid, MFAudioFormat_MPEG); //             WAVE_FORMAT_MPEG
    IF_EQUAL_RETURN(guid, MFAudioFormat_AAC); //              WAVE_FORMAT_MPEG_HEAAC
    IF_EQUAL_RETURN(guid, MFAudioFormat_ADTS); //             WAVE_FORMAT_MPEG_ADTS_AAC
    return NULL;
}

FormatReader::FormatReader(void)
{
}

MediaType FormatReader::Read(IMFMediaType *pType)
{
    UINT32 count = 0;
    MediaType out;
    HRESULT hr = pType->LockStore();
    if (FAILED(hr))
    {
        return out;
    }
    hr = pType->GetCount(&count);
    if (FAILED(hr))
    {
        return out;
    }
    for (UINT32 i = 0; i < count; i++)
    {
        hr = LogAttributeValueByIndexNew(pType, i, out);
        if (FAILED(hr))
        {
            break;
        }
    }
    hr = pType->UnlockStore();
    if (FAILED(hr))
    {
        return out;
    }
    return out;
}

FormatReader::~FormatReader(void)
{
}

#define CHECK_HR(x) if (FAILED(x)) { goto done; }

ImageGrabberCallback::ImageGrabberCallback(bool synchronous):
    m_cRef(1),
    ig_RIE(true),
    ig_Close(false),
    ig_Synchronous(synchronous),
    ig_hFrameReady(synchronous ? CreateEvent(NULL, FALSE, FALSE, NULL): 0),
    ig_hFrameGrabbed(synchronous ? CreateEvent(NULL, FALSE, TRUE, NULL): 0),
    ig_hFinish(CreateEvent(NULL, TRUE, FALSE, NULL))
{}

ImageGrabber::ImageGrabber(unsigned int deviceID, bool synchronous):
    ImageGrabberCallback(synchronous),
    ig_DeviceID(deviceID),
    ig_pSource(NULL),
    ig_pSession(NULL),
    ig_pTopology(NULL),
	ig_funcNewFrame(NULL),
	ig_userDataNewFrame(NULL)
{}

ImageGrabber::~ImageGrabber(void)
{
    if (ig_pSession)
    {
        ig_pSession->Shutdown();
    }

    CloseHandle(ig_hFinish);

    if (ig_Synchronous)
    {
        CloseHandle(ig_hFrameReady);
        CloseHandle(ig_hFrameGrabbed);
    }

    SafeRelease(&ig_pSession);
    SafeRelease(&ig_pTopology);

    DebugPrintOut(L"IMAGEGRABBER VIDEODEVICE %i: Destroying instance of the ImageGrabber class\n", ig_DeviceID);
}


HRESULT ImageGrabber::initImageGrabber(IMFMediaSource *pSource)
{
    _ComPtr<IMFActivate> pSinkActivate = NULL;
    _ComPtr<IMFMediaType> pType = NULL;
    _ComPtr<IMFPresentationDescriptor> pPD = NULL;
    _ComPtr<IMFStreamDescriptor> pSD = NULL;
    _ComPtr<IMFMediaTypeHandler> pHandler = NULL;
    _ComPtr<IMFMediaType> pCurrentType = NULL;
    MediaType MT;
     // Clean up.
    if (ig_pSession)
    {
        ig_pSession->Shutdown();
    }
    SafeRelease(&ig_pSession);
    SafeRelease(&ig_pTopology);
    ig_pSource = pSource;
    HRESULT hr = pSource->CreatePresentationDescriptor(&pPD);
    if (FAILED(hr))
    {
        goto err;
    }
    BOOL fSelected;
    hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);
    if (FAILED(hr)) {
        goto err;
    }
    hr = pSD->GetMediaTypeHandler(&pHandler);
    if (FAILED(hr)) {
        goto err;
    }
    DWORD cTypes = 0;
    hr = pHandler->GetMediaTypeCount(&cTypes);
    if (FAILED(hr)) {
        goto err;
    }
    if(cTypes > 0)
    {
        hr = pHandler->GetCurrentMediaType(&pCurrentType);
        if (FAILED(hr)) {
            goto err;
        }
        MT = FormatReader::Read(pCurrentType.Get());
    }
err:
    CHECK_HR(hr);
    CHECK_HR(hr = RawImage::CreateInstance(&ig_RIFirst, MT.MF_MT_SAMPLE_SIZE));
    CHECK_HR(hr = RawImage::CreateInstance(&ig_RISecond, MT.MF_MT_SAMPLE_SIZE));
    ig_RIOut = ig_RISecond;
    // Configure the media type that the Sample Grabber will receive.
    // Setting the major and subtype is usually enough for the topology loader
    // to resolve the topology.
    CHECK_HR(hr = MFCreateMediaType(pType.GetAddressOf()));
    CHECK_HR(hr = pType->SetGUID(MF_MT_MAJOR_TYPE, MT.MF_MT_MAJOR_TYPE));
    CHECK_HR(hr = pType->SetGUID(MF_MT_SUBTYPE, MT.MF_MT_SUBTYPE));
    // Create the sample grabber sink.
    CHECK_HR(hr = MFCreateSampleGrabberSinkActivate(pType.Get(), this, pSinkActivate.GetAddressOf()));
    // To run as fast as possible, set this attribute (requires Windows 7):
    CHECK_HR(hr = pSinkActivate->SetUINT32(MF_SAMPLEGRABBERSINK_IGNORE_CLOCK, TRUE));
    // Create the Media Session.
    CHECK_HR(hr = MFCreateMediaSession(NULL, &ig_pSession));
    // Create the topology.
    CHECK_HR(hr = CreateTopology(pSource, pSinkActivate.Get(), &ig_pTopology));
done:
    // Clean up.
    if (FAILED(hr))
    {
        if (ig_pSession)
        {
            ig_pSession->Shutdown();
        }
        SafeRelease(&ig_pSession);
        SafeRelease(&ig_pTopology);
    }
    return hr;
}

void ImageGrabber::stopGrabbing()
{
    if(ig_pSession)
        ig_pSession->Stop();
    DebugPrintOut(L"IMAGEGRABBER VIDEODEVICE %i: Stopping of of grabbing of images\n", ig_DeviceID);
}

HRESULT ImageGrabber::startGrabbing(void)
{
    PROPVARIANT var;
    PropVariantInit(&var);
    HRESULT hr = ig_pSession->SetTopology(0, ig_pTopology);
    DebugPrintOut(L"IMAGEGRABBER VIDEODEVICE %i: Start Grabbing of the images\n", ig_DeviceID);
    hr = ig_pSession->Start(&GUID_NULL, &var);
    for(;;)
    {
        _ComPtr<IMFMediaEvent> pEvent = NULL;
        HRESULT hrStatus = S_OK;
        MediaEventType met;
        if(!ig_pSession) break;
        hr = ig_pSession->GetEvent(0, &pEvent);
        if(!SUCCEEDED(hr))
        {
            hr = S_OK;
            goto done;
        }
        hr = pEvent->GetStatus(&hrStatus);
        if(!SUCCEEDED(hr))
        {
            hr = S_OK;
            goto done;
        }
        hr = pEvent->GetType(&met);
        if(!SUCCEEDED(hr))
        {
            hr = S_OK;
            goto done;
        }
        if (met == MESessionEnded)
        {
            DebugPrintOut(L"IMAGEGRABBER VIDEODEVICE %i: MESessionEnded\n", ig_DeviceID);
            ig_pSession->Stop();
            break;
        }
        if (met == MESessionStopped)
        {
            DebugPrintOut(L"IMAGEGRABBER VIDEODEVICE %i: MESessionStopped \n", ig_DeviceID);
            break;
        }
#if (WINVER >= 0x0602) // Available since Win 8
        if (met == MEVideoCaptureDeviceRemoved)
        {
            DebugPrintOut(L"IMAGEGRABBER VIDEODEVICE %i: MEVideoCaptureDeviceRemoved \n", ig_DeviceID);
            break;
        }
#endif
        if ((met == MEError) || (met == MENonFatalError))
        {
            pEvent->GetStatus(&hrStatus);
            DebugPrintOut(L"IMAGEGRABBER VIDEODEVICE %i: MEError | MENonFatalError: %u\n", ig_DeviceID, hrStatus);
            break;
        }
    }
    DebugPrintOut(L"IMAGEGRABBER VIDEODEVICE %i: Finish startGrabbing \n", ig_DeviceID);

done:
    SetEvent(ig_hFinish);

    return hr;
}

void ImageGrabberCallback::pauseGrabbing()
{
}

void ImageGrabberCallback::resumeGrabbing()
{
}

HRESULT ImageGrabber::CreateTopology(IMFMediaSource *pSource, IMFActivate *pSinkActivate, IMFTopology **ppTopo)
{
    IMFTopology* pTopology = NULL;
    _ComPtr<IMFPresentationDescriptor> pPD = NULL;
    _ComPtr<IMFStreamDescriptor> pSD = NULL;
    _ComPtr<IMFMediaTypeHandler> pHandler = NULL;
    _ComPtr<IMFTopologyNode> pNode1 = NULL;
    _ComPtr<IMFTopologyNode> pNode2 = NULL;
    HRESULT hr = S_OK;
    DWORD cStreams = 0;
    CHECK_HR(hr = MFCreateTopology(&pTopology));
    CHECK_HR(hr = pSource->CreatePresentationDescriptor(pPD.GetAddressOf()));
    CHECK_HR(hr = pPD->GetStreamDescriptorCount(&cStreams));
    for (DWORD i = 0; i < cStreams; i++)
    {
        // In this example, we look for audio streams and connect them to the sink.
        BOOL fSelected = FALSE;
        GUID majorType;
        CHECK_HR(hr = pPD->GetStreamDescriptorByIndex(i, &fSelected, &pSD));
        CHECK_HR(hr = pSD->GetMediaTypeHandler(&pHandler));
        CHECK_HR(hr = pHandler->GetMajorType(&majorType));
        if (majorType == MFMediaType_Video && fSelected)
        {
            CHECK_HR(hr = AddSourceNode(pTopology, pSource, pPD.Get(), pSD.Get(), pNode1.GetAddressOf()));
            CHECK_HR(hr = AddOutputNode(pTopology, pSinkActivate, 0, pNode2.GetAddressOf()));
            CHECK_HR(hr = pNode1->ConnectOutput(0, pNode2.Get(), 0));
            break;
        }
        else
        {
            CHECK_HR(hr = pPD->DeselectStream(i));
        }
    }
    *ppTopo = pTopology;
    (*ppTopo)->AddRef();

done:
    return hr;
}

HRESULT ImageGrabber::AddSourceNode(
    IMFTopology *pTopology,           // Topology.
    IMFMediaSource *pSource,          // Media source.
    IMFPresentationDescriptor *pPD,   // Presentation descriptor.
    IMFStreamDescriptor *pSD,         // Stream descriptor.
    IMFTopologyNode **ppNode)         // Receives the node pointer.
{
    _ComPtr<IMFTopologyNode> pNode = NULL;
    HRESULT hr = S_OK;
    CHECK_HR(hr = MFCreateTopologyNode(MF_TOPOLOGY_SOURCESTREAM_NODE, pNode.GetAddressOf()));
    CHECK_HR(hr = pNode->SetUnknown(MF_TOPONODE_SOURCE, pSource));
    CHECK_HR(hr = pNode->SetUnknown(MF_TOPONODE_PRESENTATION_DESCRIPTOR, pPD));
    CHECK_HR(hr = pNode->SetUnknown(MF_TOPONODE_STREAM_DESCRIPTOR, pSD));
    CHECK_HR(hr = pTopology->AddNode(pNode.Get()));
    // Return the pointer to the caller.
    *ppNode = pNode.Get();
    (*ppNode)->AddRef();

done:
    return hr;
}

HRESULT ImageGrabber::AddOutputNode(
    IMFTopology *pTopology,     // Topology.
    IMFActivate *pActivate,     // Media sink activation object.
    DWORD dwId,                 // Identifier of the stream sink.
    IMFTopologyNode **ppNode)   // Receives the node pointer.
{
    _ComPtr<IMFTopologyNode> pNode = NULL;
    HRESULT hr = S_OK;
    CHECK_HR(hr = MFCreateTopologyNode(MF_TOPOLOGY_OUTPUT_NODE, pNode.GetAddressOf()));
    CHECK_HR(hr = pNode->SetObject(pActivate));
    CHECK_HR(hr = pNode->SetUINT32(MF_TOPONODE_STREAMID, dwId));
    CHECK_HR(hr = pNode->SetUINT32(MF_TOPONODE_NOSHUTDOWN_ON_REMOVE, FALSE));
    CHECK_HR(hr = pTopology->AddNode(pNode.Get()));
    // Return the pointer to the caller.
    *ppNode = pNode.Get();
    (*ppNode)->AddRef();

done:
    return hr;
}

HRESULT ImageGrabber::CreateInstance(ImageGrabber **ppIG, unsigned int deviceID, bool synchronious)
{
    *ppIG = new (std::nothrow) ImageGrabber(deviceID, synchronious);
    if (ppIG == NULL)
    {
        return E_OUTOFMEMORY;
    }
    DebugPrintOut(L"IMAGEGRABBER VIDEODEVICE %i: Creating instance of ImageGrabber\n", deviceID);
    return S_OK;
}

void ImageGrabber::setNewFrameAvailableEvent(void *userData, void(*func)(int, void *, LONGLONG)) {
	if (func)
	{
		ig_funcNewFrame = func;
		ig_userDataNewFrame = userData;
	}
}

void ImageGrabber::onNewFrameAvailable(LONGLONG ts) {
	if (ig_funcNewFrame)
	{
		ig_funcNewFrame(ig_DeviceID, ig_userDataNewFrame, ts);
	}
}


STDMETHODIMP ImageGrabber::QueryInterface(REFIID riid, void** ppv)
{
    HRESULT hr = E_NOINTERFACE;
    *ppv = NULL;
    if(riid == IID_IUnknown || riid == IID_IMFSampleGrabberSinkCallback)
    {
        *ppv = static_cast<IMFSampleGrabberSinkCallback *>(this);
        hr = S_OK;
    }
    if(riid == IID_IMFClockStateSink)
    {
        *ppv = static_cast<IMFClockStateSink *>(this);
        hr = S_OK;
    }
    if(SUCCEEDED(hr))
    {
        reinterpret_cast<IUnknown *>(*ppv)->AddRef();
    }
    return hr;
}

STDMETHODIMP_(ULONG) ImageGrabber::AddRef()
{
    return InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) ImageGrabber::Release()
{
    ULONG cRef = InterlockedDecrement(&m_cRef);
    if (cRef == 0)
    {
        delete this;
    }
    return cRef;
}

STDMETHODIMP ImageGrabberCallback::OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
{
    (void)hnsSystemTime;
    (void)llClockStartOffset;
    return S_OK;
}

STDMETHODIMP ImageGrabberCallback::OnClockStop(MFTIME hnsSystemTime)
{
    (void)hnsSystemTime;
    return S_OK;
}

STDMETHODIMP ImageGrabberCallback::OnClockPause(MFTIME hnsSystemTime)
{
    (void)hnsSystemTime;
    return S_OK;
}

STDMETHODIMP ImageGrabberCallback::OnClockRestart(MFTIME hnsSystemTime)
{
    (void)hnsSystemTime;
    return S_OK;
}

STDMETHODIMP ImageGrabberCallback::OnClockSetRate(MFTIME hnsSystemTime, float flRate)
{
    (void)flRate;
    (void)hnsSystemTime;
    return S_OK;
}

STDMETHODIMP ImageGrabberCallback::OnSetPresentationClock(IMFPresentationClock* pClock)
{
    (void)pClock;
    return S_OK;
}

STDMETHODIMP ImageGrabberCallback::OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
    LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE * pSampleBuffer,
    DWORD dwSampleSize)
{
    (void)guidMajorMediaType;
    (void)llSampleTime;
    (void)dwSampleFlags;
    (void)llSampleDuration;
    (void)dwSampleSize;

    HANDLE tmp[] = {ig_hFinish, ig_hFrameGrabbed, NULL};

    DWORD status = WaitForMultipleObjects(2, tmp, FALSE, INFINITE);
    if (status == WAIT_OBJECT_0)
    {
        DebugPrintOut(L"OnProcessFrame called after ig_hFinish event\n");
        return S_OK;
    }

    if(ig_RIE)
    {
        ig_RIFirst->fastCopy(pSampleBuffer);
        ig_RIOut = ig_RIFirst;
    }
    else
    {
        ig_RISecond->fastCopy(pSampleBuffer);
        ig_RIOut = ig_RISecond;
    }

    if (ig_Synchronous)
    {
        SetEvent(ig_hFrameReady);
    }
    else
    {
        ig_RIE = !ig_RIE;
		onNewFrameAvailable(llSampleTime);
	}

    return S_OK;
}

void ImageGrabberCallback::onNewFrameAvailable(LONGLONG ts)
{
}



STDMETHODIMP ImageGrabberCallback::OnShutdown()
{
    SetEvent(ig_hFinish);
    return S_OK;
}

RawImage *ImageGrabberCallback::getRawImage()
{
    return ig_RIOut;
}

DWORD WINAPI MainThreadFunction( LPVOID lpParam )
{
    ImageGrabberThread *pIGT = (ImageGrabberThread *)lpParam;
    pIGT->run();
    return 0;
}

HRESULT ImageGrabberThread::CreateInstance(ImageGrabberThread **ppIGT, IMFMediaSource *pSource, unsigned int deviceID, bool synchronious)
{
    *ppIGT = new (std::nothrow) ImageGrabberThread(pSource, deviceID, synchronious);
    if (ppIGT == NULL)
    {
        DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: Memory cannot be allocated\n", deviceID);
        return E_OUTOFMEMORY;
    }
    else
        DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: Creating of the instance of ImageGrabberThread\n", deviceID);
    return S_OK;
}

ImageGrabberThread::ImageGrabberThread(IMFMediaSource *pSource, unsigned int deviceID, bool synchronious) :
    igt_func(NULL),
    igt_Handle(NULL),
    igt_stop(false)
{
    HRESULT hr = ImageGrabber::CreateInstance(&igt_pImageGrabber, deviceID, synchronious);
    igt_DeviceID = deviceID;
    if(SUCCEEDED(hr))
    {
        hr = igt_pImageGrabber->initImageGrabber(pSource);
        if(!SUCCEEDED(hr))
        {
            DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: There is a problem with initialization of the instance of the ImageGrabber class\n", deviceID);
        }
        else
        {
            DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: Initialization of instance of the ImageGrabber class\n", deviceID);
        }
    }
    else
    {
        DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: There is a problem with creation of the instance of the ImageGrabber class\n", deviceID);
    }
}

void ImageGrabberThread::setEmergencyStopEvent(void *userData, void(*func)(int, void *))
{
    if(func)
    {
        igt_func = func;
        igt_userData = userData;
    }
}

ImageGrabberThread::~ImageGrabberThread(void)
{
    DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: Destroing ImageGrabberThread\n", igt_DeviceID);
    if (igt_Handle)
        WaitForSingleObject(igt_Handle, INFINITE);
    delete igt_pImageGrabber;
}

void ImageGrabberThread::stop()
{
    igt_stop = true;
    if(igt_pImageGrabber)
    {
        igt_pImageGrabber->stopGrabbing();
    }
}

void ImageGrabberThread::start()
{
    igt_Handle = CreateThread(
            NULL,                  // default security attributes
            0,                     // use default stack size
            MainThreadFunction,    // thread function name
            this,                  // argument to thread function
            0,                     // use default creation flags
            &igt_ThreadIdArray);   // returns the thread identifier
}

void ImageGrabberThread::run()
{
    if(igt_pImageGrabber)
    {
        DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: Thread for grabbing images is started\n", igt_DeviceID);
        HRESULT hr = igt_pImageGrabber->startGrabbing();
        if(!SUCCEEDED(hr))
        {
            DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: There is a problem with starting the process of grabbing\n", igt_DeviceID);
        }
    }
    else
    {
        DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i The thread is finished without execution of grabbing\n", igt_DeviceID);
    }
    if(!igt_stop)
    {
        DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: Emergency Stop thread\n", igt_DeviceID);
        if(igt_func)
        {
            igt_func(igt_DeviceID, igt_userData);
        }
    }
    else
        DebugPrintOut(L"IMAGEGRABBERTHREAD VIDEODEVICE %i: Finish thread\n", igt_DeviceID);
}

ImageGrabber *ImageGrabberThread::getImageGrabber()
{
    return igt_pImageGrabber;
}

Media_Foundation::Media_Foundation(void)
{
    HRESULT hr = MFStartup(MF_VERSION);
    if(!SUCCEEDED(hr))
    {
        DebugPrintOut(L"MEDIA FOUNDATION: It cannot be created!!!\n");
    }
}

Media_Foundation::~Media_Foundation(void)
{
    HRESULT hr = MFShutdown();
    if(!SUCCEEDED(hr))
    {
        DebugPrintOut(L"MEDIA FOUNDATION: Resources cannot be released\n");
    }
}

bool Media_Foundation::buildListOfDevices()
{
    HRESULT hr = S_OK;
    _ComPtr<IMFAttributes> pAttributes = NULL;
    CoInitialize(NULL);
    hr = MFCreateAttributes(pAttributes.GetAddressOf(), 1);
    if (SUCCEEDED(hr))
    {
        hr = pAttributes->SetGUID(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
            );
    }
    if (SUCCEEDED(hr))
    {
        videoDevices *vDs = &videoDevices::getInstance();
        hr = vDs->initDevices(pAttributes.Get());
    }
    if (FAILED(hr))
    {
       DebugPrintOut(L"MEDIA FOUNDATION: The access to the video cameras denied\n");
    }

    return (SUCCEEDED(hr));
}

Media_Foundation& Media_Foundation::getInstance()
{
    static Media_Foundation instance;
    return instance;
}

RawImage::RawImage(unsigned int size): ri_new(false), ri_pixels(NULL)
{
    ri_size = size;
    ri_pixels = new unsigned char[size];
    memset((void *)ri_pixels,0,ri_size);
}

bool RawImage::isNew()
{
    return ri_new;
}

unsigned int RawImage::getSize()
{
    return ri_size;
}

RawImage::~RawImage(void)
{
    delete []ri_pixels;
    ri_pixels = NULL;
}

long RawImage::CreateInstance(RawImage **ppRImage,unsigned int size)
{
    *ppRImage = new (std::nothrow) RawImage(size);
    if (ppRImage == NULL)
    {
        return E_OUTOFMEMORY;
    }
    return S_OK;
}

void RawImage::setCopy(const BYTE * pSampleBuffer)
{
    memcpy(ri_pixels, pSampleBuffer, ri_size);
    ri_new = true;
}

void RawImage::fastCopy(const BYTE * pSampleBuffer)
{
    memcpy(ri_pixels, pSampleBuffer, ri_size);
    ri_new = true;
}

unsigned char * RawImage::getpPixels()
{
    ri_new = false;
    return ri_pixels;
}










videoDevice::videoDevice(void): vd_IsSetuped(false), vd_LockOut(OpenLock), vd_pFriendlyName(NULL),
    vd_Width(0), vd_Height(0), vd_FrameRate(0), vd_pSource(NULL), vd_pImGrTh(NULL), vd_func(NULL), vd_userData(NULL)
{
}

void videoDevice::setParametrs(CamParametrs parametrs)
{
    if(vd_IsSetuped)
    {
        if(vd_pSource)
        {
            Parametr *pParametr = (Parametr *)(&parametrs);
            Parametr *pPrevParametr = (Parametr *)(&vd_PrevParametrs);
            IAMVideoProcAmp *pProcAmp = NULL;
            HRESULT hr = vd_pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));
            if (SUCCEEDED(hr))
            {
                for(unsigned int i = 0; i < 10; i++)
                {
                    if(pPrevParametr[i].CurrentValue != pParametr[i].CurrentValue || pPrevParametr[i].Flag != pParametr[i].Flag)
                        hr = pProcAmp->Set(VideoProcAmp_Brightness + i, pParametr[i].CurrentValue, pParametr[i].Flag);
                }
                pProcAmp->Release();
            }
            IAMCameraControl *pProcControl = NULL;
            hr = vd_pSource->QueryInterface(IID_PPV_ARGS(&pProcControl));
            if (SUCCEEDED(hr))
            {
                for(unsigned int i = 0; i < 7; i++)
                {
                    if(pPrevParametr[10 + i].CurrentValue != pParametr[10 + i].CurrentValue || pPrevParametr[10 + i].Flag != pParametr[10 + i].Flag)
                    hr = pProcControl->Set(CameraControl_Pan+i, pParametr[10 + i].CurrentValue, pParametr[10 + i].Flag);
                }
                pProcControl->Release();
            }
            vd_PrevParametrs = parametrs;
        }
    }
}

CamParametrs videoDevice::getParametrs()
{
    CamParametrs out;
    if(vd_IsSetuped)
    {
        if(vd_pSource)
        {
            Parametr *pParametr = (Parametr *)(&out);
            IAMVideoProcAmp *pProcAmp = NULL;
            HRESULT hr = vd_pSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));
            if (SUCCEEDED(hr))
            {
                for(unsigned int i = 0; i < 10; i++)
                {
                    Parametr temp;
                    hr = pProcAmp->GetRange(VideoProcAmp_Brightness+i, &temp.Min, &temp.Max, &temp.Step, &temp.Default, &temp.Flag);
                    if (SUCCEEDED(hr))
                    {
                        temp.CurrentValue = temp.Default;
                        pParametr[i] = temp;
                    }
                }
                pProcAmp->Release();
            }
            IAMCameraControl *pProcControl = NULL;
            hr = vd_pSource->QueryInterface(IID_PPV_ARGS(&pProcControl));
            if (SUCCEEDED(hr))
            {
                for(unsigned int i = 0; i < 7; i++)
                {
                    Parametr temp;
                    hr = pProcControl->GetRange(CameraControl_Pan+i, &temp.Min, &temp.Max, &temp.Step, &temp.Default, &temp.Flag);
                    if (SUCCEEDED(hr))
                    {
                        temp.CurrentValue = temp.Default;
                        pParametr[10 + i] = temp;
                    }
                }
                pProcControl->Release();
            }
        }
    }
    return out;
}

long videoDevice::resetDevice(IMFActivate *pActivate)
{
    HRESULT hr = E_FAIL;
    vd_CurrentFormats.clear();
    if(vd_pFriendlyName)
        CoTaskMemFree(vd_pFriendlyName);
    vd_pFriendlyName = NULL;

    if(pActivate)
    {
        IMFMediaSource *pSource = NULL;
        hr = pActivate->GetAllocatedString(
                MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
                &vd_pFriendlyName,
                NULL
                );
        hr = pActivate->ActivateObject(
            __uuidof(IMFMediaSource),
            (void**)&pSource
            );
        enumerateCaptureFormats(pSource);
        buildLibraryofTypes();
        SafeRelease(&pSource);
        if(FAILED(hr))
        {
            vd_pFriendlyName = NULL;
            DebugPrintOut(L"VIDEODEVICE %i: IMFMediaSource interface cannot be created \n", vd_CurrentNumber);
        }
    }
    return hr;
}

long videoDevice::readInfoOfDevice(IMFActivate *pActivate, unsigned int Num)
{
    vd_CurrentNumber = Num;
    return resetDevice(pActivate);
}


long videoDevice::checkDevice(IMFAttributes *pAttributes, IMFActivate **pDevice)
{
    IMFActivate **ppDevices = NULL;
    UINT32 count;
    wchar_t *newFriendlyName = NULL;
    HRESULT hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);
    if (SUCCEEDED(hr))
    {
        if(count > 0)
        {
            if(count > vd_CurrentNumber)
            {
                hr = ppDevices[vd_CurrentNumber]->GetAllocatedString(
                MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
                &newFriendlyName,
                NULL
                );
                if (SUCCEEDED(hr))
                {
                    if(wcscmp(newFriendlyName, vd_pFriendlyName) != 0)
                    {
                        DebugPrintOut(L"VIDEODEVICE %i: Chosen device cannot be found \n", vd_CurrentNumber);
                        hr = E_INVALIDARG;
                        pDevice = NULL;
                    }
                    else
                    {
                        *pDevice = ppDevices[vd_CurrentNumber];
                        (*pDevice)->AddRef();
                    }
                }
                else
                {
                    DebugPrintOut(L"VIDEODEVICE %i: Name of device cannot be gotten \n", vd_CurrentNumber);
                }
            }
            else
            {
                DebugPrintOut(L"VIDEODEVICE %i: Number of devices more than corrent number of the device \n", vd_CurrentNumber);
                hr = E_INVALIDARG;
            }
            for(UINT32 i = 0; i < count; i++)
            {
                SafeRelease(&ppDevices[i]);
            }
            SafeRelease(ppDevices);
        }
        else
            hr = E_FAIL;
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE %i: List of DeviceSources cannot be enumerated \n", vd_CurrentNumber);
    }
    return hr;
}

long videoDevice::initDevice()
{
    HRESULT hr = S_OK;
    CoInitialize(NULL);
    _ComPtr<IMFAttributes> pAttributes = NULL;
    IMFActivate *vd_pActivate = NULL;
    hr = MFCreateAttributes(pAttributes.GetAddressOf(), 1);
    if (SUCCEEDED(hr))
    {
        hr = pAttributes->SetGUID(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
            );
    }
    if (SUCCEEDED(hr))
    {
        hr = checkDevice(pAttributes.Get(), &vd_pActivate);
        if (SUCCEEDED(hr) && vd_pActivate)
        {
            SafeRelease(&vd_pSource);
            hr = vd_pActivate->ActivateObject(
                __uuidof(IMFMediaSource),
                (void**)&vd_pSource
                );
            if (SUCCEEDED(hr))
            {
            }
            SafeRelease(&vd_pActivate);
        }
        else
        {
            DebugPrintOut(L"VIDEODEVICE %i: Device there is not \n", vd_CurrentNumber);
        }
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE %i: The attribute of video cameras cannot be getting \n", vd_CurrentNumber);
    }
    return hr;
}

MediaType videoDevice::getFormat(unsigned int id)
{
    if(id < vd_CurrentFormats.size())
    {
        return vd_CurrentFormats[id];
    }
    else return MediaType();
}

int videoDevice::getCountFormats()
{
    return (int)vd_CurrentFormats.size();
}

void videoDevice::setEmergencyStopEvent(void *userData, void(*func)(int, void *))
{
    vd_func = func;
    vd_userData = userData;
}

void videoDevice::setNewFrameAvailableEvent(void *userData, void(*func)(int, void *, LONGLONG))
{
	ImageGrabber *grabber = getImageGrabber();
	if (grabber) {
		grabber->setNewFrameAvailableEvent(userData, func);
	}
}

void videoDevice::closeDevice()
{
    if(vd_IsSetuped)
    {
        vd_IsSetuped = false;
        vd_pSource->Shutdown();
        SafeRelease(&vd_pSource);
        if(vd_LockOut == RawDataLock)
        {
            vd_pImGrTh->stop();
            Sleep(500);
            delete vd_pImGrTh;
        }
        vd_pImGrTh = NULL;
        vd_LockOut = OpenLock;
        DebugPrintOut(L"VIDEODEVICE %i: Device is stopped \n", vd_CurrentNumber);
    }
}
unsigned int videoDevice::getWidth()
{
    if(vd_IsSetuped)
        return vd_Width;
    else
        return 0;
}
unsigned int videoDevice::getHeight()
{
    if(vd_IsSetuped)
        return vd_Height;
    else
        return 0;
}

unsigned int videoDevice::getFrameRate() const
{
    if(vd_IsSetuped)
        return vd_FrameRate;
    else
        return 0;
}

IMFMediaSource *videoDevice::getMediaSource()
{
    IMFMediaSource *out = NULL;
    if(vd_LockOut == OpenLock)
    {
        vd_LockOut = MediaSourceLock;
        out = vd_pSource;
    }
    return out;
}
int videoDevice::findType(unsigned int size, unsigned int frameRate)
{
    // For required frame size look for the suitable video format.
    // If not found, get the format for the largest available frame size.
    FrameRateMap FRM;
    std::map<UINT64, FrameRateMap>::const_iterator fmt;
    fmt = vd_CaptureFormats.find(size);
    if( fmt != vd_CaptureFormats.end() )
        FRM = fmt->second;
    else if( !vd_CaptureFormats.empty() )
        FRM = vd_CaptureFormats.rbegin()->second;

    if( FRM.empty() )
        return -1;

    UINT64 frameRateMax = 0;  SUBTYPEMap STMMax;
    if(frameRate == 0)
    {
        std::map<UINT64, SUBTYPEMap>::iterator f = FRM.begin();
        for(; f != FRM.end(); f++)
        {
            // Looking for highest possible frame rate.
             if((*f).first >= frameRateMax)
             {
                 frameRateMax = (*f).first;
                 STMMax = (*f).second;
             }
        }
    }
    else
    {
        std::map<UINT64, SUBTYPEMap>::iterator f = FRM.begin();
        for(; f != FRM.end(); f++)
        {
            // Looking for frame rate higher that recently found but not higher then demanded.
            if( (*f).first >= frameRateMax && (*f).first <= frameRate )
            {
                frameRateMax = (*f).first;
                STMMax = (*f).second;
            }
        }
    }
    // Get first (default) item from the list if no suitable frame rate found.
    if( STMMax.empty() )
        STMMax = FRM.begin()->second;

    // Check if there are any format types on the list.
    if( STMMax.empty() )
        return -1;

    vectorNum VN = STMMax.begin()->second;
    if( VN.empty() )
        return -1;

    return VN[0];
}

void videoDevice::buildLibraryofTypes()
{
    unsigned int size;
    unsigned int framerate;
    std::vector<MediaType>::iterator i = vd_CurrentFormats.begin();
    int count = 0;
    for(; i != vd_CurrentFormats.end(); i++)
    {
        // Count only supported video formats.
        if( (*i).MF_MT_SUBTYPE == MFVideoFormat_RGB24 )
        {
            size = (*i).MF_MT_FRAME_SIZE;
            framerate = (*i).MF_MT_FRAME_RATE_NUMERATOR / (*i).MF_MT_FRAME_RATE_DENOMINATOR;
            FrameRateMap FRM = vd_CaptureFormats[size];
            SUBTYPEMap STM = FRM[framerate];
            String subType((*i).pMF_MT_SUBTYPEName);
            vectorNum VN = STM[subType];
            VN.push_back(count);
            STM[subType] = VN;
            FRM[framerate] = STM;
            vd_CaptureFormats[size] = FRM;
        }
        count++;
    }
}

long videoDevice::setDeviceFormat(IMFMediaSource *pSource, unsigned long  dwFormatIndex)
{
    _ComPtr<IMFPresentationDescriptor> pPD = NULL;
    _ComPtr<IMFStreamDescriptor> pSD = NULL;
    _ComPtr<IMFMediaTypeHandler> pHandler = NULL;
    _ComPtr<IMFMediaType> pType = NULL;
    HRESULT hr = pSource->CreatePresentationDescriptor(pPD.GetAddressOf());
    if (FAILED(hr))
    {
        goto done;
    }
    BOOL fSelected;
    hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, pSD.GetAddressOf());
    if (FAILED(hr))
    {
        goto done;
    }
    hr = pSD->GetMediaTypeHandler(pHandler.GetAddressOf());
    if (FAILED(hr))
    {
        goto done;
    }
    hr = pHandler->GetMediaTypeByIndex((DWORD)dwFormatIndex, pType.GetAddressOf());
    if (FAILED(hr))
    {
        goto done;
    }
    hr = pHandler->SetCurrentMediaType(pType.Get());

done:
    return hr;
}

bool videoDevice::isDeviceSetup()
{
    return vd_IsSetuped;
}

RawImage * videoDevice::getRawImageOut()
{
    if(!vd_IsSetuped) return NULL;
    if(vd_pImGrTh)
            return vd_pImGrTh->getImageGrabber()->getRawImage();
    else
    {
        DebugPrintOut(L"VIDEODEVICE %i: The instance of ImageGrabberThread class does not exist  \n", vd_CurrentNumber);
    }
    return NULL;
}

bool videoDevice::isFrameNew()
{
    if(!vd_IsSetuped) return false;
    if(vd_LockOut == RawDataLock || vd_LockOut == OpenLock)
    {
        if(vd_LockOut == OpenLock)
        {
            vd_LockOut = RawDataLock;

            //must already be closed

            HRESULT hr = ImageGrabberThread::CreateInstance(&vd_pImGrTh, vd_pSource, vd_CurrentNumber);
            if(FAILED(hr))
            {
                DebugPrintOut(L"VIDEODEVICE %i: The instance of ImageGrabberThread class cannot be created.\n", vd_CurrentNumber);
                return false;
            }
            vd_pImGrTh->setEmergencyStopEvent(vd_userData, vd_func);
            vd_pImGrTh->start();
            return true;
        }

        if(vd_pImGrTh)
            return vd_pImGrTh->getImageGrabber()->getRawImage()->isNew();
    }
    return false;
}

bool videoDevice::isDeviceMediaSource()
{
    if(vd_LockOut == MediaSourceLock) return true;
    return false;
}

ImageGrabber *videoDevice::getImageGrabber() {
	if (vd_pImGrTh)
		return vd_pImGrTh->getImageGrabber();
	return NULL;
}

bool videoDevice::isDeviceRawDataSource()
{
    if(vd_LockOut == RawDataLock) return true;
    return false;
}

bool videoDevice::setupDevice(unsigned int id)
{
    if(!vd_IsSetuped)
    {
        HRESULT hr = initDevice();
        if(SUCCEEDED(hr))
        {

            vd_Width = vd_CurrentFormats[id].width;
            vd_Height = vd_CurrentFormats[id].height;
            vd_FrameRate = vd_CurrentFormats[id].MF_MT_FRAME_RATE_NUMERATOR /
                           vd_CurrentFormats[id].MF_MT_FRAME_RATE_DENOMINATOR;

            hr = setDeviceFormat(vd_pSource, (DWORD) id);
            vd_IsSetuped = (SUCCEEDED(hr));
            if(vd_IsSetuped)
                DebugPrintOut(L"\n\nVIDEODEVICE %i: Device is setuped \n", vd_CurrentNumber);
            vd_PrevParametrs = getParametrs();

            return vd_IsSetuped;
        }
        else
        {
            DebugPrintOut(L"VIDEODEVICE %i: Interface IMFMediaSource cannot be got \n", vd_CurrentNumber);
            return false;
        }
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE %i: Device is setuped already \n", vd_CurrentNumber);
        return false;
    }
}

bool videoDevice::setupDevice(unsigned int w, unsigned int h, unsigned int idealFramerate)
{
    unsigned int id = findType(w * h, idealFramerate);
    if( id < 0 )
        return false;

    return setupDevice(id);
}

wchar_t *videoDevice::getName()
{
    return vd_pFriendlyName;
}

videoDevice::~videoDevice(void)
{
    closeDevice();
    SafeRelease(&vd_pSource);
    if(vd_pFriendlyName)
        CoTaskMemFree(vd_pFriendlyName);
}

HRESULT videoDevice::enumerateCaptureFormats(IMFMediaSource *pSource)
{
    _ComPtr<IMFPresentationDescriptor> pPD = NULL;
    _ComPtr<IMFStreamDescriptor> pSD = NULL;
    _ComPtr<IMFMediaTypeHandler> pHandler = NULL;
    _ComPtr<IMFMediaType> pType = NULL;
    HRESULT hr = pSource->CreatePresentationDescriptor(pPD.GetAddressOf());
    if (FAILED(hr))
    {
        goto done;
    }
    BOOL fSelected;
    hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, pSD.GetAddressOf());
    if (FAILED(hr))
    {
        goto done;
    }
    hr = pSD->GetMediaTypeHandler(pHandler.GetAddressOf());
    if (FAILED(hr))
    {
        goto done;
    }
    DWORD cTypes = 0;
    hr = pHandler->GetMediaTypeCount(&cTypes);
    if (FAILED(hr))
    {
        goto done;
    }
    for (DWORD i = 0; i < cTypes; i++)
    {
        hr = pHandler->GetMediaTypeByIndex(i, pType.GetAddressOf());
        if (FAILED(hr))
        {
            goto done;
        }
        MediaType MT = FormatReader::Read(pType.Get());
        vd_CurrentFormats.push_back(MT);
    }

done:
    return hr;
}

videoDevices::videoDevices(void): count(0)
{

}

void videoDevices::clearDevices()
{
    std::vector<videoDevice *>::iterator i = vds_Devices.begin();
    for(; i != vds_Devices.end(); ++i)
        delete (*i);
    vds_Devices.clear();
}

videoDevices::~videoDevices(void)
{
    clearDevices();
}

videoDevice * videoDevices::getDevice(unsigned int i)
{
    if(i >= vds_Devices.size())
    {
        return NULL;
    }
    if(i < 0)
    {
        return NULL;
    }
    return vds_Devices[i];
}

int videoDevices::getDeviceIdForUUID(const std::string& uuid)
{
	for (unsigned int i=0; i < vds_Devices.size(); ++i) {
		videoDevice* d = vds_Devices[i];
		std::wstring s(d->getName());
		std::string name(s.begin(), s.end());
		if (uuid.compare(name) == 0) {
			return (int)i;
		}
	}
	return -1;
}


long videoDevices::initDevices(IMFAttributes *pAttributes)
{
    clearDevices();
    IMFActivate **ppDevices = NULL;
    HRESULT hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);
    if (SUCCEEDED(hr))
    {
        if(count > 0)
        {
            for(UINT32 i = 0; i < count; i++)
            {
                videoDevice *vd = new videoDevice;
                vd->readInfoOfDevice(ppDevices[i], i);
                vds_Devices.push_back(vd);
                SafeRelease(&ppDevices[i]);
            }
            SafeRelease(ppDevices);
        }
        else
            hr = E_INVALIDARG;
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICES: The instances of the videoDevice class cannot be created\n");
    }
    return hr;
}

unsigned int videoDevices::getCount()
{
    return (unsigned int)vds_Devices.size();
}

videoDevices& videoDevices::getInstance()
{
    static videoDevices instance;
    return instance;
}

Parametr::Parametr()
{
    CurrentValue = 0;
    Min = 0;
    Max = 0;
    Step = 0;
    Default = 0;
    Flag = 0;
}

MediaType::MediaType()
{
    pMF_MT_AM_FORMAT_TYPEName = NULL;
    pMF_MT_MAJOR_TYPEName = NULL;
    pMF_MT_SUBTYPEName = NULL;
    Clear();
}

MediaType::~MediaType()
{
    Clear();
}

void MediaType::Clear()
{
    MF_MT_FRAME_SIZE = 0;
    height = 0;
    width = 0;
    MF_MT_YUV_MATRIX = 0;
    MF_MT_VIDEO_LIGHTING = 0;
    MF_MT_DEFAULT_STRIDE = 0;
    MF_MT_VIDEO_CHROMA_SITING = 0;
    MF_MT_FIXED_SIZE_SAMPLES = 0;
    MF_MT_VIDEO_NOMINAL_RANGE = 0;
    MF_MT_FRAME_RATE_NUMERATOR = 0;
    MF_MT_FRAME_RATE_DENOMINATOR = 0;
    MF_MT_PIXEL_ASPECT_RATIO = 0;
    MF_MT_PIXEL_ASPECT_RATIO_low = 0;
    MF_MT_ALL_SAMPLES_INDEPENDENT = 0;
    MF_MT_FRAME_RATE_RANGE_MIN = 0;
    MF_MT_FRAME_RATE_RANGE_MIN_low = 0;
    MF_MT_SAMPLE_SIZE = 0;
    MF_MT_VIDEO_PRIMARIES = 0;
    MF_MT_INTERLACE_MODE = 0;
    MF_MT_FRAME_RATE_RANGE_MAX = 0;
    MF_MT_FRAME_RATE_RANGE_MAX_low = 0;
    memset(&MF_MT_MAJOR_TYPE, 0, sizeof(GUID));
    memset(&MF_MT_AM_FORMAT_TYPE, 0, sizeof(GUID));
    memset(&MF_MT_SUBTYPE, 0, sizeof(GUID));
}

videoInput::videoInput(void): accessToDevices(false)
{
    DebugPrintOut(L"\n***** VIDEOINPUT LIBRARY - 2013 (Author: Evgeny Pereguda) *****\n\n");
    updateListOfDevices();
    if(!accessToDevices)
        DebugPrintOut(L"INITIALIZATION: There is not any suitable video device\n");
}

void videoInput::updateListOfDevices()
{
    Media_Foundation *MF = &Media_Foundation::getInstance();
    accessToDevices = MF->buildListOfDevices();
    if(!accessToDevices)
        DebugPrintOut(L"UPDATING: There is not any suitable video device\n");
}

videoInput::~videoInput(void)
{
    DebugPrintOut(L"\n***** CLOSE VIDEOINPUT LIBRARY - 2013 *****\n\n");
}

IMFMediaSource *videoInput::getMediaSource(int deviceID)
{
    if(accessToDevices)
    {
        videoDevice * VD = videoDevices::getInstance().getDevice(deviceID);
        if(VD)
        {
            IMFMediaSource *out = VD->getMediaSource();
            if(!out)
                DebugPrintOut(L"VideoDevice %i: There is not any suitable IMFMediaSource interface\n", deviceID);
            return out;
        }
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return NULL;
}

bool videoInput::setupDevice(int deviceID, unsigned int id)
{
    if (deviceID < 0 )
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return false;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
        {
            bool out = VD->setupDevice(id);
            if(!out)
                DebugPrintOut(L"VIDEODEVICE %i: This device cannot be started\n", deviceID);
            return out;
        }
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return false;
}

bool videoInput::setupDevice(int deviceID, unsigned int w, unsigned int h, unsigned int idealFramerate)
{
    if (deviceID < 0 )
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return false;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
        {
            bool out = VD->setupDevice(w, h, idealFramerate);
            if(!out)
                DebugPrintOut(L"VIDEODEVICE %i: this device cannot be started\n", deviceID);
            return out;
        }
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n", deviceID);
    }
    return false;
}

MediaType videoInput::getFormat(int deviceID, unsigned int id)
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return MediaType();
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
            return VD->getFormat(id);
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return MediaType();
}

bool videoInput::isDeviceSetup(int deviceID)
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return false;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
            return VD->isDeviceSetup();
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return false;
}

bool videoInput::isDeviceMediaSource(int deviceID)
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return false;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
            return VD->isDeviceMediaSource();
    }
    else
    {
        DebugPrintOut(L"Device(s): There is not any suitable video device\n");
    }
    return false;
}

bool videoInput::isDeviceRawDataSource(int deviceID)
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return false;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
        {
            bool isRaw = VD->isDeviceRawDataSource();
            return isRaw;
        }
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return false;
}

bool videoInput::isFrameNew(int deviceID)
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return false;
    }
    if(accessToDevices)
    {
        if(!isDeviceSetup(deviceID))
        {
            if(isDeviceMediaSource(deviceID))
                return false;
        }
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
        {
            return VD->isFrameNew();
        }
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return false;
}


unsigned int videoInput::getCountFormats(int deviceID) const
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return 0;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
            return VD->getCountFormats();
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return 0;
}

void videoInput::closeAllDevices()
{
    videoDevices *VDS = &videoDevices::getInstance();
    for(unsigned int i = 0; i < VDS->getCount(); i++)
        closeDevice(i);
}

void videoInput::setParametrs(int deviceID, CamParametrs parametrs)
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice *VD = VDS->getDevice(deviceID);
        if(VD)
            VD->setParametrs(parametrs);
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
}

CamParametrs videoInput::getParametrs(int deviceID)
{
    CamParametrs out;
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return out;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice *VD = VDS->getDevice(deviceID);
        if(VD)
            out = VD->getParametrs();
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return out;
}

void videoInput::closeDevice(int deviceID)
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice *VD = VDS->getDevice(deviceID);
        if(VD)
            VD->closeDevice();
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
}

unsigned int videoInput::getWidth(int deviceID) const
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return 0;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
            return VD->getWidth();
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return 0;
}

unsigned int videoInput::getHeight(int deviceID) const
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return 0;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
            return VD->getHeight();
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return 0;
}

unsigned int videoInput::getFrameRate(int deviceID) const
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return 0;
    }
    if(accessToDevices)
    {
        videoDevice * VD = videoDevices::getInstance().getDevice(deviceID);
        if(VD)
            return VD->getFrameRate();
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return 0;
}

wchar_t *videoInput::getNameVideoDevice(int deviceID)
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return NULL;
    }
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();
        videoDevice * VD = VDS->getDevice(deviceID);
        if(VD)
            return VD->getName();
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return L"Empty";
}

unsigned int videoInput::listDevices(bool silent)
{
    int out = 0;
    if(accessToDevices)
    {
        videoDevices *VDS = &videoDevices::getInstance();

        out = VDS->getCount();
        if(!silent) DebugPrintOut(L"\nVIDEOINPUT SPY MODE!\n\n");
        if(!silent) DebugPrintOut(L"SETUP: Looking For Capture Devices\n");
        for(int i = 0; i < out; i++)
        {
            if(!silent) DebugPrintOut(L"SETUP: %i) %s \n",i, getNameVideoDevice(i));
        }
        if(!silent) DebugPrintOut(L"SETUP: %i Device(s) found\n\n", out);
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return out;
}

videoInput& videoInput::getInstance()
{
    static videoInput instance;
    return instance;
}

bool videoInput::isDevicesAcceable()
{
    return accessToDevices;
}

#ifdef _DEBUG
void videoInput::setVerbose(bool state)
{
    DPO *dpo = &DPO::getInstance();
    dpo->setVerbose(state);
}
#endif

void videoInput::setEmergencyStopEvent(int deviceID, void *userData, void(*func)(int, void *))
{
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return;
    }
    if(accessToDevices)
    {
        if(func)
        {
            videoDevices *VDS = &videoDevices::getInstance();
            videoDevice * VD = VDS->getDevice(deviceID);
            if(VD)
                VD->setEmergencyStopEvent(userData, func);
        }
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
}

void videoInput::setNewFrameAvailableEvent(int deviceID, void *userData, void(*func)(int, void *, LONGLONG))
{
	if (deviceID < 0)
	{
		DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
		return;
	}
	if (accessToDevices)
	{
		if (func)
		{
			videoDevices *VDS = &videoDevices::getInstance();
			videoDevice * VD = VDS->getDevice(deviceID);
			if (VD)
				VD->setNewFrameAvailableEvent(userData, func);
		}
	}
	else
	{
		DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
	}
}

bool videoInput::getPixels(int deviceID, unsigned char * dstBuffer, bool flipRedAndBlue, bool flipImage)
{
    bool success = false;
    if (deviceID < 0)
    {
        DebugPrintOut(L"VIDEODEVICE %i: Invalid device ID\n", deviceID);
        return success;
    }
    if(accessToDevices)
    {
        bool isRaw = isDeviceRawDataSource(deviceID);
        if(isRaw)
        {
            videoDevice *VD = videoDevices::getInstance().getDevice(deviceID);
            RawImage *RIOut = VD->getRawImageOut();
            if(RIOut)
            {
                const unsigned int bytes = 3;
                const unsigned int height = VD->getHeight();
                const unsigned int width  = VD->getWidth();
                const unsigned int size = bytes * width * height;
                if(size == RIOut->getSize())
                {
                    processPixels(RIOut->getpPixels(), dstBuffer, width, height, bytes, flipRedAndBlue, flipImage);
                    success = true;
                }
                else
                {
                    DebugPrintOut(L"ERROR: GetPixels() - bufferSizes do not match!\n");
                }
            }
            else
            {
                DebugPrintOut(L"ERROR: GetPixels() - Unable to grab frame for device %i\n", deviceID);
            }
        }
        else
        {
            DebugPrintOut(L"ERROR: GetPixels() - Not raw data source device %i\n", deviceID);
        }
    }
    else
    {
        DebugPrintOut(L"VIDEODEVICE(s): There is not any suitable video device\n");
    }
    return success;
}

void videoInput::processPixels(unsigned char * src, unsigned char * dst, unsigned int width,
                                unsigned int height, unsigned int bpp, bool bRGB, bool bFlip)
{
    unsigned int widthInBytes = width * bpp;
    unsigned int numBytes = widthInBytes * height;
    int *dstInt, *srcInt;
    if(!bRGB)
    {
        if(bFlip)
        {
            for(unsigned int y = 0; y < height; y++)
            {
                dstInt = (int *)(dst + (y * widthInBytes));
                srcInt = (int *)(src + ( (height -y -1) * widthInBytes));
                memcpy(dstInt, srcInt, widthInBytes);
            }
        }
        else
        {
            memcpy(dst, src, numBytes);
        }
    }
    else
    {
        if(bFlip)
        {
            unsigned int x = 0;
            unsigned int y = (height - 1) * widthInBytes;
            src += y;
            for(unsigned int i = 0; i < numBytes; i+=3)
            {
                if(x >= width)
                {
                    x = 0;
                    src -= widthInBytes*2;
                }
                *dst = *(src+2);
                dst++;
                *dst = *(src+1);
                dst++;
                *dst = *src;
                dst++;
                src+=3;
                x++;
            }
        }
        else
        {
            for(unsigned int i = 0; i < numBytes; i+=3)
            {
                *dst = *(src+2);
                dst++;
                *dst = *(src+1);
                dst++;
                *dst = *src;
                dst++;
                src+=3;
            }
        }
    }
}
}


/*
*
*
* Ubitrack Component starts here
*
*
*/ 


// get a logger
static log4cpp::Category& logger(log4cpp::Category::getInstance("Ubitrack.Vision.MSMFFrameGrabber"));

using namespace Ubitrack;
using namespace Ubitrack::Vision;


#ifdef _DEBUG
struct SuppressVideoInputMessages
{
	SuppressVideoInputMessages() { videoInput::setVerbose(true); }
};

static SuppressVideoInputMessages do_it;
#endif


namespace Ubitrack {
	namespace Drivers {

		/**
		* @ingroup vision_components
		*
		* @par Input Ports
		* None.
		*
		* @par Output Ports
		* \c Output push port of type Ubitrack::Measurement::ImageMeasurement.
		*
		* @par Configuration
		* The configuration tag contains a \c <dsvl_input> configuration.
		* For details, see the MSMF documentation...
		*
		*/
		class MSMFFrameGrabber
			: public Dataflow::Component
		{
		public:

			/** constructor */
			MSMFFrameGrabber(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >);

			/** destructor, waits until thread stops */
			~MSMFFrameGrabber();

			/** starts the camera */
			void start();

			/** stops the camera */
			void stop();

			// from cvCapture
			virtual bool open(int index);
			virtual bool open(const std::string& uuid);
			virtual void close();

			virtual double getProperty(int) const;
			virtual bool setProperty(int, double);

			virtual void retrieveFrame(LONGLONG ts);


		protected:

			/** handles a frame after being converted to Vision::Image */
			void handleFrame(Measurement::Timestamp utTime, Vision::Image& bufferImage);

			/** handler method for incoming pull requests */
			Measurement::Matrix3x3 getIntrinsic(Measurement::Timestamp t)
			{
				return Measurement::Matrix3x3(t, m_undistorter->getMatrix());
			}

			// shift timestamps (ms)
			int m_timeOffset;

			// only send every nth image
			int m_divisor;

			/** desired width */
			int m_desiredWidth;

			/** desired image height */
			int m_desiredHeight;

			// desired camera name
			std::string m_desiredName;

			// desired camera index (e.g. for multiple cameras with the same name as the Vuzix HMD)
			std::string m_desiredDevicePath;


			/** exposure control */
			int m_cameraExposure;
			bool m_cameraExposureAuto;

			/** brightness control */
			int m_cameraBrightness;

			/** contrast control */
			int m_cameraContrast;

			/** saturation control */
			int m_cameraSaturation;

			/** sharpness control */
			int m_cameraSharpness;

			/** gamma control */
			int m_cameraGamma;

			/** whitebalance control */
			int m_cameraWhitebalance;
			bool m_cameraWhitebalanceAuto;

			/** gain control */
			bool m_cameraBacklightComp;

			/** gain control */
			int m_cameraGain;


			/** number of frames received */
			int m_nFrames;

			/** timestamp of last frame */
			double m_lastTime;

			/** automatic upload of images to the GPU*/
			bool m_autoGPUUpload;

			/** timestamp synchronizer */
			Measurement::TimestampSync m_syncer;

			/** undistorter */
			boost::shared_ptr<Vision::Undistortion> m_undistorter;

			// the ports
			Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;
			Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorOutPort;
			Dataflow::PullSupplier< Measurement::Matrix3x3 > m_intrinsicsPort;

			Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPortRAW;
			boost::shared_ptr < Dataflow::PushConsumer< Measurement::CameraIntrinsics > > m_intrinsicInPort;

			void newIntrinsicsPush(Measurement::CameraIntrinsics intrinsics);

			// included from from cvCapture class
			int index, width, height, fourcc;
			IplImage* frame;
			videoInput VI;

		};


		MSMFFrameGrabber::MSMFFrameGrabber(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
			: Dataflow::Component(sName)
			, m_timeOffset(0)
			, m_divisor(1)
			, m_desiredWidth(320)
			, m_desiredHeight(240)
			, m_cameraExposure(0)
			, m_cameraExposureAuto(true)
			, m_cameraBrightness(0)
			, m_cameraContrast(11)
			, m_cameraSaturation(4)
			, m_cameraSharpness(3)
			, m_cameraGamma(150)
			, m_cameraWhitebalance(4500)
			, m_cameraWhitebalanceAuto(true)
			, m_cameraBacklightComp(false)
			, m_cameraGain(34)
			, m_nFrames(0)
			, m_lastTime(-1e10)
			, m_syncer(1.0)
			, m_outPort("Output", *this)
			, m_colorOutPort("ColorOutput", *this)
			, m_intrinsicsPort("Intrinsics", *this, boost::bind(&MSMFFrameGrabber::getIntrinsic, this, _1))
			, m_outPortRAW("OutputRAW", *this)
			, m_autoGPUUpload(false)
			, index(-1)
			, width(-1)
			, height(-1)
			, fourcc(-1)
			, frame(NULL)
			, VI(videoInput::getInstance())
		{
			CoInitialize(0);

			subgraph->m_DataflowAttributes.getAttributeData("timeOffset", m_timeOffset);
			subgraph->m_DataflowAttributes.getAttributeData("divisor", m_divisor);
			subgraph->m_DataflowAttributes.getAttributeData("imageWidth", m_desiredWidth);
			subgraph->m_DataflowAttributes.getAttributeData("imageHeight", m_desiredHeight);
			m_desiredDevicePath = subgraph->m_DataflowAttributes.getAttributeString("devicePath");
			m_desiredName = subgraph->m_DataflowAttributes.getAttributeString("cameraName");

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraExposure"))
				subgraph->m_DataflowAttributes.getAttributeData("cameraExposure", m_cameraExposure);

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraExposureAuto"))
				m_cameraExposureAuto = subgraph->m_DataflowAttributes.getAttributeString("cameraExposureAuto") == "true";

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraBrightness"))
				subgraph->m_DataflowAttributes.getAttributeData("cameraBrightness", m_cameraBrightness);

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraContrast"))
				subgraph->m_DataflowAttributes.getAttributeData("cameraContrast", m_cameraContrast);

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraSaturation"))
				subgraph->m_DataflowAttributes.getAttributeData("cameraSaturation", m_cameraSaturation);

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraSharpness"))
				subgraph->m_DataflowAttributes.getAttributeData("cameraSharpness", m_cameraSharpness);

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraGamma"))
				subgraph->m_DataflowAttributes.getAttributeData("cameraGamma", m_cameraGamma);

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraWhitebalance"))
				subgraph->m_DataflowAttributes.getAttributeData("cameraWhitebalance", m_cameraWhitebalance);

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraWhitebalanceAuto"))
				m_cameraWhitebalanceAuto = subgraph->m_DataflowAttributes.getAttributeString("cameraWhitebalanceAuto") == "true";

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraBacklightComp"))
				m_cameraBacklightComp = subgraph->m_DataflowAttributes.getAttributeString("cameraBacklightComp") == "true";

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraGain"))
				subgraph->m_DataflowAttributes.getAttributeData("cameraGain", m_cameraGain);

			if (subgraph->m_DataflowAttributes.hasAttribute("cameraModelFile")){
				std::string cameraModelFile = subgraph->m_DataflowAttributes.getAttributeString("cameraModelFile");
				m_undistorter.reset(new Vision::Undistortion(cameraModelFile));
			}
			else {
				std::string intrinsicFile = subgraph->m_DataflowAttributes.getAttributeString("intrinsicMatrixFile");
				std::string distortionFile = subgraph->m_DataflowAttributes.getAttributeString("distortionFile");


				m_undistorter.reset(new Vision::Undistortion(intrinsicFile, distortionFile));
			}

			Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
			if (oclManager.isEnabled()) {
				if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
					m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
					LOG4CPP_INFO(logger, "Upload to GPU enabled? " << m_autoGPUUpload);
				}
			}

			// dynamically generate input ports
			for (Graph::UTQLSubgraph::EdgeMap::iterator it = subgraph->m_Edges.begin(); it != subgraph->m_Edges.end(); it++)
			{
				if (it->second->isInput())
				{
					if (0 == it->first.compare(0, 15, "InputIntrinsics")) {
						m_intrinsicInPort.reset(new Dataflow::PushConsumer< Measurement::CameraIntrinsics >(it->first, *this,
							boost::bind(&MSMFFrameGrabber::newIntrinsicsPush, this, _1)));

					}

				}
			}


			// init MSMF
			int count_devices = VI.listDevices(true);
			for (unsigned int i = 0; i < count_devices; ++i) {
				std::wstring wname(VI.getNameVideoDevice(i));
				LOG4CPP_INFO(logger, "Found videoDevice: " << std::string(wname.begin(), wname.end()));
			}


		}

		void MSMFFrameGrabber::newIntrinsicsPush(Measurement::CameraIntrinsics intrinsics) {
			m_undistorter.reset(new Vision::Undistortion(*intrinsics));
		}


		MSMFFrameGrabber::~MSMFFrameGrabber()
		{
			close();
			CoUninitialize();
		}

		void MSMFFrameGrabber::close()
		{
			if (index >= 0)
			{
				// unregister frame callback
				VI.setNewFrameAvailableEvent(index, NULL, NULL);

				VI.closeDevice(index);
				index = -1;
				cvReleaseImage(&frame);
			}
			width = height = -1;
		}

		// Initialize camera input
		bool MSMFFrameGrabber::open(const std::string& uuid)
		{
			int _index = videoDevices::getInstance().getDeviceIdForUUID(uuid);
			if (_index < 0)
				return false;
			return open(_index);
		}

		void NewFrameEvent(int deviceID, void *userData, LONGLONG ts)
		{
			MSMFFrameGrabber* fg = static_cast<MSMFFrameGrabber*>(userData);
			if (fg) {
				fg->retrieveFrame(ts);
			}
		}

		// Initialize camera input
		bool MSMFFrameGrabber::open(int _index)
		{

				int try_index = _index;
				int devices = 0;
				close();
				devices = VI.listDevices(true);
				if (devices == 0)
					return false;
				try_index = try_index < 0 ? 0 : (try_index > devices - 1 ? devices - 1 : try_index);

				// With maximum frame size for now.
				if (!VI.setupDevice(try_index, 0, 0, 0)) {
					LOG4CPP_ERROR(logger, "Could not setup videoDevice.");
					return false;
				}

				if (!VI.isFrameNew(try_index))
					return false;
				index = try_index;
				
				// register frame callback
				VI.setNewFrameAvailableEvent(index, this, NewFrameEvent);

				return true;
		}

		void MSMFFrameGrabber::retrieveFrame(LONGLONG ts)
		{
			if (!VI.isDeviceSetup(index))
				return;

			if (!VI.isFrameNew(index))
				return;

			const int w = (int)VI.getWidth(index);
			const int h = (int)VI.getHeight(index);
			if (!frame || w != frame->width || h != frame->height)
			{
				if (frame)
					cvReleaseImage(&frame);
				frame = cvCreateImage(cvSize(w, h), 8, 3);
			}
			// this copies the pixels .. in theory this is only needed for uncalibrated framegrabbers ...
			VI.getPixels(index, (uchar*)frame->imageData, false, true);

			Vision::Image bufferImage(w, h, 3, (uchar*)frame->imageData, IPL_DEPTH_8U, 1);
			bufferImage.set_pixelFormat(Vision::Image::BGR);
			// ts is LONGLONG in 100-nanosecond units
			Measurement::Timestamp utTime = m_syncer.convertNativeToLocal(ts/10000000.);
			handleFrame(utTime + 1000000L * m_timeOffset, bufferImage);

		}

		double MSMFFrameGrabber::getProperty(int property_id) const
		{
			// image format proprrties
			switch (property_id)
			{
			case CV_CAP_PROP_FRAME_WIDTH:
				return VI.getWidth(index);
			case CV_CAP_PROP_FRAME_HEIGHT:
				return VI.getHeight(index);
			case CV_CAP_PROP_FPS:
				return VI.getFrameRate(index);
			default:
				break;
			}
			return 0;
		}
		bool MSMFFrameGrabber::setProperty(int property_id, double value)
		{
			// image capture properties
			unsigned int fps = 0;
			bool handled = false;
			switch (property_id)
			{
			case CV_CAP_PROP_FRAME_WIDTH:
				width = cvRound(value);
				fps = VI.getFrameRate(index);
				handled = true;
				break;
			case CV_CAP_PROP_FRAME_HEIGHT:
				height = cvRound(value);
				fps = VI.getFrameRate(index);
				handled = true;
				break;
			case CV_CAP_PROP_FPS:
				width = (int)VI.getHeight(index);
				height = (int)VI.getWidth(index);
				fps = cvRound(value);
				break;
			}

			if (handled) {
				if (width > 0 && height > 0)
				{
					if ((width != (int)VI.getWidth(index) || height != (int)VI.getHeight(index) || fps != VI.getFrameRate(index))
						&& VI.isDeviceSetup(index))//|| fourcc != VI.getFourcc(index) )
					{
						VI.closeDevice(index);
						VI.setupDevice(index, width, height, fps);
					}
					width = height = -1;
					return VI.isDeviceSetup(index);
				}
				return true;
			}

			return false;
		}

		void MSMFFrameGrabber::start()
		{
			if (!m_running) {
				// start capturing
			}
			Component::start();
		}


		void MSMFFrameGrabber::stop()
		{
			if (m_running) {
				// stop/pause capturing
			}
			Component::stop();
		}


		void MSMFFrameGrabber::handleFrame(Measurement::Timestamp utTime, Vision::Image& bufferImage)
		{

#ifdef ENABLE_EVENT_TRACING
			TRACEPOINT_MEASUREMENT_CREATE(getEventDomain(), utTime, getName().c_str(), "VideoCapture")
#endif

				if (m_autoGPUUpload){
					//force upload to the GPU
					bufferImage.uMat();

				}

			boost::shared_ptr< Vision::Image > pColorImage;
			bool bColorImageDistorted = true;

			if ((m_desiredWidth > 0 && m_desiredHeight > 0) &&
				(bufferImage.width() > m_desiredWidth || bufferImage.height() > m_desiredHeight))
			{
				pColorImage.reset(new Vision::Image(m_desiredWidth, m_desiredHeight, 3));
				pColorImage->copyImageFormatFrom(bufferImage);
				cv::resize(bufferImage.Mat(), pColorImage->Mat(), cv::Size(m_desiredWidth, m_desiredHeight));
			}

			if (m_outPortRAW.isConnected()) {
				m_outPortRAW.send(Measurement::ImageMeasurement(utTime, bufferImage.Clone()));
			}

			if (m_colorOutPort.isConnected())
			{
				if (pColorImage)
					pColorImage = m_undistorter->undistort(pColorImage);
				else
					pColorImage = m_undistorter->undistort(bufferImage);
				bColorImageDistorted = false;

				m_colorOutPort.send(Measurement::ImageMeasurement(utTime, pColorImage));
			}


			if (m_outPort.isConnected())
			{
				boost::shared_ptr< Vision::Image > pGreyImage(new Vision::Image(bufferImage.width(), bufferImage.height(), 1, IPL_DEPTH_8U));

				if (pColorImage){
					if (pColorImage->getImageState() == Image::ImageUploadState::OnCPUGPU || pColorImage->getImageState() == Image::ImageUploadState::OnGPU)
						cv::cvtColor(pColorImage->uMat(), pGreyImage->uMat(), cv::COLOR_RGB2GRAY);
					else
						pGreyImage = pColorImage->CvtColor(CV_BGR2GRAY, 1);
				}
				else{
					if (bufferImage.getImageState() == Image::ImageUploadState::OnCPUGPU || bufferImage.getImageState() == Image::ImageUploadState::OnGPU)
						cv::cvtColor(bufferImage.uMat(), pGreyImage->uMat(), cv::COLOR_RGB2GRAY);
					else
						pGreyImage = bufferImage.CvtColor(CV_BGR2GRAY, 1);
				}

				if (bColorImageDistorted)
					pGreyImage = m_undistorter->undistort(pGreyImage);

				m_outPort.send(Measurement::ImageMeasurement(utTime, pGreyImage));
			}
		}



	}
} // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT(Dataflow::ComponentFactory* const cf) {
	cf->registerComponent< Ubitrack::Drivers::MSMFFrameGrabber >("MSMFFrameGrabber");
}

#endif // HAVE_MSMF