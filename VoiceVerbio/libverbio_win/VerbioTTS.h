#ifndef VERBIO_TTS_H__
#define VERBIO_TTS_H__

#include "Verbio.h"

#ifndef VERBIO_API
#ifdef _WINDOWS
	#ifdef VERBIO_EXPORTS
	#define VERBIO_API __declspec(dllexport)
	#else
	#define VERBIO_API __declspec(dllimport)
	#endif
#else
	#define VERBIO_API 
#endif
#endif

class VerbioTTSResource;

class VERBIO_API VerbioTTSLicense {
public:
	VerbioTTSLicense() {};
	virtual ~VerbioTTSLicense() {};
	
	virtual int GetAvailableResources(const char *language) const = 0;
	
	virtual bool IsEvaluation() const = 0;
	virtual bool IsLite() const = 0;
};

class VERBIO_API VerbioSpeakerInfo {
public:
	VerbioSpeakerInfo() {};
	virtual ~VerbioSpeakerInfo() {};

	virtual const char *GetIdent() const = 0;
	virtual const char *GetName() const = 0;
	virtual const char *GetGender() const = 0;
	virtual const char *GetAge() const = 0;
	virtual const char *GetLanguage() const = 0;
};

class VERBIO_API VerbioTTS
{
public:
	VerbioTTS() {};
	virtual ~VerbioTTS() {};
	
	virtual int  SetDefaultServerIP(const char *serverIP) = 0;
	virtual int  AddBackUpServerIP(const char *serverIP) = 0;
	virtual int  SetNetTimeout(int sec) = 0;
	virtual int  SetCallBackServerDisconnect(void (*cb_serverdisconnect)(const char *)) = 0;
	
	virtual short GetNumberOfAvailableLngs(const char *server = 0) = 0;
	virtual const char *GetLanguage(unsigned short lngId) = 0;
	virtual int  SetDefaultLanguage(const char *language) = 0;
	virtual int  AddLanguage(const char *language) = 0;
	
	virtual int Open() = 0;
	virtual int Close() = 0;
	virtual int Disconnect() = 0;
	virtual int Reconnect (const char *disconnectedIP) = 0;
	
	virtual const char *GetError() = 0;

	virtual int SetParameter(unsigned long parm, void *value) = 0;
	virtual int GetParameter(unsigned long parm, void *value) = 0;
	
	virtual const char *GetDefaultLanguage() = 0;
	virtual const VerbioTTSLicense *GetLicenseInfo() = 0;
	virtual int GetVersion(unsigned long *prodver) = 0;
	virtual const char *GetServerIP(int number = -1) = 0;
	
	virtual unsigned short GetNumberOfAvailableSpks() = 0;
	virtual VerbioSpeakerInfo* GetSpeakerInfo(unsigned short spkId) = 0;
	virtual const float GetSamplingFrequency() = 0;

	virtual VerbioTTSResource* GetVerbioTTSResource(int dev = -1, const char *server = 0) = 0;
};

class VERBIO_API VerbioTTSResource
{
public:
	VerbioTTSResource() {};
	virtual ~VerbioTTSResource() {};

#ifdef _WIN32
	virtual int AssignAudioChannel(int wdev) = 0;
	virtual int AssignDialogicChannel(int chdev) = 0;
	virtual int AssignNMSChannel(int chdev) = 0;
#endif

	virtual const VerbioTTS *GetVerbioTTSParent() = 0;
	virtual const int GetVerbioTTSHandle() = 0;

	virtual const char *GetError() = 0;
	
	virtual int SetServerIP(const char *serverIP) = 0;
	
	virtual int PlayStrOpen(const char *string, int mode = VERBIO_AUDIO_MULAW) = 0;
	virtual int PlayStrRead(char *buffer, int count) = 0;
	virtual int PlayStrClose() = 0;
	virtual int VerbioSeek(long offset, int origin) = 0;

#ifdef _WIN32	
	virtual int PlayStr(const char *string, int mode = VERBIO_AUDIO_MULAW, const char *filename = 0, void *extraparm = 0, int *stopplay = 0, int fetchtimeout = 5) = 0;
#endif

	virtual int SetLanguage(const char *language) = 0;
	virtual const char* GetLanguage() = 0;
	virtual int SetSpeakerName(const char *name) = 0;
	virtual int SetSpeakerGender(const char *gender) = 0;
	virtual int SetSpeakerAge(const char *age) = 0;
	virtual VerbioSpeakerInfo* GetSpeakerInfo() = 0;
	virtual int SetRate(int wordsperminute) = 0;
	virtual int GetRate() = 0;
	virtual int SetPitch(int F0) = 0;
	virtual int GetPitch() = 0;
	virtual int SetVolume(int volume) = 0;
	virtual int GetVolume() = 0;

	virtual int SetParameter(unsigned long parm, void *value) = 0;
	virtual int GetParameter(unsigned long parm, void *value) = 0;
	
	virtual int SetDictionary(const char *language, const char *filename) = 0;   
	virtual int SetAbbreviations(const char *language, const char *filename) = 0;
	virtual int TransferFile(const char *lfilename, char *rfilename) = 0;

#ifdef SAPI
#ifdef _WIN32
	virtual HSAPI SAPIOpen(short language, short sublanguage = SUBLANG_NEUTRAL, const char *name = 0, WORD gender = GENDER_NEUTRAL, WORD age = TTSAGE_ADULT, DWORD features = TTSFEATURE_ANYWORD, DWORD interfaces = TTSI_ILEXPRONOUNCE) = 0;
	virtual int	SAPISelect(HSAPI hSapi) = 0;
	virtual int SAPIClose(HSAPI hSapi) = 0;
#endif
#endif

	virtual int WaitForLicense(int timeout) = 0;
	virtual int ReleaseLicense() = 0;
};

extern "C" VERBIO_API VerbioTTS* getVerbioTTS();

#endif
