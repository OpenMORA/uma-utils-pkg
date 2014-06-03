#ifndef VERBIO_ASR_H__
#define VERBIO_ASR_H__

#include "Verbio.h"
#include <stdlib.h>

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

//ASR defines
#define VERBIO_GRAMMAR_CONNECTED	0x00000000
#define VERBIO_GRAMMAR_ISOLATED		0x00010000
#define VERBIO_GRAMMAR_UNIGRAM		0x00020000
#define VERBIO_GRAMMAR_ABNF			0x00080000
#define VERBIO_GRAMMAR_XML			0x00100000
#define VERBIO_GRAMMAR_OVERWRITE	0x00200000

//NLSML defines
#define VERBIO_NLSML_SEMANTIC		0x00000001
#define VERBIO_NLSML_EMPTYRULES		0x00000002
#define VERBIO_NLSML_VOICEGENIE		0x00000004
#define VERBIO_NLSML_GENESYS		0x00000008
#define VERBIO_NLSML_AVAYA			0x00000010

class VerbioASRResource;

class VERBIO_API VerbioASRLicense {
public:
	VerbioASRLicense() {};
	virtual ~VerbioASRLicense() {};
	
	virtual int GetAvailableResources(const char *configuration) const = 0;
	
	virtual bool IsEvaluation() const = 0;
	virtual bool IsLite() const = 0;
};

class VERBIO_API VerbioResult {
public:
	VerbioResult() {};
	virtual ~VerbioResult() {};

	virtual const char *GetNLSMLString(unsigned long mode = 0) = 0;
	
	virtual int GetGrammarHandle(int hyp = 0) const = 0;
	
	virtual int GetNumberOfSlots(int hyp = 0) const = 0;
	virtual const char *GetName(int slotId, int hyp = 0) const = 0;
	virtual const char *GetValue(const char *slot = 0, int hyp = 0) const = 0;
	virtual float GetConfidence(const char *slot = 0, int hyp = 0) const = 0;
	virtual int GetInterval(float *tbegin, float *tend, const char *slot = 0, int hyp = 0) const = 0;
	virtual const char *GetUtterance(const char *slot = 0, int hyp = 0) const = 0;

	virtual int GetNumberOfUnits(int hyp = 0) const = 0;
	virtual int GetIndex(int unit, int hyp = 0) const = 0;
	virtual float GetEnergy(int unit, int hyp = 0) const = 0;
	virtual int GetFrames(int unit, int hyp = 0) const = 0;
	virtual float GetRScore(int unit, int hyp = 0) const = 0;
	virtual long GetIni(int unit, int hyp = 0) const = 0;
	virtual int GetSelected(int unit, int hyp = 0) const = 0;
	
	virtual const char *GetValue(int unit, int hyp = 0) = 0;
	virtual float GetConfidence(int unit, int hyp = 0) = 0;
	virtual const char *GetUtterance(int unit, int hyp = 0) = 0;
	virtual const char *GetRule(int unit, int hyp = 0) = 0;
};

class VERBIO_API VerbioVADResult
{
public:
	VerbioVADResult() {};
	virtual ~VerbioVADResult() {};

	virtual int GetState() const = 0;
	virtual unsigned long GetInitialSilence() const = 0;
	virtual unsigned long GetFinalSilence() const = 0;
	virtual unsigned long GetVoiceDetected() const = 0;
};

class VERBIO_API VerbioASR
{
public:
	VerbioASR() {};
	virtual ~VerbioASR() {};
	
	virtual int  SetDefaultServerIP(const char *serverIP) = 0;
	virtual int  AddBackUpServerIP(const char *serverIP) = 0;
	virtual int  SetNetTimeout(int sec) = 0;
	virtual int  SetCallBackServerDisconnect(void (*cb_serverdisconnect)(const char *)) = 0;
	
	virtual short GetNumberOfAvailableLngs(const char *server = 0) = 0;
	virtual const char *GetLanguage(unsigned short lngId) = 0;
	virtual int  SetDefaultLanguage(const char *language) = 0;
	virtual int  AddLanguage(const char *language) = 0;

	virtual short GetNumberOfAvailableConfs(const char *server = 0) = 0;
	virtual const char *GetConfiguration(unsigned short confId) = 0;
	virtual int  SetDefaultConfiguration(const char *configuration) = 0;
	virtual int  AddConfiguration(const char *configuration) = 0;

	virtual short GetNumberOfAvailableSLMConfs(const char *server = 0) = 0;
	virtual const char *GetSLMConfiguration(unsigned short confId) = 0;
	virtual short GetNumberOfAvailableSLMModels(const char *config) = 0;
	virtual const char *GetSLMModel(unsigned short modelId) = 0;
	
	virtual int Open() = 0;
	virtual int Close() = 0;
	virtual int Disconnect() = 0;
	virtual int Reconnect (const char *disconnectedIP) = 0;

	virtual const char *GetError() = 0;

	virtual int SetParameter(unsigned long parm, void *value) = 0;
	virtual int GetParameter(unsigned long parm, void *value) = 0;
	
	virtual const char *GetDefaultLanguage() = 0;
	virtual const VerbioASRLicense *GetLicenseInfo() = 0;
	virtual int GetVersion(unsigned long *prodver) = 0;
	virtual const char *GetServerIP(int number = -1) = 0;

	virtual const float *GetDefaultSamplingFrequency() = 0; 
	virtual int SetSamplingFrequency(float samplingFreq) = 0; 
	
	virtual int PrepareGrammar(const char *fname, int type, int *errorline = 0, int fetchtimeout = 5, const char *language = 0) = 0;
	
	virtual VerbioASRResource * GetVerbioASRResource(int dev = -1, const char *serverIP = 0) = 0;
};

class VERBIO_API VerbioASRResource
{
public:
	VerbioASRResource() {};
	virtual ~VerbioASRResource() {};

	#ifdef _WIN32
	virtual int AssignAudioChannel(int wdev) = 0;
	virtual int AssignDialogicChannel(int chdev, CSPParm *parm = 0) = 0;
	virtual int AssignNMSChannel(int chdev) = 0;
	#endif

	virtual const VerbioASR *GetVerbioASRParent() = 0;
	virtual const int GetVerbioASRHandle() = 0;
	
	virtual const char *GetError() = 0;
	
	virtual int SetServerIP(const char *serverIP) = 0;

	virtual void VADClear(VAD_PRM *prm) = 0;
	virtual int  VADOpen(VAD_PRM *prm = 0, int mode = VERBIO_AUDIO_MULAW) = 0;
	virtual int  VADWrite(const char *buffer, int count, VerbioVADResult **result = 0) = 0;
	virtual int  VADClose() = 0;

	virtual int  DTMFOpen(int mode = VERBIO_AUDIO_MULAW) = 0;
	virtual const char*  DTMFWrite(const char *buffer, int count) = 0;
	virtual int  DTMFClose() = 0;

	virtual int RecStrOpen(int initsil, int maxsil, int mode = VERBIO_AUDIO_MULAW, int (*StopRecStr)(int) = 0) = 0;
	virtual int RecStrWrite(const char *buffer, int count) = 0;
	virtual int RecStrClose() = 0;
	virtual int VerbioSeek(long offset, int origin) = 0;

	#ifdef _WIN32
	virtual int RecStr(int maxtime, int initsil, int maxsil, int mode = VERBIO_AUDIO_MULAW | VERBIO_AUDIO_TONE, const char *fname = 0, void *extraparm = 0) = 0;
	virtual int RecFile(int maxtime, int initsil, int maxsil, int mode = VERBIO_AUDIO_MULAW | VERBIO_AUDIO_TONE, const char *fname = 0, void *extraparm = 0) = 0;
	#endif

	virtual int SetConfiguration(const char *configuration) = 0;
	virtual const char* GetConfiguration() = 0;
	virtual int SetLanguage(const char *language) = 0;
	virtual const char* GetLanguage() = 0;
	virtual int SetNBest(int nbest) = 0;
	virtual int SetActiveNBest(int nbest) = 0;
	virtual int GetNBest() = 0;
	virtual int GetActiveNBest() = 0;
	
	virtual int SetParameter(unsigned long parm, void *value) = 0;
	virtual int GetParameter(unsigned long parm, void *value) = 0;
	
	virtual int PrepareGrammar(const char *fname, int type, int *errorline = 0, int fetchtimeout = 5, const char *language = 0) = 0;
	virtual int LoadGrammar(const char *grammar, int mode) = 0;
	virtual int	FreeGrammar(int handle) = 0;
	virtual int	ActivateGrammar(int handle) = 0;
	virtual int	DeactivateGrammar(int handle) = 0;
	virtual int TransferFile(const char *lfilename, char *rfilename) = 0;
	
	virtual VerbioResult* GetResult(int maxind = 1) = 0;
	virtual int GetResult(char *result, float *confidence, int maxind = 1, int hyp = 0) = 0;
	virtual unsigned long GetWaveformSize() = 0;
	virtual const char* GetWaveform() = 0;
	virtual int CheckDTMFCompliance(const char digit) = 0;
	
	virtual int WaitForLicense(int timeout, const char *cfg = 0) = 0;
	virtual int ReleaseLicense(const char *cfg = 0) = 0;

	virtual int RegisterVVICallBack (int (*)(const char *result, unsigned long score, size_t ntfyId)) = 0;

	virtual int SpeakerAddFile (const char *id, const char *filename, const char *transcription) = 0;
	virtual int SpeakerTrain (const char *id, const char *filename, int *lpiword) = 0;
	virtual int	SpeakerVerify (const char *id, const char *filename, const char *transcription, float *score) = 0;
	virtual int SpeakerExist (const char *id) = 0;
};

extern "C" VERBIO_API VerbioASR* getVerbioASR();

#endif
