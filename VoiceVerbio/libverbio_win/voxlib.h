#ifndef _VOXLIB_H_
#define _VOXLIB_H_

#ifdef _WIN32
#include "windows.h"
#define VERBIOAPI WINAPI
#else
#define VERBIOAPI
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef int (*VX_RSP_CALLBACK) (int);

#ifndef VXRSP
#define VXRSP
typedef struct {
	int maxsil;
	int initsil;
	int io_fhandle;
	int skip_samples;
	VX_RSP_CALLBACK callback;
	int reserved[3];
} VX_RSP;
#endif

#ifndef VADPRM
#define VADPRM
typedef struct {
	unsigned short frame_len;
	unsigned short frame_step;
	float sil_req_ms;
	float voice_req_ms;
	float low_factor;
	float high_factor;
	float final_factor;
	float final_high_factor;
	float max_ref_factor;
	float min_high_thresh;
	float aam_min;
	float aam_max;

	int (*callback)(int state, unsigned long initsil, unsigned long finalsil, unsigned long voice);
}
VAD_PRM;
#endif

/* Voice Activity Detection States */
#define VVX_INIT			0
#define VVX_SILENCE			1
#define VVX_LOW     		2
#define VVX_HIGH			3
#define VVX_FINAL			4
#define VVX_VOICE   		5

#define IND_NOWORD              -1000
#define IND_EARLY_START         -1001
#define IND_NOVOICE             -1002
#define IND_STREAM              -3000

/* Vox Options (obsolete) */
#define OVX_NOPLAYSTR      0x00020000
#define OVX_NORECSTR       0x00040000
#define OVX_TTSLITE        0x00080000
#define OVX_TTS16K         0x00400000

/* Vox Languages (obsolete) */
#define LVX_SPANISH        0x00000000
#define LVX_CATALAN        0x01000000
#define LVX_PORTUGUESE     0x02000000
#define LVX_BASQUE         0x04000000
#define LVX_BRAZILIAN      0x08000000

/* Vox Languages */
#define LNG_BASQUE                 "eu"
#define LNG_CATALAN                "ca"
#define LNG_ENGLISH                "en-us"
#define LNG_ARABIAN                "ar-ma"
#define LNG_FRENCH                 "fr"
#define LNG_GALICIAN               "ga"
#define LNG_PORTUGUESE             "pt"
#define LNG_PORTUGUESE_BRAZILIAN   "pt-br"
#define LNG_SPANISH                "es"
#define LNG_SPANISH_MEXICAN        "es-mx"
#define LNG_SPANISH_ARGENTINIAN    "es-ar"
#define LNG_SPANISH_COLOMBIAN      "es-co"
#define LNG_SPANISH_CHILEAN        "es-cl"
#define LNG_SPANISH_VENEZUELAN     "es-ve"
#define LNG_CATALAN_VALENCIAN      "ca-va"

/* Vox Configurations */
#define CFG_ENGLISH                  "en-us"
#define CFG_ARABIAN                  "ar-ma"
#define CFG_FRENCH                   "fr"
#define CFG_SPANISH                  "es"
#define CFG_SPANISH16K               "es16k"
#define CFG_CATALAN16K               "ca16k"
#define CFG_SPANISH_CATALAN          "es_ca"
#define CFG_SPANISH_GALICIAN         "es_ga"
#define CFG_SPANISH_BASQUE           "es_eu"
#define CFG_SPANISH_MEXICAN          "es-mx"
#define CFG_SPANISH_ARGENTINIAN      "es-ar"
#define CFG_SPANISH_COLOMBIAN        "es-co"
#define CFG_SPANISH_CHILEAN          "es-cl"
#define CFG_SPANISH_VENEZUELAN       "es-ve"
#define CFG_PORTUGUESE               "pt"
#define CFG_PORTUGUESE_BRAZILIAN     "pt-br"
#define CFG_SPANISH_CATALAN_BASQUE_GALICIAN  "es_ca_eu_ga"

#define CFG_MAXWORDS                 "words"

/* Vox Grammar Types */
#define GVX_CONNECTED      0x00000000     /* connected words       */
#define GVX_ISOLATED       0x00010000     /* isolated words        */
#define GVX_UNIGRAM        0x00020000     /* unigram               */
#define GVX_ABNF           0x00080000     /* ABNF                  */
#define GVX_VVI            0x04000000     /* VoxPopuli             */
#define GVX_DTMF           0x08000000     /* DTMF                  */

#define GVX_DIGITS_ES      0x00040000     /* connected digits (es) */
#define GVX_DIGITS_CA      0x00100000     /* connected digits (ca) */
#define GVX_DIGITS_PT      0x00400000     /* connected digits (pt) */
#define GVX_DIGITS_BR      0x02000000     /* connected digits (br) */
#define GVX_DIGITS_EU      0x00800000     /* connected digits (eu) */
#define GVX_NODEFSIL       0x01000000     /* disables default ABNF silences */
#define GVX_USELNG         0x10000000     /* uses language in mode in prevcb() */

#define GVX_LOAD           0x10000000
#define GVX_ACTIVATE       0x20000000
#define GVX_DEACTIVATE     0x40000000
#define GVX_UNLOAD         0x80000000

/* IberVox vx_playstr modes */
#define MC_TTSFILE         0x00080000     /* TTS File input        */

/* IberVox vx_recind / vx_nbest modes */
#ifndef _USR_UNIT
#define _USR_UNIT
typedef struct {
	int   index;
	float energy;
	int   frames;
	float rscore;
	long  ini;
	int   selected;
}
USR_UNIT;
#endif
#define MC_UNITINFO        ((int) 0x80000000)     /* UNIT info             */

/* File types */
#define MC_MULAW           0x00000000     /* mu-law Dialogic (raw) */
#define MC_ALAW            0x00000020     /* A-law Dialogic (raw)  */
#define MC_LIN16           0x00200000     /* Linear (16 bit) raw   */
#define MC_TTSFILE         0x00080000     /* file input */
#define MC_INITSIL         0x00002000     /* use init silence */

/* License modes */
#define LIC_EVALUATION     1
#define LIC_LITE           2
#define LIC_TTS_CATALAN    4
#define LIC_ASR_CATALAN    8
#define LIC_TTS_KATIA     16

/* Vox getparm and setparm */
#define VXGB_RECSTRLIC     0x60800201
#define VXGB_PLAYSTRLIC    0x60800202
#define VXGB_RECSAMPFREQ   0xE0800203
#define VXGB_VSDMAXREF     0xE0800204
#define VXGB_LICMODE       0x60800205
#define VXGB_NETTIMEOUT    0x60800206
#define VXGB_CMDTIMEOUT    0x60800207
#define VXGB_VSDMINREF     0xE0800207
#define VXGB_TUNNING_ACT   0x60800208
#define VXGB_TUNNING_DIR   0xA0800209
#define VXGB_VVI_ARQ       0xA080020A
#define VXGB_MAXFRAMES     0x60800209

/* VoxServer Client parameters */
#define VXGB_DEFSERVER     0x80000801
#define VXGB_LOCALADDR     0x80000802
#define VXGB_SRVCLOSE      0x80000803
#define VXGB_TTSSPKINFO    0x80000804
#define VXGB_START_SPK     0x80000805
#define VXGB_START_CONF    0x80000806
#define	VXGB_START_VIT     0x80000807
#define	VXGB_START_SLM     0x80000808

#define VXCH_NBEST         0x60000101
#define VXCH_ACTIVE_NBEST  0x60000112
#define VXCH_TTSFREQUENCY  0x60000113
#define VXCH_HIGHTHRESHOLD 0xE0000102
#define VXCH_REFTHRESHOLD  0xE0000103
#define VXCH_TTSSPEED      0x60000104
#define VXCH_TTSLNG        0x60000106
#define VXCH_TTSPITCH      0x60000107
#define VXCH_KLENGTH       0x60000109
#define VXCH_GRMWEIGHT     0xE000010A
#define VXCH_GRMCONSTANT   0xE000010B
#define VXCH_MINTHRESHOLD  0xE000010C
#define VXCH_DEFASRLNG     0x6000010C
#define VXCH_TTSVOLUME     0x6000010D
#define VXCH_MSSPEECH      0x6000010E
#define VXCH_VOICEDETECTED 0x6000010F
#define VXCH_RECSAMPLES    0x60000110
#define VXCH_RECMODE       0x60000111
#define VXCH_INITSIL       0x60000114
#define VXCH_FINALSIL      0x60000115
#define VXCH_VOICE         0x60000116
#define VXCH_SERVER        0x80000401
#define VXCH_TTSSPKNAME    0x80000402
#define VXCH_DEFASRLANG    0x80000403
#define VXCH_DEFTTSLANG    0x80000404
#define VXCH_DEFASRCFG     0x80000405
#define VXCH_DEACTWORD     0x80000406
#define VXCH_ACTWORD       0x80000407

#define VXCH_HTHRESHOLD    VXCH_HIGHTHRESHOLD

/* Verbio error codes */
#define EVX_NOERROR        0	/* NO ERROR */
#define EVX_SRERROR      - 1	/* DIALOGIC SRL ERROR. (Check ATDV_LASTERR() and ATDV_ERRMSGP()) */
#define EVX_DXERROR      - 2	/* DIALOGIC VOICE ERROR. (Check ATDV_LASTERR() and ATDV_ERRMSGP()) */
#define EVX_NOBOARDS     - 3	/* NO BOARDS DETECTED (Check Dialogic Drivers Start Up) */
#define EVX_INVSETUP     - 4	/* Vox ERROR (Files may be corrupted. Check disk and repeat Vox Setup) */
#define EVX_NOMEM        - 5	/* OUT OF MEMORY. (Check memory leakages) */
#define EVX_VCBFILE      - 6	/* THE VOCABULARY FILE NAME IS NOT VALID. (Check the vocabulary file name and path writing permission) */
#define EVX_INVWORD      - 7	/* THE VOCABULARY CONTAINS AN INVALID WORD. (Check and correct invalid words) */
#define EVX_NOLICFILE    - 8	/* NO LICENSE FILE WAS FOUND. (Use Setup and CheckOut to obtain the Vox directory structure and the license file) */
#define EVX_INVLIC       - 9	/* THE LICENSE FILE IS NOT VALID. (Use CheckOut to obtain a valid license file) */
#define EVX_SYSTEM       -10	/* SYSTEM ERROR (Check errno) */
#define EVX_INVSRVER     -11	/* INVALID DIALOGIC SRL RELEASE (libsrlmt.dll) */
#define EVX_INVDXVER     -12	/* INVALID DIALOGIC VOICE LIBRARY RELEASE (libdxxmt.dll) */
#define EVX_NOLIBINIT    -13	/* VOXLIB WAS NOT SUCCESSFULLY LOADED. (Call vox_libinit() before using any Vox function) */
#define EVX_NOLIC        -14	/* NO LICENSE */
#define EVX_NOSETVCB     -15	/* NO ACTIVE VOCABULARY. (Use vox_setvcb() to set the active vocabulary) */
#define EVX_NORECSTR     -16	/* NO RECOGNITION. (Use vox_recstr() to init recognition) */
#define EVX_NOLINE       -17	/* NO MORE LINES ARE AVAILABLE FOR THE SPECIFIED CHANNEL DEVICE */
#define EVX_BADPARM      -18	/* INVALID PARAMETER IN FUNCTION CALL */
#define EVX_NOTIMP       -19	/* NOT IMPLEMENTED */
#define EVX_NORECIND     -20	/* NO RECIND OR NBEST. (Call vox_recind() before calling ATVOX_NIND()) */
#define EVX_INVFILE      -21	/* INVALID FILENAME */
#define EVX_NETWORK      -22	/* NETWORK ERROR */
#define EVX_DICFILE      -23	/* THE DICTIONARY FILE NAME IS NOT VALID */
#define EVX_PARSER       -24	/* ABNF PARSER ERROR */
#define EVX_INVVER       -25	/* THE VOXSERVER VERSION DOES NOT MATCH THE CLIENT VERSION */
#define EVX_NBEST        -26    /* MCCH_ACTIVE_NBEST GREATER THAN MCCH_NBEST */
#define EVX_CONTENT	     -27    /* INVALID CONTENT FILE */
#define EVX_NOTENOUGH    -28    /* NOT ENOUGH DIGITS FOR TRAINING */

/* Vox functions */
int    VERBIOAPI vox_asr_init(const char* configuration, const char* defasrlng);
int    VERBIOAPI vox_tts_init(const char* configuration, const char* defttslng);
int    VERBIOAPI vox_getasrlic(const char* configuration);
int    VERBIOAPI vox_getttslic(const char* language); 
int    VERBIOAPI vox_devclose(int dev);
int    VERBIOAPI vox_thclose(void);
int    VERBIOAPI vox_libclose(void);
int    VERBIOAPI vox_chkwrd(const char *word, int language);
int    VERBIOAPI vox_checkwrd(const char *word, const char *defasrlng, char* hmm, int size);
int    VERBIOAPI vox_prevcb(const char *fileName, unsigned int mode);
int    VERBIOAPI vox_prevcbex(const char* fileName, unsigned int mode, int* lpiword);
int    VERBIOAPI vox_prevcbex2(const char* fileName, unsigned int mode, int* lpiword, const char *language);
int    VERBIOAPI vox_prevcbdev(int dev, const char* fileName, unsigned int mode, int* lpiword, const char *language);
int    VERBIOAPI vox_setvcb(int dev, const char *fileName, unsigned int mode);
int    VERBIOAPI vox_setcd(int dev, unsigned int mode);
int    VERBIOAPI vox_loadvcb(int dev, const char *fileName, unsigned int mode);
int    VERBIOAPI vox_loadcd(int dev, unsigned int mode);
int    VERBIOAPI vox_activatevcb(int dev, int vcbhandle, unsigned int mode);
int    VERBIOAPI vox_deactivatevcb(int dev, int vcbhandle, unsigned int mode);
int    VERBIOAPI vox_unloadvcb(int dev, int vcbhandle, unsigned int mode);
int    VERBIOAPI vox_recstr(int dev, const char *fileName, VX_RSP *rspp, unsigned int mode);
int    VERBIOAPI vox_recstrm(int dev, const void *samp, int nsamp, VX_RSP *rspp, unsigned int mode);
int    VERBIOAPI vox_recind(int dev, int maxind, int *index, float *score, unsigned int mode);
int    VERBIOAPI vox_playstr(int dev, const char* fileName, const char* string, unsigned int mode);
const char* VERBIOAPI vox_word(int dev, int ind);
const char* VERBIOAPI vox_wordex(int dev, int ind, int pos);
const char* VERBIOAPI vox_wordrule(int dev, char *rule, int pos, float *score);
void   VERBIOAPI vox_clrrsp(VX_RSP *rspp);
long   VERBIOAPI ATVOX_LASTERR(int dev);
const char* VERBIOAPI ATVOX_ERRMSGP(int dev);
int    VERBIOAPI ATVOX_NIND(int dev);
int    VERBIOAPI ATVOX_IVCB(int dev);
int    VERBIOAPI ATVOX_BUILTIN(int dev);
int    VERBIOAPI vox_getparm(int dev, unsigned long parm, void *valuep);
int    VERBIOAPI vox_setparm(int dev, unsigned long parm, const void *valuep);
int    VERBIOAPI vox_set_lasterr(int dev, long error);
int    VERBIOAPI vox_nbest(int dev, int maxind, int *index, float *score, int ibest, unsigned int mode);
int    VERBIOAPI vox_termplaystr(int dev);
int    VERBIOAPI vox_termrecstr(int dev);
long   VERBIOAPI vox_GetDllVersion(unsigned long *fileverp, unsigned long *prodverp);
const char* VERBIOAPI vox_SerialNumber(void);
int    VERBIOAPI vox_ApplyDictionary(const char *inFileName, const char *outFileName, const char *dicFileName);
int    VERBIOAPI vox_playstr_open(int dev, const char* string, unsigned int mode);
int    VERBIOAPI vox_playstr_read(int playdev, void *buffer, unsigned int count);
int    VERBIOAPI vox_playstr_close(int dev, int playdev);
int    VERBIOAPI vox_recstr_open(int dev, VX_RSP *rspp, unsigned int mode);
int    VERBIOAPI vox_recstr_write(int recdev, const void *buffer, unsigned int count);
int    VERBIOAPI vox_recstr_close(int dev, int recdev);
long   VERBIOAPI vox_lseek(int lh, long offset, int origin);
int    VERBIOAPI vox_testrecstr(int ndevs, int* dev, const char* fileName, VX_RSP* rspp, unsigned int mode);
int    VERBIOAPI vox_recstr_wait(int dev, int ms);
int    VERBIOAPI vox_recstr_release(int dev);
int    VERBIOAPI vox_playstr_wait(int dev, int ms);
int    VERBIOAPI vox_playstr_release(int dev);
int    VERBIOAPI vox_reccfg_wait(int dev, int ms, const char *config);
int    VERBIOAPI vox_reccfg_release(int dev, const char *config);
int    VERBIOAPI vox_ttsSetDictionary(int dev, int mcLang, const char *filename);
int    VERBIOAPI vox_ttsSetAbbreviations(int dev, int mcLang, const char *filename);
int    VERBIOAPI vox_SetDictionary(int dev, const char *mcLang, const char *filename);
int    VERBIOAPI vox_SetAbbreviations(int dev, const char *mcLang, const char *filename);
void   VERBIOAPI vox_srvclose(const char* server);
int    VERBIOAPI vox_putfile(int dev, const char *lfilename, char *rfilename);
int	   VERBIOAPI vox_RegisterVVICallback(int dev, int (*)(const char *result, unsigned long score, size_t ntfyId));

//------- VAD - Voice Activity Detection -----------------
typedef struct _VAD_PARAM VAD_PARAM;
VAD_PARAM* VERBIOAPI vox_vsd_open(int dev, int recdev, VAD_PRM *prm, unsigned int mode);
int    VERBIOAPI vox_vsd_write(VAD_PARAM* vsddev, const void *buffer, unsigned int count);
int    VERBIOAPI vox_vsd_close(int dev, VAD_PARAM* vsddev);
void   VERBIOAPI vox_clrvad (VAD_PRM *prm);
//-------------------------------------------------------
//------- DTMF - DTMF Signal Detection ------------------
typedef struct _DTMF_REG DTMF_REG;
DTMF_REG* VERBIOAPI vox_dtmf_open(int dev, unsigned int mode);
const char* VERBIOAPI vox_dtmf_write(DTMF_REG* dtmfdev, const void *buffer, unsigned int count);
int    VERBIOAPI vox_dtmf_close(int dev, DTMF_REG* dtmfdev);
const char* VERBIOAPI vox_dtmf_word(int dev, const char *sequence);
//--------------------------------------------------------
//------- SPK ID - Speaker Verification ------------------
int    VERBIOAPI vox_spk_init();
int    VERBIOAPI vox_spk_addfile (int dev, const char *id, const char *filename, const char *transcription);
int	   VERBIOAPI vox_spk_train (int dev, const char *id, const char *filename, int *lpiword); 
int	   VERBIOAPI vox_spk_verify (int dev, const char *id, const char *filename, const char *transcription, float *score);
int    VERBIOAPI vox_spk_exist (int dev, const char *id);
//--------------------------------------------------------

//#ifdef _WIN32
typedef void (*VX_SRVCLOSE) (const char *server);
static int VERBIOAPI vox_regsrvclose(VX_SRVCLOSE function) {
	return vox_setparm(-1, VXGB_SRVCLOSE, (void *) function);
}
//#endif

typedef struct _HSAPI *HSAPI;
#ifndef _MODEATTRIBUTES
#define _MODEATTRIBUTES
#define SAPI_AGE_SIZE		16
#define SAPI_GENDER_SIZE	16
#define SAPI_NAME_SIZE		256
#define SAPI_VENDOR_SIZE	256
typedef struct {

	char               age[SAPI_AGE_SIZE];          //Child, Teen, Adult, Senior
	char               gender[SAPI_GENDER_SIZE];    //Male, Female
	unsigned short int language;
	char               name[SAPI_NAME_SIZE];
	char               vendor[SAPI_VENDOR_SIZE];

} MODEATTRIBUTES, *PMODEATTRIBUTES;
#endif
int    VERBIOAPI vox_SapiOpenEx(int dev, PMODEATTRIBUTES ttsattributes, HSAPI* pmode);
int    VERBIOAPI vox_SapiSelect(int dev, HSAPI pmode);
int    VERBIOAPI vox_SapiClose(int dev, HSAPI pmode);

/* Obsolete */
#ifdef SAPI
#ifdef _WIN32
#include "speech.h"
#else
typedef struct TTSMODEINFO *PTTSMODEINFO;
#endif
int    VERBIOAPI vox_SapiOpen(int dev, PTTSMODEINFO ttsmode, HSAPI* pmode, unsigned int mode);
#endif /* SAPI */

/* Obsolete */
int    VERBIOAPI vox_libinit(int options);
int    VERBIOAPI vox_libinit2(int options, const char* cfg);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _VOXLIB_H_ */
