#ifndef VERBIO_H__
#define VERBIO_H__

#define SAPI

#ifdef _WINDOWS
#include <windows.h>
#endif

#include "voxlib.h"

//Global parameters

/* Engines */
#define VERBIO_ASR_ENGINE                "asr"
#define VERBIO_TTS_ENGINE                "tts"

/* Languages */
#define VERBIO_LANG_SPANISH               "es"
#define VERBIO_LANG_CATALAN               "ca"
#define VERBIO_LANG_CATALAN_VALENCIAN     "ca-va"
#define VERBIO_LANG_BASQUE                "eu"
#define VERBIO_LANG_GALICIAN              "ga"
#define VERBIO_LANG_SPANISH_MEXICAN       "es-mx"
#define VERBIO_LANG_SPANISH_COLOMBIAN     "es-co"
#define VERBIO_LANG_SPANISH_ARGENTINIAN   "es-ar"
#define VERBIO_LANG_SPANISH_VENEZUELAN    "es-ve"
#define VERBIO_LANG_SPANISH_CHILEAN       "es-cl"
#define VERBIO_LANG_PORTUGUESE            "pt"
#define VERBIO_LANG_PORTUGUESE_BRAZILIAN  "pt-br"
#define VERBIO_LANG_ENGLISH               "en-us"
#define VERBIO_LANG_ARABIAN               "ar-ma"
#define VERBIO_LANG_FRENCH                "fr"

/* ASR Configurations */
#define VERBIO_CFG_SPANISH							"es"
#define VERBIO_CFG_SPANISH16K						"es16k"
#define VERBIO_CFG_CATALAN16K						"ca16k"
#define VERBIO_CFG_SPANISH_CATALAN					"es_ca"
#define VERBIO_CFG_SPANISH_GALICIAN					"es_ga"
#define VERBIO_CFG_SPANISH_BASQUE					"es_eu"
#define VERBIO_CFG_SPANISH_CATALAN_BASQUE_GALICIAN  "es_ca_eu_ga"
#define VERBIO_CFG_SPANISH_MEXICAN					"es-mx"
#define VERBIO_CFG_SPANISH_COLOMBIAN				"es-co"
#define VERBIO_CFG_SPANISH_ARGENTINIAN				"es-ar"
#define VERBIO_CFG_SPANISH_VENEZUELAN				"es-ve"
#define VERBIO_CFG_SPANISH_CHILEAN					"es-cl"
#define VERBIO_CFG_PORTUGUESE						"pt"
#define VERBIO_CFG_PORTUGUESE_BRAZILIAN				"pt-br"
#define VERBIO_CFG_ENGLISH							"en-us"
#define VERBIO_CFG_ARABIAN							"ar-ma"
#define VERBIO_CFG_FRENCH							"fr"

#define VERBIO_CFG_MAXWORDS                         "words"

#define VERBIO_AUDIO_LIN16			0x00200000	//PCM 8 Khz, 16 bits
#define VERBIO_AUDIO_MULAW			0x00000000	//Mu-Law 8Khz, 8 bits
#define VERBIO_AUDIO_ALAW			0x00000020	//A-Law 8Khz, 8 bits
#define VERBIO_AUDIO_TONE			0x00000200	//Play tone before recognition
#define VERBIO_AUDIO_CSP			0x00008000	//Use bargein capabilities
#define VERBIO_AUDIO_NOSILENCE		0x00004000	//Do not append automatic final silence in TTS

//BargeIn para Dialogic
struct CSPParm {
	int	VADinit;	
	int	BargeIn;
	int	XferSize;
	int	NLP;
	int TapLength;
	int	Threshold1;
	int Trigg1;
	int Window1;
	int	Threshold2;
	int Trigg2;
	int Window2;

	CSPParm () {
		VADinit	   =    1;	
		BargeIn	   =    1;
		XferSize   = 2048;
		NLP        =    1;
		TapLength  =  128;
		Threshold1 =  -35;  // Default = -40
		Trigg1     =    5;  // Default =  10
		Window1    =   10;  // Default =  10
		Threshold2 =  -35;  // Default = -40
		Trigg2     =    5;  // Default =  10
		Window2    =   10;  // Default =  10
	}
};

#endif
