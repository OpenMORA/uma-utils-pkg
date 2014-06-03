
/** \file VoiceEngineHandler_Audio.cpp
 *\brief Implementation of WIN/LIN-specific audio I/O.
 */

#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <errno.h>

#ifndef _WIN32
	// non-Win32: Pulse library
	#include <pulse/simple.h>
	#include <pulse/simple.h>
	#include <pulse/error.h>
	#include <pulse/gccmacro.h>
#endif

#include "VoiceEngineHandler.hpp"
#include "VerbioASR.h"
#include "VerbioTTS.h"


#include <mrpt/utils.h>

using namespace std;


// return false on error
bool VerbioHandler::InitializeAudioChannels()
{
#ifdef _WIN32
	// --------------------------------------
	// Win32 version
	// --------------------------------------
	return !((m_ASRResource->AssignAudioChannel(0) == -1) || (m_TTSResource->AssignAudioChannel(0) == -1));

#else
	// --------------------------------------
	// Linux version
	// --------------------------------------
	DeinitializeAudioChannels(); // Just in case this is called twice...

	// Open connection to Pulse audio server:
	pa_sample_spec ss;

	ss.format 	= PA_SAMPLE_ALAW;
	ss.rate		= 8000;
	ss.channels = 1;

	char ss_str_desc[PA_SAMPLE_SPEC_SNPRINT_MAX];
	// Pretty print a sample type specification to a string
	printf("Initializing PulseAudio playback with: %s\n",pa_sample_spec_snprint(ss_str_desc, sizeof(ss_str_desc), &ss));

	m_pulse_session_play = pa_simple_new(
		NULL,               // Use the default server.
		"MORA pVoice",      // Our application's name.
		PA_STREAM_PLAYBACK,
		NULL,               // Use the default device.
		"TTS audio",        // Description of our stream.
		&ss,                // Our sample format.
		NULL,               // Use default channel map
		NULL,               // Use default buffering attributes.
		NULL                // Ignore error code.
		);

	if (!m_pulse_session_play)
	{
		printf("[InitializeAudioChannels] **ERROR** initializing Pulse for playback.\n");
		return false;
	}


	// Pretty print a sample type specification to a string
	ss.format 	= PA_SAMPLE_S16LE;
	ss.rate		= 8000;
	ss.channels = 1;
	printf("Initializing PulseAudio record with: %s\n",pa_sample_spec_snprint(ss_str_desc, sizeof(ss_str_desc), &ss));

	m_pulse_session_rec = pa_simple_new(
		NULL,               // Use the default server.
		"MORA pVoice",      // Our application's name.
		PA_STREAM_RECORD,
		NULL,               // Use the default device.
		"ASR audio",        // Description of our stream.
		&ss,                // Our sample format.
		NULL,               // Use default channel map
		NULL,               // Use default buffering attributes.
		NULL                // Ignore error code.
		);

	if (!m_pulse_session_rec)
	{
		printf("[InitializeAudioChannels] **ERROR** initializing Pulse for record.\n");
		return false;
	}


	return true;
#endif
}

void VerbioHandler::DeinitializeAudioChannels()
{
#ifdef _WIN32
	return;
#else
	if (m_pulse_session_play)
	{
		pa_simple_free(m_pulse_session_play);
		m_pulse_session_play=NULL;
	}
	if (m_pulse_session_rec)
	{
		pa_simple_free(m_pulse_session_rec);
		m_pulse_session_rec=NULL;
	}
#endif
}


bool VerbioHandler::ReproduceString(const string &str2Reproduce)
{
	if (str2Reproduce.empty())
		return true;	// Nothing to SAY.

#ifdef _WIN32
	if(m_TTSResource->PlayStr(str2Reproduce.c_str(),VERBIO_AUDIO_LIN16) <0)
	{
		printf("[VerbioHandler::ReproduceString] Unable to reproduce the string %s: %s\n",str2Reproduce.c_str(),m_TTSResource->GetError());
		return false;
    }
    else return true;

#else
	// Linux version ================================
	if (m_pulse_session_play==NULL)
	{
		printf("[VerbioHandler::ReproduceString] Unable to reproduce the string %s: Pulse audio not init.\n",str2Reproduce.c_str());
		return false;
	}

	printf("REPRODUCIENDO: '%s'\n",str2Reproduce.c_str());

	//const int dev=0; // Verbio channel:

	int tts_stream = m_TTSResource->PlayStrOpen(str2Reproduce.c_str(), MC_ALAW);
	if (tts_stream==-1)
	{
		printf("[VerbioHandler::ReproduceString] Error in PlayStrOpen\n");
		return false;
	}

	vector<char>	buffer_snd;
	size_t  buffer_real_len=0;
	size_t  tts_read;

	do
	{
		size_t bytes_to_read = 10000;
		buffer_snd.resize(buffer_real_len+bytes_to_read+10);
		tts_read= m_TTSResource->PlayStrRead(/*tts_stream,*/ &buffer_snd[buffer_real_len],bytes_to_read);
		if (tts_read>=0)
			buffer_real_len+=tts_read;
		else
		{
			printf("[VerbioHandler::ReproduceString] ERROR in PlayStrRead\n");
		}
	} while (tts_read>0);

	buffer_snd.resize(buffer_real_len);

	// done with verbio here:
	m_TTSResource->PlayStrClose(/*tts_stream*/);

	if (buffer_snd.empty())
	{
		printf(__FILE__": ERROR: read 0 bytes froom PlayStrClose\n");
		return false;
	}

	// Send to pulse audio:
	// ------------------------
	int paErr=0;
	if (pa_simple_write(m_pulse_session_play, &buffer_snd[0], buffer_snd.size(), &paErr) < 0)
	{
		fprintf(stderr, __FILE__": pa_simple_write() failed\n"); //, pa_strerror(paErr));
		return false;
	}

	// Wait until all data already written is played by the daemon
	pa_simple_drain(m_pulse_session_play,&paErr);

	return true;
#endif

}


#ifndef _WIN32
// Callback from verbio lib:
volatile bool has_to_stop_recording = false;
int StopRecStr(int a)
{
	printf("\nstop recording\n");
	fflush(stdout);
	has_to_stop_recording =true;
	return 0;
}
#endif

extern volatile bool verbio_has_to_stop_recording;

/** A cross platform version of Win32 "RecStr":
  * \param maxtime In millisecs.
  * \param initsil In millisecs.
  * \param maxsil In millisecs.
  * \return false on error
  */
bool VerbioHandler::RecStr(int maxtime, int initsil, int maxsil)
{
#ifdef _WIN32
	// --------------------------------------
	// Win32 version
	// --------------------------------------
	return m_ASRResource->RecStr(maxtime, initsil, maxsil,VERBIO_AUDIO_LIN16,"recordedFile.pcm")>=0;

#else
	// --------------------------------------
	// Linux version
	// --------------------------------------
	if (m_pulse_session_rec==NULL)
	{
		printf("[VerbioHandler::RecStr] Pulse audio record not init.\n");
		return false;
	}

	int ret = m_ASRResource->RecStrOpen(initsil,maxsil,VERBIO_AUDIO_LIN16, StopRecStr);
	printf("RecStrOpen: %i\n",ret); fflush(stdout);

	const static size_t bytes_to_read = 1024;
	char bufData[bytes_to_read];

	has_to_stop_recording=false;

	static bool first =true;
    static mrpt::utils::CTicTac tic;
    if (first) tic.Tic();

	while (!has_to_stop_recording && !verbio_has_to_stop_recording)
	{
		// Grab audio data and send to verbio:
		int paErr=0;
		if (pa_simple_read(m_pulse_session_rec,bufData,bytes_to_read,&paErr)<0)
		{
			m_ASRResource->RecStrClose();
			fprintf(stderr, __FILE__": pa_simple_read() failed: %s\n", pa_strerror(paErr));
			return false;
		}

		if (first)
		{
		    const double t=tic.Tac();
		    if (t>2.0)
		    {
                first = false;
                m_ASRResource->RecStrClose();
                return false;
            }
		}

		// Send to verbio:
		ret = m_ASRResource->RecStrWrite(bufData,bytes_to_read);
	//	printf("."); fflush(stdout);
	}

	m_ASRResource->RecStrClose();

	return true;
#endif
}

