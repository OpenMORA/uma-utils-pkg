VoiceFestival module in OpenMORA
========================================

This module requires installing the "festival" Text To Speech package to work at run-time. 
However, it can be compiled without problems even without festival installed in the system.
The "festival" App is designed to work under linux systems, so in windows use Cygwin.
For more info see: http://www.cstr.ed.ac.uk/projects/festival/download.html


Installing festival under Debian / Ubuntu: 
========================================
$ sudo apt-get install festival


Configuration for non-English voices: 
=========================================
$ festival

(voice_JuntaDeAndalucia_es_pa_diphone)
(SayText "Hola a todos, esto es una prueba de la nueva voz")

OR: 

printf "(voice_JuntaDeAndalucia_es_pa_diphone)\n(SayText \"Hola a todos\")\n" | festival --pipe 


High quality free Spanish voices:
=========================================
Under a request from the Junta de Andalucia (Spain), two companies sited in 
Sevilla (Indisys & MP-Sistemas) have released to the public two high quality 
voices, available for download here: 

http://forja.guadalinex.org/frs/?group_id=21





