# Orange_Pi_Zero
Orange Pi Zero examples

============	INSTALL	ALSA LIB  ===================
install alsa:		sudo apt-get install libasound2-dev
			sudo apt-get install libasound-dev	=> test ok on default xenial
install c lib:		sudo apt-get install libc6-dev


============	TEST ALSA LIB	===================
play wav file:		sudo aplay ChillingMusic.wav

test MIC		arecord -d 5 test-mic.wav
			aplay test-mic.wav
