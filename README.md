# Orange_Pi_Zero
Orange Pi Zero examples

============	INSTALL	ALSA LIB  =================== <br>
install alsa:		sudo apt-get install libasound2-dev	<br>
			sudo apt-get install libasound-dev	=> test ok on default xenial	<br>
install c lib:		sudo apt-get install libc6-dev	<br>


============	TEST ALSA LIB	===================	<br>
play wav file:		sudo aplay ChillingMusic.wav	<br>

test MIC		arecord -d 5 test-mic.wav	<br>
			aplay test-mic.wav		<br>
