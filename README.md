# Orange_Pi_Zero
Orange Pi Zero examples

============	INSTALL	ALSA LIB  =================== <br>
<b>install alsa:</b>		sudo apt-get install libasound2-dev	<br>
			sudo apt-get install libasound-dev	=> test ok on default xenial	<br>
			
<b>install c lib:</b>		sudo apt-get install libc6-dev	<br>


============	TEST ALSA LIB	===================	<br>
<b>play wav file:</b>		sudo aplay ChillingMusic.wav	<br>

<b>test MIC:</b>		arecord -d 5 test-mic.wav

aplay test-mic.wav		<br>




			UART

- show uart com:	dmesg|grep -u tty

- /dev/ttyS1:	pin 8 (Tx) & pin 10 (Rx)
- /dev/ttyS2:	pin 11 (Rx) & pin 13 (Tx)
