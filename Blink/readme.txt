compile:
	gcc -o blink blink.c -lwiringPi -lpthread
	
or
	gcc -Wall -o blink blink.c -lwiringPi

RUN:
	sudo ./blink

OUTPUT:

LED ON
LED OFF