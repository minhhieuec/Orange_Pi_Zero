compile:
	gcc -o blink blink.c -lwiringPi -lpthread
	
or
	gcc -Wall -o blink blink.c -lwiringPi

RUN:
	./blink

OUTPUT:

	LED ON
	LED OFF
	LED ON
	LED OFF
	...