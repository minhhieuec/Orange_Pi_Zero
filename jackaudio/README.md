3 bands equalizer process from mic input audio samples(bass, mid, treble)

to build:
gcc -o thru_client thru_client.c `pkg-config --cflags --libs jack` -lm

to run:

./thru_client
