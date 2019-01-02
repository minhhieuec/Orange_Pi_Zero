/*
 *  Small program to read a 16-bit, signed, 44.1kHz wave file and play it.
 *  Written by Brian Fraser, heavily based on code found at:
 *  http://www.alsa-project.org/alsa-doc/alsa-lib/_2test_2pcm_min_8c-example.html
 */
#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>


#define SAMPLE_RATE   44100
#define NUM_CHANNELS  1
#define SAMPLE_SIZE   (sizeof(short))   // bytes per sample

#define BUFF_LEN	160000

short PCMAudioBuff[BUFF_LEN] = {0};

// Prototypes:
snd_pcm_t *Audio_openDevice();
void Audio_playFile(snd_pcm_t *handle, short *audio_input);

snd_pcm_t * audio_capture_config();
void Audio_getdatafromMIC (snd_pcm_t *handle, short *audio_input);


int main(void)
{

  printf("Beginning play-back of %s\n", SOURCE_FILE);

  // Config audio input
  snd_pcm_t *capture_handle = audio_capture_config();


  // Configure Output Device
  snd_pcm_t *handle = Audio_openDevice();


  //Audio_readWaveFileIntoMemory(SOURCE_FILE, &sampleFile);
  Audio_getdatafromMIC(capture_handle, &PCMAudioBuff[0]);


  // Play Audio
  Audio_playFile(handle, &PCMAudioBuff[0]);

  // Cleanup, letting the music in buffer play out (drain), then close and free.
  snd_pcm_drain(handle);
  snd_pcm_hw_free(handle);
  snd_pcm_close(handle);

  printf("Done!\n");
  return 0;
}


snd_pcm_t * audio_capture_config()
{
	snd_pcm_t *handle;

	// Open the PCM output
	int err = snd_pcm_open(&handle, "default", SND_PCM_STREAM_CAPTURE, 0);
	if (err < 0) {
		printf("cannot open audio device: %s\n", snd_strerror(err));
		exit(EXIT_FAILURE);
	}


	// Configure parameters of PCM output
	err = snd_pcm_set_params(handle,
	  SND_PCM_FORMAT_S16_LE,
	  SND_PCM_ACCESS_RW_INTERLEAVED,
	  NUM_CHANNELS,
	  SAMPLE_RATE,
	  1,      // Allow software resampling
	  50000);   // 0.05 seconds per buffer
	if (err < 0) {
	printf("Play-back configuration error: %s\n", snd_strerror(err));
	exit(EXIT_FAILURE);
	}


  return handle;
}


void Audio_getdatafromMIC(snd_pcm_t *handle, short *audio_input){

  snd_pcm_sframes_t frames = snd_pcm_readi (handle, audio_input, BUFF_LEN);

  if (frames != BUFF_LEN){
  	printf("frames != pWaveStruct->numSamples\n");
      exit (1);
  }

  printf("Audio_getdatafromMIC done! \n");
}



// Open the PCM audio output device and configure it.
// Returns a handle to the PCM device; needed for other actions.
snd_pcm_t *Audio_openDevice()
{
  snd_pcm_t *handle;

  // Open the PCM output
  int err = snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
  if (err < 0) {
    printf("Play-back open error: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }

  // Configure parameters of PCM output
  err = snd_pcm_set_params(handle,
      SND_PCM_FORMAT_S16_LE,
      SND_PCM_ACCESS_RW_INTERLEAVED,
      NUM_CHANNELS,
      SAMPLE_RATE,
      1,      // Allow software resampling
      50000);   // 0.05 seconds per buffer
  if (err < 0) {
    printf("Play-back configuration error: %s\n", snd_strerror(err));
    exit(EXIT_FAILURE);
  }

  return handle;
}


// Play the audio file (blocking)
void Audio_playFile(snd_pcm_t *handle, short *audio_input)
{
  // If anything is waiting to be written to screen, can be delayed unless flushed.
  //fflush(stdout);

  // Write data and play sound (blocking)
  snd_pcm_sframes_t frames = snd_pcm_writei(handle, audio_input, BUFF_LEN);

  // Check for errors
  if (frames < 0)
    frames = snd_pcm_recover(handle, frames, 0);
  if (frames < 0) {
    fprintf(stderr, "ERROR: Failed writing audio with snd_pcm_writei(): %li\n", frames);
    exit(EXIT_FAILURE);
  }
  if (frames > 0 && frames < BUFF_LEN)
    printf("Short write (expected %d, wrote %li)\n", BUFF_LEN, frames);
}