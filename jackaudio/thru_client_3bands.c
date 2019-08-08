/** @file thru_client.c
 *
 * @brief This simple through client demonstrates the basic features of JACK
 * as they would be used by many applications.
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#ifndef WIN32
#include <unistd.h>
#endif
#include <jack/jack.h>

#if 1
/**
 * @brief 32-bit floating-point type definition.
 */
typedef float float32_t;
/**
 * @brief Instance structure for the floating-point Biquad cascade filter.
 */
typedef struct {
  uint32_t numStages; /**< number of 2nd order stages in the filter.  Overall
                         order is 2*numStages. */
  float32_t *pState;  /**< Points to the array of state coefficients.  The array
                         is of length 4*numStages. */
  float32_t *pCoeffs; /**< Points to the array of coefficients.  The array is of
                         length 5*numStages. */
} arm_biquad_casd_df1_inst_f32;

/**
 * @brief  Initialization function for the floating-point Biquad cascade filter.
 * @param[in,out] S          points to an instance of the floating-point Biquad
 * cascade structure.
 * @param[in]     numStages  number of 2nd order stages in the filter.
 * @param[in]     pCoeffs    points to the filter coefficients.
 * @param[in]     pState     points to the state buffer.
 */
void arm_biquad_cascade_df1_init_f32(arm_biquad_casd_df1_inst_f32 *S,
                                     uint8_t numStages, float32_t *pCoeffs,
                                     float32_t *pState);

/**
 * @brief Processing function for the floating-point Biquad cascade filter.
 * @param[in]  S          points to an instance of the floating-point Biquad
 * cascade structure.
 * @param[in]  pSrc       points to the block of input data.
 * @param[out] pDst       points to the block of output data.
 * @param[in]  blockSize  number of samples to process.
 */
void arm_biquad_cascade_df1_f32(const arm_biquad_casd_df1_inst_f32 *S,
                                float32_t *pSrc, float32_t *pDst,
                                uint32_t blockSize);

float32_t mic_bass_coeff[5] = {1, -1.98565211393795, 0.985853668419449, 1.98565211393795, -0.985853668419449}; // 0db
// {1.00862723842334, -1.99181805036279, 0.98339299229701, 1.99181805036279, -0.992020230720351}; // 10db
float32_t mic_mid_coeff[5] = //{1, -1.84849691613332, 0.867418585785029, 1.84849691613332, -0.867418585785029};   //  0db
{1.08301385166895, -1.9037292474287, 0.84020243715633, 1.9037292474287, -0.923216288825285};   //  10db
float32_t mic_treb_coeff[5] = //{1,  -0.194716511703197, 0.33808110131236, 0.194716511703197, -0.33808110131236}; // 0db
{1.47058985695729,  -0.227697937206526, 0.0941377766615551, 0.227697937206526, -0.564727633618849}; // 10db

#define NUMSTAGES 1

arm_biquad_casd_df1_inst_f32 s_mic_bass_eq;
arm_biquad_casd_df1_inst_f32 s_mic_mid_eq;
arm_biquad_casd_df1_inst_f32 s_mic_treb_eq;

float32_t biquadStateBandBassF32[4 * NUMSTAGES];
float32_t biquadStateBandMidF32[4 * NUMSTAGES];
float32_t biquadStateBandTrebF32[4 * NUMSTAGES];

void init_micro_equalizer(void){
    printf ( "init_micro_equalizer\n" );
    arm_biquad_cascade_df1_init_f32(&s_mic_bass_eq, NUMSTAGES,
                                    (float32_t *) &mic_bass_coeff,
                                    &biquadStateBandBassF32[0]);

    arm_biquad_cascade_df1_init_f32(&s_mic_mid_eq, NUMSTAGES,
                                    (float32_t *) &mic_mid_coeff,
                                    &biquadStateBandMidF32[0]);

    arm_biquad_cascade_df1_init_f32(&s_mic_treb_eq, NUMSTAGES,
                                    (float32_t *) &mic_treb_coeff,
                                    &biquadStateBandTrebF32[0]);
}

void micro_equalizer_process(float32_t *audio_input, float32_t *audio_out, uint32_t blockSize){
    arm_biquad_cascade_df1_f32(&s_mic_bass_eq, audio_input, audio_out, 1024);
    arm_biquad_cascade_df1_f32(&s_mic_mid_eq, audio_out, audio_out, 1024);
    arm_biquad_cascade_df1_f32(&s_mic_treb_eq, audio_out, audio_out, 1024);
}

#endif

jack_port_t **input_ports;
jack_port_t **output_ports;
jack_client_t *client;

static void signal_handler ( int sig )
{
    jack_client_close ( client );
    fprintf ( stderr, "signal received, exiting ...\n" );
    exit ( 0 );
}

/**
 * The process callback for this JACK application is called in a
 * special realtime thread once for each audio cycle.
 *
 * This client follows a simple rule: when the JACK transport is
 * running, copy the input port to the output.  When it stops, exit.
 */

int
process ( jack_nframes_t nframes, void *arg )
{
    int i;
    jack_default_audio_sample_t *in, *out;
    for ( i = 0; i < 2; i++ )
    {
        in = jack_port_get_buffer ( input_ports[i], nframes );
        out = jack_port_get_buffer ( output_ports[i], nframes );
        
        micro_equalizer_process(in, out, nframes * sizeof ( jack_default_audio_sample_t ));
        // memcpy ( out, in, nframes * sizeof ( jack_default_audio_sample_t ) );
    }
    return 0;
}

/**
 * JACK calls this shutdown_callback if the server ever shuts down or
 * decides to disconnect the client.
 */
void
jack_shutdown ( void *arg )
{
    free ( input_ports );
    free ( output_ports );
    exit ( 1 );
}

int
main ( int argc, char *argv[] )
{
    int i;
    const char **ports;
    const char *client_name;
    const char *server_name = NULL;
    jack_options_t options = JackNullOption;
    jack_status_t status;

    init_micro_equalizer();

    if ( argc >= 2 )        /* client name specified? */
    {
        client_name = argv[1];
        if ( argc >= 3 )    /* server name specified? */
        {
            server_name = argv[2];
            options |= JackServerName;
        }
    }
    else              /* use basename of argv[0] */
    {
        client_name = strrchr ( argv[0], '/' );
        if ( client_name == 0 )
        {
            client_name = argv[0];
        }
        else
        {
            client_name++;
        }
    }

    /* open a client connection to the JACK server */

    client = jack_client_open ( client_name, options, &status, server_name );
    if ( client == NULL )
    {
        fprintf ( stderr, "jack_client_open() failed, "
                  "status = 0x%2.0x\n", status );
        if ( status & JackServerFailed )
        {
            fprintf ( stderr, "Unable to connect to JACK server\n" );
        }
        exit ( 1 );
    }
    if ( status & JackServerStarted )
    {
        fprintf ( stderr, "JACK server started\n" );
    }
    if ( status & JackNameNotUnique )
    {
        client_name = jack_get_client_name ( client );
        fprintf ( stderr, "unique name `%s' assigned\n", client_name );
    }

    /* tell the JACK server to call `process()' whenever
       there is work to be done.
    */

    jack_set_process_callback ( client, process, 0 );

    /* tell the JACK server to call `jack_shutdown()' if
       it ever shuts down, either entirely, or if it
       just decides to stop calling us.
    */

    jack_on_shutdown ( client, jack_shutdown, 0 );

    /* create two ports pairs*/
    input_ports = ( jack_port_t** ) calloc ( 2, sizeof ( jack_port_t* ) );
    output_ports = ( jack_port_t** ) calloc ( 2, sizeof ( jack_port_t* ) );

    char port_name[16];
    for ( i = 0; i < 2; i++ )
    {
        sprintf ( port_name, "input_%d", i + 1 );
        input_ports[i] = jack_port_register ( client, port_name, JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0 );
        sprintf ( port_name, "output_%d", i + 1 );
        output_ports[i] = jack_port_register ( client, port_name, JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0 );
        if ( ( input_ports[i] == NULL ) || ( output_ports[i] == NULL ) )
        {
            fprintf ( stderr, "no more JACK ports available\n" );
            exit ( 1 );
        }
    }

    /* Tell the JACK server that we are ready to roll.  Our
     * process() callback will start running now. */

    if ( jack_activate ( client ) )
    {
        fprintf ( stderr, "cannot activate client" );
        exit ( 1 );
    }

    /* Connect the ports.  You can't do this before the client is
     * activated, because we can't make connections to clients
     * that aren't running.  Note the confusing (but necessary)
     * orientation of the driver backend ports: playback ports are
     * "input" to the backend, and capture ports are "output" from
     * it.
     */

    ports = jack_get_ports ( client, NULL, NULL, JackPortIsPhysical|JackPortIsOutput );
    if ( ports == NULL )
    {
        fprintf ( stderr, "no physical capture ports\n" );
        exit ( 1 );
    }

    for ( i = 0; i < 2; i++ )
        if ( jack_connect ( client, ports[i], jack_port_name ( input_ports[i] ) ) )
            fprintf ( stderr, "cannot connect input ports\n" );

    free ( ports );

    ports = jack_get_ports ( client, NULL, NULL, JackPortIsPhysical|JackPortIsInput );
    if ( ports == NULL )
    {
        fprintf ( stderr, "no physical playback ports\n" );
        exit ( 1 );
    }

    for ( i = 0; i < 2; i++ )
        if ( jack_connect ( client, jack_port_name ( output_ports[i] ), ports[i] ) )
            fprintf ( stderr, "cannot connect input ports\n" );

    free ( ports );

    /* install a signal handler to properly quits jack client */
#ifdef WIN32
    signal ( SIGINT, signal_handler );
    signal ( SIGABRT, signal_handler );
    signal ( SIGTERM, signal_handler );
#else
    signal ( SIGQUIT, signal_handler );
    signal ( SIGTERM, signal_handler );
    signal ( SIGHUP, signal_handler );
    signal ( SIGINT, signal_handler );
#endif

    /* keep running until the transport stops */

    while (1)
    {
#ifdef WIN32
        Sleep ( 1000 );
#else
        sleep ( 1 );
#endif
    }

    jack_client_close ( client );
    exit ( 0 );
}

/**    
 * @details    
 * @brief  Initialization function for the floating-point Biquad cascade filter.    
 * @param[in,out] *S           points to an instance of the floating-point Biquad cascade structure.    
 * @param[in]     numStages    number of 2nd order stages in the filter.    
 * @param[in]     *pCoeffs     points to the filter coefficients array.    
 * @param[in]     *pState      points to the state array.    
 * @return        none    
 *    
 *    
 * <b>Coefficient and State Ordering:</b>    
 *    
 * \par    
 * The coefficients are stored in the array <code>pCoeffs</code> in the following order:    
 * <pre>    
 *     {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}    
 * </pre>    
 *    
 * \par    
 * where <code>b1x</code> and <code>a1x</code> are the coefficients for the first stage,    
 * <code>b2x</code> and <code>a2x</code> are the coefficients for the second stage,    
 * and so on.  The <code>pCoeffs</code> array contains a total of <code>5*numStages</code> values.    
 *    
 * \par    
 * The <code>pState</code> is a pointer to state array.    
 * Each Biquad stage has 4 state variables <code>x[n-1], x[n-2], y[n-1],</code> and <code>y[n-2]</code>.    
 * The state variables are arranged in the <code>pState</code> array as:    
 * <pre>    
 *     {x[n-1], x[n-2], y[n-1], y[n-2]}    
 * </pre>    
 * The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.    
 * The state array has a total length of <code>4*numStages</code> values.    
 * The state variables are updated after each block of data is processed; the coefficients are untouched.    
 *    
 */

void arm_biquad_cascade_df1_init_f32(
  arm_biquad_casd_df1_inst_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState)
{
  /* Assign filter stages */
  S->numStages = numStages;

  /* Assign coefficient pointer */
  S->pCoeffs = pCoeffs;

  /* Clear state buffer and size is always 4 * numStages */
  memset(pState, 0, (4u * (uint32_t) numStages) * sizeof(float32_t));

  /* Assign state pointer */
  S->pState = pState;
}

/**    
 * @} end of BiquadCascadeDF1 group    
 */

/**    
 * @defgroup BiquadCascadeDF1 Biquad Cascade IIR Filters Using Direct Form I Structure    
 *    
 * This set of functions implements arbitrary order recursive (IIR) filters.    
 * The filters are implemented as a cascade of second order Biquad sections.    
 * The functions support Q15, Q31 and floating-point data types.  
 * Fast version of Q15 and Q31 also supported on CortexM4 and Cortex-M3.    
 *    
 * The functions operate on blocks of input and output data and each call to the function    
 * processes <code>blockSize</code> samples through the filter.    
 * <code>pSrc</code> points to the array of input data and    
 * <code>pDst</code> points to the array of output data.    
 * Both arrays contain <code>blockSize</code> values.    
 *    
 * \par Algorithm    
 * Each Biquad stage implements a second order filter using the difference equation:    
 * <pre>    
 *     y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2]    
 * </pre>    
 * A Direct Form I algorithm is used with 5 coefficients and 4 state variables per stage.    
 * \image html Biquad.gif "Single Biquad filter stage"    
 * Coefficients <code>b0, b1 and b2 </code> multiply the input signal <code>x[n]</code> and are referred to as the feedforward coefficients.    
 * Coefficients <code>a1</code> and <code>a2</code> multiply the output signal <code>y[n]</code> and are referred to as the feedback coefficients.    
 * Pay careful attention to the sign of the feedback coefficients.    
 * Some design tools use the difference equation    
 * <pre>    
 *     y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] - a1 * y[n-1] - a2 * y[n-2]    
 * </pre>    
 * In this case the feedback coefficients <code>a1</code> and <code>a2</code> must be negated when used with the CMSIS DSP Library.    
 *    
 * \par    
 * Higher order filters are realized as a cascade of second order sections.    
 * <code>numStages</code> refers to the number of second order stages used.    
 * For example, an 8th order filter would be realized with <code>numStages=4</code> second order stages.    
 * \image html BiquadCascade.gif "8th order filter using a cascade of Biquad stages"    
 * A 9th order filter would be realized with <code>numStages=5</code> second order stages with the coefficients for one of the stages configured as a first order filter (<code>b2=0</code> and <code>a2=0</code>).    
 *    
 * \par    
 * The <code>pState</code> points to state variables array.    
 * Each Biquad stage has 4 state variables <code>x[n-1], x[n-2], y[n-1],</code> and <code>y[n-2]</code>.    
 * The state variables are arranged in the <code>pState</code> array as:    
 * <pre>    
 *     {x[n-1], x[n-2], y[n-1], y[n-2]}    
 * </pre>    
 *    
 * \par    
 * The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.    
 * The state array has a total length of <code>4*numStages</code> values.    
 * The state variables are updated after each block of data is processed, the coefficients are untouched.    
 *    
 * \par Instance Structure    
 * The coefficients and state variables for a filter are stored together in an instance data structure.    
 * A separate instance structure must be defined for each filter.    
 * Coefficient arrays may be shared among several instances while state variable arrays cannot be shared.    
 * There are separate instance structure declarations for each of the 3 supported data types.    
 *    
 * \par Init Functions    
 * There is also an associated initialization function for each data type.    
 * The initialization function performs following operations:    
 * - Sets the values of the internal structure fields.    
 * - Zeros out the values in the state buffer.    
 * To do this manually without calling the init function, assign the follow subfields of the instance structure:
 * numStages, pCoeffs, pState. Also set all of the values in pState to zero. 
 *    
 * \par    
 * Use of the initialization function is optional.    
 * However, if the initialization function is used, then the instance structure cannot be placed into a const data section.    
 * To place an instance structure into a const data section, the instance structure must be manually initialized.    
 * Set the values in the state buffer to zeros before static initialization.    
 * The code below statically initializes each of the 3 different data type filter instance structures    
 * <pre>    
 *     arm_biquad_casd_df1_inst_f32 S1 = {numStages, pState, pCoeffs};    
 *     arm_biquad_casd_df1_inst_q15 S2 = {numStages, pState, pCoeffs, postShift};    
 *     arm_biquad_casd_df1_inst_q31 S3 = {numStages, pState, pCoeffs, postShift};    
 * </pre>    
 * where <code>numStages</code> is the number of Biquad stages in the filter; <code>pState</code> is the address of the state buffer;    
 * <code>pCoeffs</code> is the address of the coefficient buffer; <code>postShift</code> shift to be applied.    
 *    
 * \par Fixed-Point Behavior    
 * Care must be taken when using the Q15 and Q31 versions of the Biquad Cascade filter functions.    
 * Following issues must be considered:    
 * - Scaling of coefficients    
 * - Filter gain    
 * - Overflow and saturation    
 *    
 * \par    
 * <b>Scaling of coefficients: </b>    
 * Filter coefficients are represented as fractional values and    
 * coefficients are restricted to lie in the range <code>[-1 +1)</code>.    
 * The fixed-point functions have an additional scaling parameter <code>postShift</code>    
 * which allow the filter coefficients to exceed the range <code>[+1 -1)</code>.    
 * At the output of the filter's accumulator is a shift register which shifts the result by <code>postShift</code> bits.    
 * \image html BiquadPostshift.gif "Fixed-point Biquad with shift by postShift bits after accumulator"    
 * This essentially scales the filter coefficients by <code>2^postShift</code>.    
 * For example, to realize the coefficients    
 * <pre>    
 *    {1.5, -0.8, 1.2, 1.6, -0.9}    
 * </pre>    
 * set the pCoeffs array to:    
 * <pre>    
 *    {0.75, -0.4, 0.6, 0.8, -0.45}    
 * </pre>    
 * and set <code>postShift=1</code>    
 *    
 * \par    
 * <b>Filter gain: </b>    
 * The frequency response of a Biquad filter is a function of its coefficients.    
 * It is possible for the gain through the filter to exceed 1.0 meaning that the filter increases the amplitude of certain frequencies.    
 * This means that an input signal with amplitude < 1.0 may result in an output > 1.0 and these are saturated or overflowed based on the implementation of the filter.    
 * To avoid this behavior the filter needs to be scaled down such that its peak gain < 1.0 or the input signal must be scaled down so that the combination of input and filter are never overflowed.    
 *    
 * \par    
 * <b>Overflow and saturation: </b>    
 * For Q15 and Q31 versions, it is described separately as part of the function specific documentation below.    
 */

/**    
 * @addtogroup BiquadCascadeDF1    
 * @{    
 */

/**    
 * @param[in]  *S         points to an instance of the floating-point Biquad cascade structure.    
 * @param[in]  *pSrc      points to the block of input data.    
 * @param[out] *pDst      points to the block of output data.    
 * @param[in]  blockSize  number of samples to process per call.    
 * @return     none.    
 *    
 */

void arm_biquad_cascade_df1_f32(
  const arm_biquad_casd_df1_inst_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  float32_t *pIn = pSrc;                         /*  source pointer            */
  float32_t *pOut = pDst;                        /*  destination pointer       */
  float32_t *pState = S->pState;                 /*  pState pointer            */
  float32_t *pCoeffs = S->pCoeffs;               /*  coefficient pointer       */
  float32_t acc;                                 /*  Simulates the accumulator */
  float32_t b0, b1, b2, a1, a2;                  /*  Filter coefficients       */
  float32_t Xn1, Xn2, Yn1, Yn2;                  /*  Filter pState variables   */
  float32_t Xn;                                  /*  temporary input           */
  uint32_t sample, stage = S->numStages;         /*  loop counters             */


#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  do
  {
    /* Reading the coefficients */
    b0 = *pCoeffs++;
    b1 = *pCoeffs++;
    b2 = *pCoeffs++;
    a1 = *pCoeffs++;
    a2 = *pCoeffs++;

    /* Reading the pState values */
    Xn1 = pState[0];
    Xn2 = pState[1];
    Yn1 = pState[2];
    Yn2 = pState[3];

    /* Apply loop unrolling and compute 4 output values simultaneously. */
    /*      The variable acc hold output values that are being computed:    
     *    
     *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]    
     *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]    
     *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]    
     *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]    
     */

    sample = blockSize >> 2u;

    /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
     ** a second loop below computes the remaining 1 to 3 samples. */
    while(sample > 0u)
    {
      /* Read the first input */
      Xn = *pIn++;

      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
      Yn2 = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

      /* Store the result in the accumulator in the destination buffer. */
      *pOut++ = Yn2;

      /* Every time after the output is computed state should be updated. */
      /* The states should be updated as:  */
      /* Xn2 = Xn1    */
      /* Xn1 = Xn     */
      /* Yn2 = Yn1    */
      /* Yn1 = acc   */

      /* Read the second input */
      Xn2 = *pIn++;

      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
      Yn1 = (b0 * Xn2) + (b1 * Xn) + (b2 * Xn1) + (a1 * Yn2) + (a2 * Yn1);

      /* Store the result in the accumulator in the destination buffer. */
      *pOut++ = Yn1;

      /* Every time after the output is computed state should be updated. */
      /* The states should be updated as:  */
      /* Xn2 = Xn1    */
      /* Xn1 = Xn     */
      /* Yn2 = Yn1    */
      /* Yn1 = acc   */

      /* Read the third input */
      Xn1 = *pIn++;

      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
      Yn2 = (b0 * Xn1) + (b1 * Xn2) + (b2 * Xn) + (a1 * Yn1) + (a2 * Yn2);

      /* Store the result in the accumulator in the destination buffer. */
      *pOut++ = Yn2;

      /* Every time after the output is computed state should be updated. */
      /* The states should be updated as: */
      /* Xn2 = Xn1    */
      /* Xn1 = Xn     */
      /* Yn2 = Yn1    */
      /* Yn1 = acc   */

      /* Read the forth input */
      Xn = *pIn++;

      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
      Yn1 = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn2) + (a2 * Yn1);

      /* Store the result in the accumulator in the destination buffer. */
      *pOut++ = Yn1;

      /* Every time after the output is computed state should be updated. */
      /* The states should be updated as:  */
      /* Xn2 = Xn1    */
      /* Xn1 = Xn     */
      /* Yn2 = Yn1    */
      /* Yn1 = acc   */
      Xn2 = Xn1;
      Xn1 = Xn;

      /* decrement the loop counter */
      sample--;

    }

    /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
     ** No loop unrolling is used. */
    sample = blockSize & 0x3u;

    while(sample > 0u)
    {
      /* Read the input */
      Xn = *pIn++;

      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
      acc = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

      /* Store the result in the accumulator in the destination buffer. */
      *pOut++ = acc;

      /* Every time after the output is computed state should be updated. */
      /* The states should be updated as:    */
      /* Xn2 = Xn1    */
      /* Xn1 = Xn     */
      /* Yn2 = Yn1    */
      /* Yn1 = acc   */
      Xn2 = Xn1;
      Xn1 = Xn;
      Yn2 = Yn1;
      Yn1 = acc;

      /* decrement the loop counter */
      sample--;

    }

    /*  Store the updated state variables back into the pState array */
    *pState++ = Xn1;
    *pState++ = Xn2;
    *pState++ = Yn1;
    *pState++ = Yn2;

    /*  The first stage goes from the input buffer to the output buffer. */
    /*  Subsequent numStages  occur in-place in the output buffer */
    pIn = pDst;

    /* Reset the output pointer */
    pOut = pDst;

    /* decrement the loop counter */
    stage--;

  } while(stage > 0u);

#else

  /* Run the below code for Cortex-M0 */

  do
  {
    /* Reading the coefficients */
    b0 = *pCoeffs++;
    b1 = *pCoeffs++;
    b2 = *pCoeffs++;
    a1 = *pCoeffs++;
    a2 = *pCoeffs++;

    /* Reading the pState values */
    Xn1 = pState[0];
    Xn2 = pState[1];
    Yn1 = pState[2];
    Yn2 = pState[3];

    /*      The variables acc holds the output value that is computed:        
     *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]        
     */

    sample = blockSize;

    while(sample > 0u)
    {
      /* Read the input */
      Xn = *pIn++;

      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
      acc = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

      /* Store the result in the accumulator in the destination buffer. */
      *pOut++ = acc;

      /* Every time after the output is computed state should be updated. */
      /* The states should be updated as:    */
      /* Xn2 = Xn1    */
      /* Xn1 = Xn     */
      /* Yn2 = Yn1    */
      /* Yn1 = acc   */
      Xn2 = Xn1;
      Xn1 = Xn;
      Yn2 = Yn1;
      Yn1 = acc;

      /* decrement the loop counter */
      sample--;
    }

    /*  Store the updated state variables back into the pState array */
    *pState++ = Xn1;
    *pState++ = Xn2;
    *pState++ = Yn1;
    *pState++ = Yn2;

    /*  The first stage goes from the input buffer to the output buffer. */
    /*  Subsequent numStages  occur in-place in the output buffer */
    pIn = pDst;

    /* Reset the output pointer */
    pOut = pDst;

    /* decrement the loop counter */
    stage--;

  } while(stage > 0u);

#endif /*   #ifndef ARM_MATH_CM0_FAMILY         */

}


  /**    
   * @} end of BiquadCascadeDF1 group    
   */