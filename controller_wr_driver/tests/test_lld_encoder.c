#include <tests.h>
#include <lld_encoder.h>

//#define MATLAB_ENCODER

#ifdef MATLAB_ENCODER

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};
#endif

/**
 * @brief   Test ticks and revs counting, also direction detection
 * @note    chprintf works only when encoder is rotation
 */
void testEncoderCommonRoutine( void )
{
#ifdef MATLAB_ENCODER
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    int32_t matlab_revs = 0;
    uint8_t matlab_start_flag = 0;
#else
    debug_stream_init( );
#endif
    lldEncoderInit( );

    rawEncoderValue_t       enc_test_ticks       = 0;
    rawRevEncoderValue_t    enc_test_revs        = 0;
#ifdef ABSOLUTE_ENCODER
    rawEncoderValue_t   enc_test_abs_revs    = 0;
#endif
    uint8_t                enc_test_dir         = 0;

    systime_t time = chVTGetSystemTimeX();

    while( 1 )
    {
#ifdef MATLAB_ENCODER
        time += MS2ST(10);
#else
        time += MS2ST(100);
#endif

        enc_test_ticks      = lldGetEncoderRawTicks( );
        enc_test_revs       = lldGetEncoderRawRevs( );
#ifdef ABSOLUTE_ENCODER
        enc_test_abs_revs   = lldGetAbsoluteEncoderRawRevs( );
#endif
        enc_test_dir        = lldGetEncoderDirection( );

#ifdef MATLAB_ENCODER
        char rc_data    = sdGetTimeout( &SD7, TIME_IMMEDIATE );

        if( rc_data == 'p') matlab_start_flag = 1;

        if( matlab_start_flag == 1 )
        {
          matlab_revs = (int)(enc_test_revs * 500);
          sdWrite(&SD7, (uint8_t*) &matlab_revs, 2);
        }
        chThdSleepUntil(time);
#else

#ifdef ABSOLUTE_ENCODER
        dbgprintf( "Ts:(%d)\tRs:(%d)\tA_Rs:(%d)\tD:(%d)\n\r",
                  enc_test_ticks, enc_test_revs, enc_test_abs_revs, enc_test_dir);
        chThdSleepUntil(time);
#else
        dbgprintf( "Ts:(%d)\tRs:(%d)\tD:(%d)\n\r",
                   enc_test_ticks, (int)enc_test_revs, enc_test_dir );
        chThdSleepUntil(time);
#endif

#endif
    }
}
