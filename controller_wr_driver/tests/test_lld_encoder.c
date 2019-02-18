#include <tests.h>
#include <lld_encoder.h>

/***************************************************/

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

//#define TEST_NULL_POINT

void testEncoderCommonRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    lldEncoderInit( );

    rawEncoderValue_t   enc_revs        = 0;
    rawEncoderValue_t   enc_ticks       = 0;
    rawEncoderValue_t   enc_curt_state  = 0;
    rawEncoderValue_t   enc_prev_state  = 0;
    dirEncoderValue_t   enc_dir_test    = 0;


    chprintf( (BaseSequentialStream *)&SD7, "TEST ENCODER\n\r" );

    while( 1 )
    {
        enc_revs        = getEncoderRevsNumber( );
        enc_ticks       = getEncoderRawTickNumber( );
        enc_curt_state  = getEncoderState( );
        enc_dir_test    = getEncoderDirection( );

        if(enc_curt_state != enc_prev_state)
        {
            chprintf( (BaseSequentialStream *)&SD7, "State:(%d)\tDir:(%c)\n\r",
                      enc_curt_state, enc_dir_test);
        }


        enc_prev_state = enc_curt_state;

        chThdSleepMilliseconds( 1 );

    }

}
