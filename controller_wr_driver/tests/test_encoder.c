#include <tests.h>
#include <encoder.h>


static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};


void testEncoderRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    encoderInit( );


    rawEncoderValue_t       enc_ticks           = 0;
    encoderValue_t          enc_revs            = 0;
    rawEncoderValue_t       enc_table_value     = 0;
    rawEncoderValue_t       prev_enc_table_val  = 0;
    encoderValue_t          enc_rotate_dir      = 0;
    encoderValue_t          enc_distance        = 0;

    chprintf( (BaseSequentialStream *)&SD7, "TEST ENCODER\n\r" );

    while( 1 )
    {
        enc_ticks           = getEncoderRawTickNumber( );
        enc_revs            = getEncoderRevNumber( );
        enc_table_value     = getEncoderValTable();
        enc_rotate_dir      = getEncoderDirectionState();
        enc_distance        = getEncoderDistanceCm();

        /* Show info only when encoder works */
        if(prev_enc_table_val != enc_table_value)
          chprintf( (BaseSequentialStream *)&SD7, "Ticks:(%d)\tRevs:(%d)\tDir:(%c)\tDist:(%d)\n\r\t",
                    enc_ticks, enc_revs, enc_rotate_dir, enc_distance );

        prev_enc_table_val = enc_table_value;
        chThdSleepMilliseconds( 1 );
    }
}
