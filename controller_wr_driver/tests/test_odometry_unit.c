#include <tests.h>
#include <odometry_unit.h>


static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};

/**
 * @brief   Test ticks and revs counting, also direction detection
 * @note    chprintf works only when encoder is rotation
 */
//void testEncoderCommonRoutine( void )
//{
//    sdStart( &SD7, &sdcfg );
//    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
//    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX
//
//    encoderInit( );
//
//
//    rawEncoderValue_t       enc_ticks           = 0;
//    encoderValue_t          enc_revs            = 0;
//    rawEncoderValue_t       enc_table_value     = 0;
//    rawEncoderValue_t       prev_enc_table_val  = 0;
//    encoderValue_t          enc_rotate_dir      = 0;
//    encoderValue_t          enc_distance        = 0;
//
//    chprintf( (BaseSequentialStream *)&SD7, "TEST ENCODER\n\r" );
//
//    while( 1 )
//    {
//        enc_ticks           = getEncoderRawTickNumber( );
//        enc_revs            = getEncoderRevNumber( );
//        enc_table_value     = getEncoderValTable();
//        enc_rotate_dir      = getEncoderDirectionState();
//        enc_distance        = getEncoderDistanceCm();
//
//        /* Show info only when encoder works */
//        if(prev_enc_table_val != enc_table_value)
//          chprintf( (BaseSequentialStream *)&SD7, "Ticks:(%d)\tRevs:(%d)\tDir:(%c)\tDist:(%d)\n\r\t",
//                    enc_ticks, enc_revs, enc_rotate_dir, enc_distance );
//
//        prev_enc_table_val = enc_table_value;
//        chThdSleepMilliseconds( 1 );
//    }
//}

/**
 * @brief   Test speed detection [TPS, RPS, MPS, MPM, KPH]
 */
void testEncoderSpeedRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

//    encoderInit( );

//    rawEncoderValue_t       enc_speed_tps       = 0;
//    encoderValue_t          enc_speed_rps       = 0;
//    encSpeedValue_t         wheel_speed_cmps    = 0;
//    float                   wheel_speed_mps     = 0;
//    float                   wheel_speed_kmph    = 0;
//
//    uint32_t                counter             = 0;
//
//    chprintf( (BaseSequentialStream *)&SD7, "TEST ENCODER SPEED\n\r" );
//
//    while( 1 )
//    {
//        counter += 1;
//
//        enc_speed_tps       = getEncoderSpeedTPS( );
//        enc_speed_rps       = getEncoderSpeedRPS( );
//        wheel_speed_cmps    = getEncWheelSpeedCmPS( );
//        wheel_speed_mps     = getEncWheelSpeedMPS( );
//        wheel_speed_kmph    = getEncWheelSpeedKPH( );
//
//        if( counter == 20 )
//        {
//            chprintf( (BaseSequentialStream *)&SD7, "TPS:(%d)\tRPS:(%d)\tCMPS:(%d)MPS:(%d)\tKPS:(%d)\n\r\t",
//                      enc_speed_tps, enc_speed_rps, wheel_speed_cmps, (int)( wheel_speed_mps * 100 ), (int)( wheel_speed_kmph * 100 ) );
//
//            counter = 0;
//        }
//
//        chThdSleepMilliseconds( 10 );
//    }
}



void testOdometryRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX


    chprintf( (BaseSequentialStream *)&SD7, "TEST ODOMETRY\n\r" );

    float   tetta_deg       = 0;
    double  tetta_rad       = 0;

    double  test_x          = 0;
    double  test_y          = 0;

    uint32_t test_counter   = 0;

    while( 1 )
    {
        test_counter += 1;

        tetta_rad   = getObjTetaAngleRad( );
        tetta_deg   = getObjTettaAngleDeg( );
        test_x      = getObjPosX( );
        test_y      = getObjPosY( );

        if( test_counter == 30)
        {
          chprintf( (BaseSequentialStream *)&SD7, "T:(%d)[rad]\tT:(%d)[deg]\tX:(%d)[m]\tY:(%d)[m]\n\r",
                    (int)tetta_rad, (int)tetta_deg, (int)test_x, (int)test_y );

        }
        chThdSleepMilliseconds( 10 );
    }
}
