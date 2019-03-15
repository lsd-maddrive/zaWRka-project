#include <tests.h>
#include <lld_odometry.h>
#include <lld_steer_angle_fb.h>
#include <lld_control.h>


static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0, .cr2 = 0, .cr3 = 0
};


#define JUST_ODOMETRY


void testOdometryRoutine( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX

    lldOdometryInit( );

    odometryValue_t         test_distance   = 0;
    odometryRawSpeedValue_t test_speed_rps  = 0;
    odometrySpeedValue_t    test_speed_cmps = 0;
    odometrySpeedValue_t    test_speed_mps  = 0;

    odometryValue_t         test_tetta_rad  = 0;
    odometryValue_t         test_tetta_deg  = 0;
    odometryValue_t         test_x_pos      = 0;
    odometryValue_t         test_y_pos      = 0;

    steerAngleRawValue_t    test_raw_fb     = 0;
    steerAngleRadValue_t    test_rad_fb     = 0;
    steerAngleDegValue_t    test_deg_fb     = 0;


    chprintf( (BaseSequentialStream *)&SD7, "TEST ODOMETRY\n\r" );

    while( 1 )
    {
        test_distance   = lldGetOdometryObjDistance( OBJ_DIST_CM );
        test_speed_rps  = lldGetOdometryRawSpeedRPS( );
        test_speed_cmps = lldGetOdometryObjSpeedCMPS( );
        test_speed_mps  = lldGetOdometryObjSpeedMPS( );
        test_tetta_rad  = lldGetOdometryObjTettaRad( );
        test_tetta_deg  = lldGetOdometryObjTettaDeg( );
        test_x_pos      = lldGetOdometryObjX( OBJ_DIST_CM );
        test_y_pos      = lldGetOdometryObjY( OBJ_DIST_CM );

        test_raw_fb     = lldGetSteerAngleFiltrMeanRawADC( );
        test_rad_fb     = lldGetSteerAngleRad( );
        test_deg_fb     = lldGetSteerAngleDeg( );


#ifdef TOTAL_ODOMETRY
        chprintf( (BaseSequentialStream *)&SD7, "DIST:(%d)\tRPS:(%d)\tCMPS:(%d)\tMPS:(%d)\tT_R:(%d)\tT_D:(%d)\tX:(%d)\tY:(%d)\n\r",
                  (int)test_distance, (int)test_speed_rps, (int)test_speed_cmps, (int)test_speed_mps,
                  (int)test_tetta_rad, (int)test_tetta_deg,
                  (int)test_x_pos, (int)test_y_pos );
#endif

#ifdef SPEED_ODOMETRY
        chprintf( (BaseSequentialStream *)&SD7, "RPS:(%d)\tCMPS:(%d)\tMPS:(%d)\n\r",
                          (int)test_speed_rps, (int)test_speed_cmps, (int)(test_speed_mps * 10) );
#endif

#ifdef POS_ODOMETRY
        chprintf( (BaseSequentialStream *)&SD7, "DIST:(%d)\tMPS:(%d)\tT_R:(%d)\tX:(%d)\tY:(%d)\n\r",
                          (int)test_distance, (int)test_speed_mps*10, (int)(test_tetta_rad * 10),
                          (int)test_x_pos, (int)test_y_pos );
#endif

#ifdef TETTA_ODOMETRY
       chprintf( (BaseSequentialStream *)&SD7, "Tetta RAD:(%d)\tTetta DEG:(%d)\tSteer_RAD:(%d)\tSteer_DEG:(%d)\n\r",
                                  (int)(test_tetta_rad * 10), (int)(test_tetta_deg ), (int)(test_rad_fb * 10), (int)test_deg_fb);
#endif

#ifdef JUST_ODOMETRY
       chprintf( (BaseSequentialStream *)&SD7, "STEER_DEG:(%d)\tT_R:(%d)\tT_D:(%d)\tX:(%d)\tY:(%d)\n\r",
                         (int)test_deg_fb,
                         (int)test_tetta_rad, (int)test_tetta_deg,
                         (int)test_x_pos, (int)test_y_pos );
#endif

        chThdSleepMilliseconds( 100 );
    }
}

void testResetOdometryRoutine( void )
{
    lldOdometryInit( );
    lldControlInit( );
    debug_stream_init( );

    odometryValue_t         test_distance   = 0;
    odometryRawSpeedValue_t test_speed_rps  = 0;
    odometrySpeedValue_t    test_speed_cmps = 0;
    odometrySpeedValue_t    test_speed_mps  = 0;

    odometryValue_t         test_tetta_rad  = 0;
    odometryValue_t         test_tetta_deg  = 0;
    odometryValue_t         test_x_pos      = 0;
    odometryValue_t         test_y_pos      = 0;

    steerAngleRawValue_t    test_raw_fb     = 0;
    steerAngleRadValue_t    test_rad_fb     = 0;
    steerAngleDegValue_t    test_deg_fb     = 0;

    float                   test_speed      = 0;
    float                   d_speed         = 2; // %
    float                   test_steer      = 0;
    float                   d_steer         = 1; // %

    systime_t   time = chVTGetSystemTimeX( );

    while( 1 )
    {
      char rc_data  = sdGetTimeout( &SD3, TIME_IMMEDIATE );

      switch( rc_data )
      {
        case 'a':
          lldResetOdometry( );
          break;
        case 's':
          test_speed += d_speed;
          break;
        case 'd':
          test_speed -= d_speed;
          break;
        case 'z':
          test_steer += d_steer;
          break;
        case 'x':
          test_steer += d_steer;
          break;
        case ' ':
          test_speed = 0;
          test_steer = 0;
          break;

        default:
          ;
      }

      lldControlSetSteerMotorPower( test_steer );
      lldControlSetDrMotorPower( test_speed );


      test_distance   = lldGetOdometryObjDistance( OBJ_DIST_CM );
      test_speed_rps  = lldGetOdometryRawSpeedRPS( );
      test_speed_cmps = lldGetOdometryObjSpeedCMPS( );
      test_speed_mps  = lldGetOdometryObjSpeedMPS( );
      test_tetta_rad  = lldGetOdometryObjTettaRad( );
      test_tetta_deg  = lldGetOdometryObjTettaDeg( );
      test_x_pos      = lldGetOdometryObjX( OBJ_DIST_CM );
      test_y_pos      = lldGetOdometryObjY( OBJ_DIST_CM );

      test_raw_fb     = lldGetSteerAngleFiltrMeanRawADC( );
      test_rad_fb     = lldGetSteerAngleRad( );
      test_deg_fb     = lldGetSteerAngleDeg( );

      dbgprintf( "DIST:(%d)\tRPS:(%d)\tCMPS:(%d)\tMPS:(%d)\tT_R:(%d)\tT_D:(%d)\tX:(%d)\tY:(%d)\n\r",
              (int)test_distance, (int)test_speed_rps, (int)test_speed_cmps, (int)test_speed_mps,
              (int)test_tetta_rad, (int)test_tetta_deg,
              (int)test_x_pos, (int)test_y_pos );



        time = chThdSleepUntilWindowed( time, time + MS2ST( 200 ) );
    }


}
