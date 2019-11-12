#include <tests.h>
#include <lld_odometry.h>
#include <lld_steer_angle_fb.h>
#include <lld_control.h>
#include <remote_control.h>
#include <drive_cs.h>

#define JUST_ODOMETRY

/*
 * @brief   Test Odometry Routine
 * @note    There are 5 options:
 *          * TOTAL_ODOMETRY (
 *          * SPEED_ODOMETRY
 *          * POS_ODOMETRY
 *          * TETTA_ODOMETRY
 *          * JUST_ODOMETRY
*/
void testOdometryRoutine( void )
{
      lldOdometryInit( );
      debug_stream_init( );
#ifdef TOTAL_ODOMETRY
      odometryValue_t         test_distance   = 0;
      odometryRawSpeedValue_t test_speed_rps  = 0;
      odometrySpeedValue_t    test_speed_cmps = 0;
      odometrySpeedValue_t    test_speed_mps  = 0;
      odometryValue_t         test_tetta_rad  = 0;
      odometryValue_t         test_tetta_deg  = 0;
      odometryValue_t         test_x_pos      = 0;
      odometryValue_t         test_y_pos      = 0;
#endif
#ifdef SPEED_ODOMETRY
      odometryRawSpeedValue_t test_speed_rps  = 0;
      odometrySpeedValue_t    test_speed_cmps = 0;
      odometrySpeedValue_t    test_speed_mps  = 0;
#endif
#ifdef POS_ODOMETRY
      odometryValue_t         test_distance   = 0;
      odometrySpeedValue_t    test_speed_mps  = 0;
      odometryValue_t         test_tetta_rad  = 0;
      odometryValue_t         test_x_pos      = 0;
      odometryValue_t         test_y_pos      = 0;
#endif
#ifdef TETTA_ODOMETRY
      odometryValue_t         test_tetta_rad  = 0;
      odometryValue_t         test_tetta_deg  = 0;
      steerAngleRadValue_t    test_rad_fb     = 0;
      steerAngleDegValue_t    test_deg_fb     = 0;
#endif
#ifdef JUST_ODOMETRY
      steerAngleDegValue_t    test_deg_fb     = 0;
      odometryValue_t         test_tetta_rad  = 0;
      odometryValue_t         test_tetta_deg  = 0;
      odometryValue_t         test_x_pos      = 0;
      odometryValue_t         test_y_pos      = 0;
#endif
    systime_t   time = chVTGetSystemTimeX( );
    while( 1 )
    {
#ifdef TOTAL_ODOMETRY
        test_distance   = lldGetOdometryObjDistance( OBJ_DIST_CM );
        test_speed_rps  = lldGetOdometryRawSpeedRPS( );
        test_speed_cmps = lldGetOdometryObjSpeedCMPS( );
        test_speed_mps  = lldGetOdometryObjSpeedMPS( );
        test_tetta_rad  = lldGetOdometryObjTettaRad( );
        test_tetta_deg  = lldGetOdometryObjTettaDeg( );
        test_x_pos      = lldGetOdometryObjX( OBJ_DIST_CM );
        test_y_pos      = lldGetOdometryObjY( OBJ_DIST_CM );
#endif
#ifdef  SPEED_ODOMETRY
        test_speed_rps  = lldGetOdometryRawSpeedRPS( );
        test_speed_cmps = lldGetOdometryObjSpeedCMPS( );
        test_speed_mps  = lldGetOdometryObjSpeedMPS( );
#endif
#ifdef  POS_ODOMETRY
        test_distance   = lldGetOdometryObjDistance( OBJ_DIST_CM );
        test_speed_mps  = lldGetOdometryObjSpeedMPS( );
        test_tetta_rad  = lldGetOdometryObjTettaRad( );
        test_x_pos      = lldGetOdometryObjX( OBJ_DIST_CM );
        test_y_pos      = lldGetOdometryObjY( OBJ_DIST_CM );
#endif
#ifdef  TETTA_ODOMETRY
        test_tetta_rad  = lldGetOdometryObjTettaRad( );
        test_tetta_deg  = lldGetOdometryObjTettaDeg( );
        test_rad_fb     = lldGetSteerAngleRad( );
        test_deg_fb     = lldGetSteerAngleDeg( );
#endif
#ifdef JUST_ODOMETRY
        test_deg_fb     = lldGetSteerAngleDeg( );
        test_tetta_rad  = lldGetOdometryObjTettaRad( );
        test_tetta_deg  = lldGetOdometryObjTettaDeg( );
        test_x_pos      = lldGetOdometryObjX( OBJ_DIST_CM );
        test_y_pos      = lldGetOdometryObjY( OBJ_DIST_CM );
#endif

#ifdef TOTAL_ODOMETRY
        dbgprintf( "DIST:(%d)\tRPS:(%d)\tCMPS:(%d)\tMPS:(%d)\tT_R:(%d)\tT_D:(%d)\tX:(%d)\tY:(%d)\n\r",
                  (int)test_distance, (int)test_speed_rps, (int)test_speed_cmps,
                  (int)test_speed_mps,
                  (int)test_tetta_rad, (int)test_tetta_deg,
                  (int)test_x_pos, (int)test_y_pos );
#endif

#ifdef SPEED_ODOMETRY
        dbgprintf( "RPS:(%d)\tCMPS:(%d)\tMPS:(%d)\n\r",
                      (int)test_speed_rps, (int)test_speed_cmps,
                      (int)(test_speed_mps * 10) );
#endif

#ifdef POS_ODOMETRY
        dbgprintf( "DIST:(%d)\tMPS:(%d)\tT_R:(%d)\tX:(%d)\tY:(%d)\n\r",
                      (int)test_distance, (int)test_speed_mps*10, (int)(test_tetta_rad * 10),
                      (int)test_x_pos, (int)test_y_pos );
#endif

#ifdef TETTA_ODOMETRY
        dbgprintf( "Tetta RAD:(%d)\tTetta DEG:(%d)\tSteer_RAD:(%d)\tSteer_DEG:(%d)\n\r",
                      (int)(test_tetta_rad * 10), (int)(test_tetta_deg ),
                      (int)(test_rad_fb * 10), (int)test_deg_fb );
#endif

#ifdef JUST_ODOMETRY
       dbgprintf( "STEER_DEG:(%d)\tT_R:(%d)\tT_D:(%d)\tX:(%d)\tY:(%d)\n\r",
                     (int)test_deg_fb,
                     (int)test_tetta_rad, (int)test_tetta_deg,
                     (int)test_x_pos, (int)test_y_pos );
#endif

        time = chThdSleepUntilWindowed( time, time + MS2ST( 100 ) );
    }
}

/*
 * @brief   Test for odometry reset
*/
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

      dbgprintf( "DIST:(%d)\tRPS:(%d)\tCMPS:(%d)\tMPS:(%d)\tT_R:(%d)\tT_D:(%d)\tX:(%d)\tY:(%d)\n\r",
              (int)test_distance, (int)test_speed_rps, (int)test_speed_cmps, (int)test_speed_mps,
              (int)test_tetta_rad, (int)test_tetta_deg,
              (int)test_x_pos, (int)test_y_pos );

        time = chThdSleepUntilWindowed( time, time + MS2ST( 100 ) );
    }
}

/*
 * @brief   Test for odometry (x, y, tetta values)
 * @note    Control via RC 
*/
void testRCOdodmetry( void )
{
  lldOdometryInit( );
  lldControlInit( );
  debug_stream_init( );
  remoteControlInit( NORMALPRIO );


  icuControlValue_t   rc_steer_prt    = 0;
  icuControlValue_t   rc_speed_prt    = 0;
  bool                mode            = false;

  odometryValue_t     test_tetta_deg  = 0;
  odometryValue_t     test_x_pos      = 0;
  odometryValue_t     test_y_pos      = 0;

  uint32_t            show_counter    = 0;

  systime_t time = chVTGetSystemTimeX();

  while( 1 )
  {
    show_counter += 1;
    mode = rcModeIsEnabled();

    if( mode == true )
    {
      rc_steer_prt    = rcGetSteerControlValue( );
      rc_speed_prt    = rcGetSpeedControlValue( );

      lldControlSetDrMotorPower( rc_speed_prt );
      lldControlSetSteerMotorPower( rc_steer_prt );
    }
    else
    {
      lldControlSetSteerMotorRawPower( lldControlGetSteerMotorZeroPower() );
      lldControlSetDrMotorRawPower( lldControlGetDrMotorZeroPower() );
    }

    test_tetta_deg  = lldGetOdometryObjTettaDeg( );
    test_x_pos      = lldGetOdometryObjX( OBJ_DIST_CM );
    test_y_pos      = lldGetOdometryObjY( OBJ_DIST_CM );

    if( show_counter == 20 )
    {
      dbgprintf( "X:(%d)\tY:(%d)\tT:(%d)\n\r",
                  (int)test_x_pos, (int)test_y_pos,
                  (int)test_tetta_deg);

      show_counter = 0;
    } 

    time = chThdSleepUntilWindowed( time, time + MS2ST( 10 ) );
  }
}

/*
 * @brief   Test odometry
 * @note    Reference value of x-distance = 20 cm 
 *          Speed is constant (0.1 m/s)
*/
void testXdistanceOdometry( void )
{
  lldOdometryInit( );
  debug_stream_init( );
  driverCSInit( NORMALPRIO );
  driverIsEnableCS(true);

  odometryValue_t     test_x_pos          = 0;

  uint32_t            show_counter    = 0;

  systime_t time = chVTGetSystemTimeX();

  while( 1 )
  {
    show_counter += 1;
    driveSteerCSSetPosition( 0 );
    test_x_pos      = lldGetOdometryObjX( OBJ_DIST_CM );

    if(test_x_pos >= 20)
    {
      driveSpeedCSSetSpeed( 0 );
    }
    else
    {
      driveSpeedCSSetSpeed( 0.1 );
    }


    if( show_counter == 20 )
    {
      dbgprintf( "X:(%d)\n\r", (int)test_x_pos );

      show_counter = 0;
    } 

    time = chThdSleepUntilWindowed( time, time + MS2ST( 10 ) );
  }
}