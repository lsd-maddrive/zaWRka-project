#include <tests.h>
#include <lld_steer_angle_fb.h>


#include <lld_control.h>

int32_t AdcVal = 0;
int16_t PosVal = 0;
int16_t Angle  = 0;

static const SerialConfig sdcfg = {
  .speed = 115200,
  .cr1 = 0,
  .cr2 = 0,
  .cr3 = 0
};

/////////////////////////////////////////////////////////////////////
static void gpt_callback ( GPTDriver *Tim3 );
static GPTDriver                     *Tim3 = &GPTD3;

static const GPTConfig gpt3cfg = {
  .frequency =  100000,
  .callback  =  gpt_callback,
  .cr2       =  0,
  .dier      =  0U
};

#define period_20ms         gpt3cfg.frequency/50
#define period_100ms        gpt3cfg.frequency/10
#define period_50ms         gpt3cfg.frequency/20

int32_t total_ticks                       = 0;
int32_t KalmanTime                        = 0;
int32_t periodCheckPoint                  = 0;
int32_t last_periodCheckPoint             = 0;

static void gpt_callback (GPTDriver *gptd)
{
    gptd = gptd;
    total_ticks += period_50ms;
}

/////////////////////////////////////////////////////////////////////

void sd_set(void)
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX
}

void testSteeringControl (void)
{
//   sd_set();
//   lldSteeringControlInit();
//   /* Start working GPT driver in asynchronous mode */
//   gptStart(Tim3, &gpt3cfg);
//   gptStartContinuous(Tim3, period_50ms);
//
//   int16_t deg_steer_angle = 0;
//
//   double rad_steer_angle = 0;
//
//   double test_minus    = 0;
//   double test_plus     = 0;
//
//   test_minus   = tan(-M_PI);
//   test_plus    = tan(1.5 * M_PI);
//    while( true )
//    {
//    	last_periodCheckPoint = gptGetCounterX(Tim3);
//    	//AdcVal = lldSteeringControlGetAdcVal();
//    	AdcVal = lldSteeringControlGetAdcVal_Kalman ();
//    	periodCheckPoint = gptGetCounterX(Tim3);
//    	KalmanTime = total_ticks + periodCheckPoint - last_periodCheckPoint;
//    	total_ticks = 0;
//        //sdWrite( &SD7, (uint16_t *)&AdcVal, sizeof( AdcVal ) );
//    	deg_steer_angle = lldGetSteerDegAngle( );
//    	rad_steer_angle = deg_steer_angle * M_PI / 180;
//    	chprintf( (BaseSequentialStream *)&SD7, "ADC:(%d)\tDEG:(%d)\tRAD:(%d)\n\r",
//    	          AdcVal, deg_steer_angle, (int)(rad_steer_angle * 10) );
////    	chprintf( (BaseSequentialStream *)&SD7, "Tg(-pi):(%d)\tTg(1.5pi):(%d)\n\r",
////    	                  (int)(test_minus*10),(int)(test_plus*10) );
//
//    	chThdSleepMilliseconds( 200 );
//    }
}

#define STEER_FB_TERMINAL


void testSteerAngleSendData( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX

    lldSteerAngleFBInit( );
    lldControlInit( );

    steerAngleRawValue_t    test_raw_steer          = 0;
    steerAngleRawValue_t    test_mean_raw_steer     = 0;
    controlValue_t          test_steer_cntl         = 0;
    steerAngleRadValue_t    test_rad_angle          = 0;
    steerAngleDegValue_t    test_deg_angle          = 0;
//    controlValue_t          test_delta_steer_cntr   = 0;

#ifdef STEER_FB_MATLAB
    char                    steer_matlab_start  = 0;
    uint8_t                 steer_start_flag    = 0;
#endif

    while( true )
    {

        test_raw_steer      = lldGetRawADCSteerAngleFB( );
        test_mean_raw_steer = lldGetFiltrMeanRawADCSteerAngleFB( );
        test_rad_angle      = lldGetRadSteerAngleFB( test_mean_raw_steer );
        test_deg_angle      = lldGetDegSteerAngleFB( test_rad_angle );

#ifdef STEER_FB_TERMINAL
        char rc_data = sdGetTimeout( &SD7, TIME_IMMEDIATE );

        switch( rc_data )
        {
          case 'a':     // turn right
            lldControlSetSteerMotorPower( -100 );
            break;
          case 's':     // center
            lldControlSetSteerMotorPower( 0 );
            break;
          case 'd':     // turn left
            lldControlSetSteerMotorPower( 100 );
            break;
          default:
            ;
        }

        chprintf( (BaseSequentialStream *)&SD7, "ADC_RAW:(%d)\tMEAN_RAW:(%d)\tRAD:(%d)\tDEG:(%d)\n\r",
                  test_raw_steer, test_mean_raw_steer, (int)(test_rad_angle * 10), (int)test_deg_angle );

        chThdSleepMilliseconds( 300 );
#endif

#ifdef STEER_FB_MATLAB
        steer_matlab_start = sdGetTimeout( &SD7, TIME_IMMEDIATE );

        if( steer_matlab_start == 's' ) steer_start_flag = 1;

        if( steer_start_flag == 1)
        {
            palToggleLine( LINE_LED3 );
            sdWrite(&SD7, &test_raw_steer, 2);
            sdWrite(&SD7, &test_mean_raw_steer, 2);
        }

        chThdSleepMilliseconds( 10 );
#endif


    }


}

/*
 * Test for steering angle calculation
 * */
void testSteerAngleDetection( void )
{
    sdStart( &SD7, &sdcfg );
    palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );    // TX
    palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );    // RX

    lldControlInit( );

    while(1)
    {
        char rc_data = sdGet( &SD7 );
        switch( rc_data )
        {
             case 'a':      // turn right
               lldControlSetSteerMotorPower( -100 );
               break;
             case 'q':
               lldControlSetSteerMotorPower( 0 );
               break;
             case 'z':      // turn left
               lldControlSetSteerMotorPower( 100 );
               break;
        }

        chThdSleepMilliseconds( 200 );
    }


}

