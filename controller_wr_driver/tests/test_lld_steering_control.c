#include <tests.h>
#include <lld_steering_control.h>


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
   sd_set();
   lldSteeringControlInit();
   /* Start working GPT driver in asynchronous mode */
   gptStart(Tim3, &gpt3cfg);
   gptStartContinuous(Tim3, period_50ms);

   int16_t deg_steer_angle = 0;

   double rad_steer_angle = 0;

   double test_minus    = 0;
   double test_plus     = 0;

   test_minus   = tan(-M_PI);
   test_plus    = tan(1.5 * M_PI);
    while( true )
    {
    	last_periodCheckPoint = gptGetCounterX(Tim3);
    	//AdcVal = lldSteeringControlGetAdcVal();
    	AdcVal = lldSteeringControlGetAdcVal_Kalman ();
    	periodCheckPoint = gptGetCounterX(Tim3);
    	KalmanTime = total_ticks + periodCheckPoint - last_periodCheckPoint;
    	total_ticks = 0;
        //sdWrite( &SD7, (uint16_t *)&AdcVal, sizeof( AdcVal ) );
    	deg_steer_angle = lldGetSteerDegAngle( );
    	rad_steer_angle = deg_steer_angle * M_PI / 180;
    	chprintf( (BaseSequentialStream *)&SD7, "ADC:(%d)\tDEG:(%d)\tRAD:(%d)\n\r",
    	          AdcVal, deg_steer_angle, (int)(rad_steer_angle * 10) );
//    	chprintf( (BaseSequentialStream *)&SD7, "Tg(-pi):(%d)\tTg(1.5pi):(%d)\n\r",
//    	                  (int)(test_minus*10),(int)(test_plus*10) );

    	chThdSleepMilliseconds( 200 );
    }
}



