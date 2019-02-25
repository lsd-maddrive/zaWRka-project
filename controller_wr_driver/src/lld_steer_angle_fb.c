#include <tests.h>
#include <lld_steer_angle_fb.h>

/***********************************/
/***    FILTER CONFIGURATION     ***/
/***********************************/

#define STEER_FILTER_MEAN       0

#define STEER_ACTIVE_FILTER     STEER_FILTER_MEAN


/***********************************/
/***    OBJECT CONFIGURATION     ***/
/***********************************/

/*
 * R1_left  = 56.5 cm => tg = L / R1 = 0.53097 => 28 deg
 * R1_right = 44   cm => tg = L / R1 = 0.6818  => 34 deg
 * */

#define STEER_MAX_LEFT_ANGLE_DEG    28
#define STEER_MAX_RIGHT_ANGLE_DEG   34


/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/

/***    ADC CONFIG    ***/

/***     Timer 4 = Trigger   ***/
#define ADC_MODE_TRIGGER        ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(12)
/***     12-BIT RESOLUTION   ***/
#define ADC_12_BIT_CONF         (0)
#define ADC_10_BIT_CONF         (ADC_CR1_RES_0)

#define ADC_1_NUM_CHANNELS      1
#define ADC_1_BUF_DEPTH         1

#define ADC_STEER_ANGL_LINE     PAL_LINE( GPIOC, 0 )



static ADCDriver *steerADCDriver = &ADCD1;

steerAngleRawValue_t            steerAngleRawADCValue   = 0;

static adcsample_t          adc_1_buffer[ADC_1_NUM_CHANNELS * ADC_1_BUF_DEPTH];

/***    FILTER CONFIG    ***/

#define STEER_WINDOW_VAL    15

static  uint32_t adc_cb_counter = 0;

steerAngleRawValue_t    steer_mean_sum              = 0;
steerAngleRawValue_t    steer_mean_filter_adc_val   = 0;

static void adc_1_cb ( ADCDriver *adcp, adcsample_t *buffer, size_t n )
{
    adcp = adcp;
    n = n;

    adc_cb_counter += 1;

    steerAngleRawADCValue = adc_1_buffer[0];
    steer_mean_sum += steerAngleRawADCValue;

    if( adc_cb_counter == STEER_WINDOW_VAL )
    {
        steer_mean_filter_adc_val = steer_mean_sum / STEER_WINDOW_VAL;
        adc_cb_counter      = 0;
        steer_mean_sum       = 0;
    }
}

static const ADCConversionGroup steer_adc_1_cnfg = {
  .circular     = true,
  .num_channels = ADC_1_NUM_CHANNELS,
  .end_cb       = adc_1_cb,
  .error_cb     = 0,
  .cr1          = ADC_12_BIT_CONF,
  .cr2          = ADC_MODE_TRIGGER,
  .smpr1        = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_144),
  .smpr2        = 0,
  .sqr1         = ADC_SQR1_NUM_CH(ADC_1_NUM_CHANNELS),
  .sqr2         = 0,
  .sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)
};

/***    TIMER CONFIG    ***/

static GPTDriver            *adcTrgDriver   = &GPTD4;

static const GPTConfig gpt_trg_cnfg = {
  .frequency =  100000,
  .callback  =  NULL,
  .cr2       =  TIM_CR2_MMS_1,
  .dier      =  0U
 };


#define     STEER_ADC_RIGHT         1075
#define     STEER_ADC_LEFT          1980
#define     STEER_ADC_CENTER        1510


#define     STEER_RAD_RIGHT         (float)(-0.593) // -34
#define     STEER_RAD_LEFT          (float)0.52  // 28
#define     STEER_RAD_CENTER        (float)0.0



static bool         isInitialized       = false;

float       steer_right_k   = 0;
float       steer_right_b   = 0;

float       steer_left_k    = 0;
float       steer_left_b    = 0;

float       rad_2_deg       = 0;


/**
 * @brief       Initialization of unit for steering angle feedback
 * @note        ADC initialization and GPT initialization
*/
void lldSteerAngleFBInit ( void )
{
    if ( isInitialized )
            return;

    /***    ADC start       ***/
    adcStart( steerADCDriver, NULL );
    palSetLineMode( ADC_STEER_ANGL_LINE, PAL_MODE_INPUT_ANALOG );
    adcStartConversion( steerADCDriver, &steer_adc_1_cnfg, adc_1_buffer, ADC_1_BUF_DEPTH );

    /***    TIMER start     ***/
    gptStart(adcTrgDriver, &gpt_trg_cnfg);
    /***    Trigger = 1 ms ***/
    gptStartContinuous( adcTrgDriver, gpt_trg_cnfg.frequency/100 );

    /***    Coefficient calculation  ***/
    steer_right_k = (STEER_RAD_CENTER - STEER_RAD_RIGHT) / (STEER_ADC_CENTER - STEER_ADC_RIGHT);
    steer_right_b = STEER_RAD_RIGHT - STEER_ADC_RIGHT * steer_right_k;

    steer_left_k  = (STEER_RAD_CENTER - STEER_RAD_LEFT) / (STEER_ADC_CENTER - STEER_ADC_LEFT);
    steer_left_b  = STEER_RAD_LEFT - STEER_ADC_LEFT * steer_left_k;

    rad_2_deg     = 180 / M_PI;
    /* Set initialization flag */

    isInitialized = true;
}

/**
 * @brief       Get raw value of steering angle
 * @return      ADC value [0; 4096]
*/
steerAngleRawValue_t lldGetRawADCSteerAngleFB (void)
{
    return steerAngleRawADCValue;
}


/**
 * @brief       Get raw filtered value of steering angle
 * @return      ADC value [0; 4096]
 * @note        Mean filter
*/
steerAngleRawValue_t lldGetFiltrMeanRawADCSteerAngleFB ( void )
{
    return steer_mean_filter_adc_val;
}


/**
 * @brief       Get steering angle [rad]
 * @return      max_right   ->  STEER_RAD_RIGHT
 *              center      ->  STEER_RAD_CENTER
 *              max_left    ->  STEER_RAD_LEFT
*/
steerAngleRadValue_t lldGetRadSteerAngleFB ( steerAngleRawValue_t steer_adc_val )
{
    steerAngleRadValue_t    steer_rad_angl  = 0;

    if( steer_adc_val < STEER_ADC_RIGHT && steer_adc_val > STEER_ADC_LEFT )
        steer_rad_angl = 0;
    else if( steer_adc_val < STEER_ADC_CENTER && steer_adc_val >= STEER_ADC_RIGHT ) // right
        steer_rad_angl = steer_adc_val * steer_right_k + steer_right_b;
    else if( steer_adc_val > STEER_ADC_CENTER && steer_adc_val <= STEER_ADC_LEFT )  // left
        steer_rad_angl = steer_adc_val * steer_left_k + steer_left_b;

    return steer_rad_angl;
}

/**
 * @brief       Get steering angle [deg]
 * @return      max_right   ->  -34
 *              center      ->  0
 *              max_left    ->  28
 *@note         IMPORTANT!
 *              Use AFTER Initialization
*/
steerAngleDegValue_t lldGetDegSteerAngleFB( steerAngleRadValue_t rad_angle )
{

    return ( rad_angle * rad_2_deg );
}

/***    NOT FIXED   ***/


/*
   delta for left direction:      1780  < ADC_val < 1850 ( 1790 < ADC_val < 1860)
   delta for right direction:      970  < ADC_val < 1135 ( 905  < ADC_val < 960 )
   delta for central direction:   1475  < ADC_val < 1540 ( 1510 < ADC_val < 1570)
*/


static int32_t  servLimValue        = 10; // permissible error after reading from ADC
static int32_t  servMAX             = 1820;
static int32_t  servMIN             = 1000;
static int32_t  servMID             = 1520;

static int32_t  deltaServMAX        = 300;
static int32_t  deltaServMIN        = 520;

/*** Variable configuration ***/

#define servADCchannel       ADC_SEQ1_CH

static uint8_t  filter_cnt                 = 0;
int32_t ADCfilter[] = {0,0,0,0,0};

static uint8_t  filter_cnt2                 = 0;
int32_t ADCfilter2[] = {0,0,0,0,0};

static bool     firstKalmVal                   = false;
int16_t KalmAdcVal        = 0;
int16_t lastKalmAdcVal    = 0;

/* ADC value */

static float leftMaxAngle                  = 34.56;
static float rightMaxAngle                 = 29.98;

static float leftFrontPosAngle             = 0;
static float rightFrontPosAngle            = 0;



void lldSteeringControlInit  (void)
{
	lldSteerAngleFBInit();

   leftFrontPosAngle    = leftMaxAngle / deltaServMAX;
   rightFrontPosAngle   = rightMaxAngle / deltaServMIN;

   isInitialized = true;
}

int16_t lldSteeringControlGetAdcVal (void)
{
    if ( !isInitialized )
	    return false;
	return lldGetRawADCSteerAngleFB();
}


int16_t lldSteeringControlGetAdcVal_Kalman (void)
{
    if ( !isInitialized )
	    return false;

    int16_t KalmCoef          = 0.05;

    if (firstKalmVal == false)
    {
    	KalmAdcVal = lldGetRawADCSteerAngleFB();
    	firstKalmVal = true;
    }
    else
    {
    	KalmAdcVal = KalmCoef * KalmAdcVal + lastKalmAdcVal * (1 - KalmCoef);
    }
    lastKalmAdcVal = KalmAdcVal;

	return KalmAdcVal;
}


int16_t lldSteeringControGetAdcPos_filt (void)
{
  if ( !isInitialized )
      return false;

  int16_t ADC_val = 0;
  int16_t Sum = 0;
  uint8_t i = 0;
  uint8_t j = 0;

  if (filter_cnt < 5)
  {
	  ADCfilter[filter_cnt] = lldGetRawADCSteerAngleFB();
	  ADC_val = ADCfilter[filter_cnt];
	  filter_cnt++;
  }
  else
  {
	  while (i < 5)
	  {
		  Sum = Sum + ADCfilter[i];
		  i++;
	  }

	  while (j < 4 )
	  {
		  ADCfilter[j] = ADCfilter[j + 1];
		  j++;
	  }
	  ADCfilter[4] = lldGetRawADCSteerAngleFB();
	  ADC_val = Sum / 5;
  }
  return ADC_val;
}

int16_t lldSteeringControGetAdcPos_doublefilt (void)
{
  if ( !isInitialized )
      return false;

  int16_t ADC_filtered_val = 0;
  int16_t Sum = 0;
  uint8_t i = 0;
  uint8_t j = 0;

  if (filter_cnt2 < 5)
  {
	  ADCfilter2[filter_cnt2] = lldSteeringControGetAdcPos_filt();
	  ADC_filtered_val = ADCfilter2[filter_cnt2];
	  filter_cnt2++;
  }
  else
  {
	  while (i < 5)
	  {
		  Sum = Sum + ADCfilter2[i];
		  i++;
	  }

	  while (j < 4 )
	  {
		  ADCfilter2[j] = ADCfilter2[j + 1];
		  j++;
	  }
	  ADCfilter2[14] = lldSteeringControGetAdcPos_filt();
	  ADC_filtered_val = Sum / 5;
  }
  return ADC_filtered_val;
}



degSteerAngleValue_t lldGetSteerDegAngle (void)
{
  if ( !isInitialized )
      return false;

  int16_t RotateAngle  = 0;
  int16_t lldAdcVal = lldGetRawADCSteerAngleFB();

    if (lldAdcVal < servMIN - servLimValue)
  {
      lldAdcVal = servMIN - servLimValue ;
  }
    else if (lldAdcVal > servMAX + servLimValue)
  {
      lldAdcVal = servMAX + servLimValue;
  }
  if (lldAdcVal - servMID > 0)
  {
      RotateAngle = (lldAdcVal - servMID) * leftFrontPosAngle;
  }
  else
  {
      RotateAngle = (lldAdcVal - servMID) * rightFrontPosAngle;
  }
  return RotateAngle;
}

