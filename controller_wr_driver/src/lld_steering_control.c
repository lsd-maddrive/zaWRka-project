#include <tests.h>
#include <lld_steering_control.h>



/**************************/
/*** CONFIGURATION ZONE ***/
/**************************/


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




/******************************/
/*** CONFIGURATION ZONE END ***/
/******************************/

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

static bool     isInitialized              = false;

static float leftMaxAngle                  = 34.56;
static float rightMaxAngle                 = 29.98;

static float leftFrontPosAngle             = 0;
static float rightFrontPosAngle            = 0;

///////////////////////ADC1 init///////////////////////////

#define ADC_MODE_TRIGGER                ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(12)
#define ADC_MODE_MANUAL                 ADC_CR2_SWSTART

#define ADC1_NUM_CHANNELS 1
#define ADC1_BUF_DEPTH    1

int16_t AdcBuf        = 0;

static adcsample_t adc_buffer[ADC1_NUM_CHANNELS * ADC1_BUF_DEPTH];

static void adccallback ( ADCDriver *adcp, adcsample_t *buffer, size_t n );

static const ADCConversionGroup cfg_grp1 = {
  .circular     = true,
  .num_channels = ADC1_NUM_CHANNELS,
  .end_cb       = adccallback,
  .error_cb     = 0,
  .cr1          = ADC_RES_CONF,
  .cr2          = ADC_MODE_TRIGGER,
  .smpr1        = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_144),
  .smpr2        = 0,
  .sqr1         = ADC_SQR1_NUM_CH(ADC1_NUM_CHANNELS),
  .sqr2         = 0,
  .sqr3         = ADC_SQR3_SQ1_N(ADC_SEQ1_NUM)
};

static const GPTConfig gpt_loop = {
  .frequency =  100000,
  .callback  =  NULL,
  .cr2       =  TIM_CR2_MMS_1,
  .dier      =  0U
 };

static ADCDriver                *globalADCDriver    = &ADCD1;
static GPTDriver                *adcTriggerDriver   = &GPTD4;

static void adccallback ( ADCDriver *adcp, adcsample_t *buffer, size_t n )
{
    adcp = adcp;
    n = n;
    AdcBuf = buffer[0];
    if ( buffer != adc_buffer )
    {

    }
}

static bool     adc_Init      = false;

void InitAdc ( void )
{
    if ( adc_Init )
    {
        return;
    }
    adcStart( globalADCDriver, NULL );

    palSetLineMode( LINE_ADC123_IN10, PAL_MODE_INPUT_ANALOG );

    adcStartConversion( globalADCDriver, &cfg_grp1, adc_buffer, ADC1_BUF_DEPTH );

    gptStart(adcTriggerDriver, &gpt_loop);
    /*1 ms, triggering 1000 times per sec */
    gptStartContinuous( adcTriggerDriver, gpt_loop.frequency/1000 );

    adc_Init = true;
}


adcsample_t GetAdcVal (void)
{
    return AdcBuf;
}


void lldSteeringControlInit  (void)
{
	InitAdc();

   leftFrontPosAngle    = leftMaxAngle / deltaServMAX;
   rightFrontPosAngle   = rightMaxAngle / deltaServMIN;
    
   isInitialized = true; 
}

int16_t lldSteeringControlGetAdcVal (void)
{
    if ( !isInitialized )
	    return false;
	return GetAdcVal();
}


int16_t lldSteeringControlGetAdcVal_Kalman (void)
{
    if ( !isInitialized )
	    return false;

    int16_t KalmCoef          = 0.05;

    if (firstKalmVal == false)
    {
    	KalmAdcVal = GetAdcVal();
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
	  ADCfilter[filter_cnt] = GetAdcVal();
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
	  ADCfilter[4] = GetAdcVal();
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
  int16_t lldAdcVal = GetAdcVal();
     
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

