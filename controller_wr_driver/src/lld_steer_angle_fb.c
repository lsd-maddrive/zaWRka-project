#include <tests.h>
#include <lld_steer_angle_fb.h>

/***********************************/
/***    FILTER CONFIGURATION     ***/
/***********************************/

#define STEER_FILTER_MEAN       0
#define STEER_FILTER_LPF        1

#define STEER_ACTIVE_FILTER     STEER_FILTER_LPF

/***********************************/
/***    OBJECT CONFIGURATION     ***/
/***********************************/

#define     STEER_ADC_RIGHT         1075
#define     STEER_ADC_LEFT          1980
#define     STEER_ADC_CENTER        1510

#define     STEER_RAD_RIGHT         (float)(-0.593) // -34
#define     STEER_RAD_LEFT          (float)0.52     // 29
#define     STEER_RAD_CENTER        (float)0.0

/*
 * R1_left  = 56.5 cm => tg = L / R1 = 0.53097 => 28 deg
 * R1_right = 44   cm => tg = L / R1 = 0.6818  => 34 deg
 */

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

static ADCDriver                *steerADCDriver         = &ADCD1;

steerAngleRawValue_t            steer_raw_ADC_val   = 0;

static adcsample_t              adc_1_buffer[ADC_1_NUM_CHANNELS * ADC_1_BUF_DEPTH];

/***************************/
/***    FILTER CONFIG    ***/
/***************************/

steerAngleRawValue_t    steer_filtered_adc_val      = 0;

/***        MEAN Filter         ***/
#define STEER_MEAN_WINDOW_VAL    15

static  uint32_t        adc_cb_counter              = 0;

steerAngleRawValue_t    steer_mean_sum              = 0;

/***    Low Pass Filter ( LPF ) ***/

#define STEER_LPF_FACTOR         (float)0.9

steerAngleRawValue_t    prev_steer_lpf_adc_val      = 0;

static void adc_1_cb ( ADCDriver *adcp, adcsample_t *buffer, size_t n )
{
    adcp = adcp;
    n = n;

    adc_cb_counter += 1;

    steer_raw_ADC_val = adc_1_buffer[0];

#if( STEER_ACTIVE_FILTER == STEER_FILTER_MEAN )
    steer_mean_sum += steer_raw_ADC_val;

    if( adc_cb_counter == STEER_MEAN_WINDOW_VAL )
    {
        steer_filtered_adc_val = steer_mean_sum / STEER_MEAN_WINDOW_VAL;
        adc_cb_counter      = 0;
        steer_mean_sum      = 0;
    }
#elif( STEER_ACTIVE_FILTER == STEER_FILTER_LPF )
    steer_filtered_adc_val = steer_raw_ADC_val * (1 - STEER_LPF_FACTOR) + prev_steer_lpf_adc_val * STEER_LPF_FACTOR;

    prev_steer_lpf_adc_val = steer_filtered_adc_val;
    adc_cb_counter      = 0;
#else
    adc_cb_counter      = 0;
#endif
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

/***    COEFFICIENT INIT    ***/

static bool     isInitialized       = false;

float           steer_right_k   = 0;
float           steer_right_b   = 0;

float           steer_left_k    = 0;
float           steer_left_b    = 0;

float           rad_2_deg       = 0;

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
steerAngleRawValue_t lldGetSteerAngleRawADC (void)
{
    return steer_raw_ADC_val;
}

/**
 * @brief       Get raw filtered value of steering angle
 * @return      ADC value [0; 4096]
 * @note        Mean filter
 *              NEED TO SET
 *              STEER_ACTIVE_FILTER = STEER_FILTER_MEAN
*/
steerAngleRawValue_t lldGetSteerAngleFiltrMeanRawADC ( void )
{
    return steer_filtered_adc_val;
}

/**
 * @brief       Get raw filtered value of steering angle
 * @return      ADC value [0; 4096]
 * @note        Low pass Filter ( LPF )
 *              NEED TO SET
 *              STEER_ACTIVE_FILTER = STEER_FILTER_LPF
*/
steerAngleRawValue_t lldGetSteerAngleFiltrLPFRawADC ( void )
{
    return steer_filtered_adc_val;
}

/**
 * @brief       Get steering angle [rad]
 * @return      max_right   ->  STEER_RAD_RIGHT
 *              center      ->  STEER_RAD_CENTER
 *              max_left    ->  STEER_RAD_LEFT
*/
steerAngleRadValue_t lldGetSteerAngleRad ( void )
{
    steerAngleRadValue_t    steer_rad_angl  = 0;

    if( steer_filtered_adc_val < STEER_ADC_RIGHT && steer_filtered_adc_val > STEER_ADC_LEFT )
        steer_rad_angl = 0;
    else if( steer_filtered_adc_val < STEER_ADC_CENTER && steer_filtered_adc_val >= STEER_ADC_RIGHT ) // right
        steer_rad_angl = steer_filtered_adc_val * steer_right_k + steer_right_b;
    else if( steer_filtered_adc_val > STEER_ADC_CENTER && steer_filtered_adc_val <= STEER_ADC_LEFT )  // left
        steer_rad_angl = steer_filtered_adc_val * steer_left_k + steer_left_b;

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
steerAngleDegValue_t lldGetSteerAngleDeg( void )
{
    steerAngleRadValue_t rad_angle = lldGetSteerAngleRad( );
    return ( rad_angle * rad_2_deg );
}


