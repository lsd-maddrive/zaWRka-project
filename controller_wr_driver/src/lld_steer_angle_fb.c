#include <tests.h>
#include <common.h>
#include <lld_steer_angle_fb.h>

/*============================================================================*/
/* FILTER CONFIGURATION                                                       */
/*============================================================================*/

#define STEER_FILTER_MEAN       0
#define STEER_FILTER_LPF        1

#define STEER_ACTIVE_FILTER     STEER_FILTER_LPF

/*============================================================================*/
/* OBJECT CONFIGURATION                                                       */
/*============================================================================*/

#define     STEER_ADC_CENTER        1350

/*============================================================================*/
/* CONFIGURATION ZONE                                                         */
/*============================================================================*/

/***    ADC CONFIG    ***/

/***     Timer 4 = Trigger   ***/
#define ADC_MODE_TRIGGER_T4      ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(12)
/***     12-BIT RESOLUTION   ***/
#define ADC_12_BIT_CONF         (0)
#define ADC_10_BIT_CONF         (ADC_CR1_RES_0)

#define ADC_1_NUM_CHANNELS      1
#define ADC_1_BUF_DEPTH         1
static adcsample_t              adc_1_buffer[ADC_1_NUM_CHANNELS * ADC_1_BUF_DEPTH];

#define ADC_STEER_ANGL_LINE     PAL_LINE( GPIOC, 0 )

static ADCDriver    *steerADCDriver         = &ADCD1;
static bool         isInitialized       = false;

steerAngleRawValue_t    steer_raw_ADC_val   = 0;
steerAngleRawValue_t    steer_filtered_adc_val      = 0;


static void adc_1_cb ( ADCDriver *adcp, adcsample_t *buffer, size_t n )
{
    // to avoid warnings only
    adcp = adcp;
    n = n;
    buffer = buffer;

    steer_raw_ADC_val = adc_1_buffer[0];

#if( STEER_ACTIVE_FILTER == STEER_FILTER_MEAN )
    #define STEER_MEAN_WINDOW_VAL    20

    static  uint32_t            adc_cb_counter = 0;
    static steerAngleRawValue_t steer_mean_sum = 0;

    steer_mean_sum += steer_raw_ADC_val;
    adc_cb_counter += 1;

    if( adc_cb_counter == STEER_MEAN_WINDOW_VAL )
    {
        steer_filtered_adc_val = steer_mean_sum / STEER_MEAN_WINDOW_VAL;
        adc_cb_counter      = 0;
        steer_mean_sum      = 0;
    }

#elif( STEER_ACTIVE_FILTER == STEER_FILTER_LPF )
    static float steer_lpf_factor = 0.9;

    steer_filtered_adc_val = steer_raw_ADC_val * (1 - steer_lpf_factor) + steer_filtered_adc_val * steer_lpf_factor;

#endif
}

static const ADCConversionGroup steer_adc_1_cnfg = {
  .circular     = true,
  .num_channels = ADC_1_NUM_CHANNELS,
  .end_cb       = adc_1_cb,
  .error_cb     = 0,
  .cr1          = ADC_12_BIT_CONF,
  .cr2          = ADC_MODE_TRIGGER_T4,
  .smpr1        = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_480),
  .smpr2        = 0,
  .sqr1         = ADC_SQR1_NUM_CH(ADC_1_NUM_CHANNELS),
  .sqr2         = 0,
  .sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)
};

/*============================================================================*/
/* TIMER CONFIG                                                               */
/*============================================================================*/
static GPTDriver            *adcTrgDriver   = &GPTD4;

static const GPTConfig gpt_trg_cnfg = {
  .frequency =  1000000,
  .callback  =  NULL,
  .cr2       =  TIM_CR2_MMS_1,
  .dier      =  0U
 };

/*============================================================================*/
/* COEFFICIENT INIT                                                           */
/*============================================================================*/

static range_map_t  steer_left_map;
static range_map_t  steer_right_map;

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
    /***    Trigger = 2 ms ***/
    gptStartContinuous( adcTrgDriver, gpt_trg_cnfg.frequency/2000 );

    /***    Calibration results  ***/
    range_map_init_raw(&steer_left_map, 0.0354, -47.8);
    range_map_init_raw(&steer_right_map, 0.0286, -38.7);

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
steerAngleRawValue_t lldGetSteerAngleFiltrRawADC ( void )
{
    return steer_filtered_adc_val;
}

/**
 * @brief       Get steering angle [deg]
 * @return      Angle in degree
*/
steerAngleDegValue_t lldGetSteerAngleDeg ( void )
{
    steerAngleDegValue_t    steer_angle  = 0;

    if( steer_filtered_adc_val < STEER_ADC_CENTER ) // right
        steer_angle = range_map_call(&steer_right_map, steer_filtered_adc_val);
    else if( steer_filtered_adc_val >= STEER_ADC_CENTER ) // left
        steer_angle = range_map_call(&steer_left_map, steer_filtered_adc_val);

    return steer_angle;
}

/**
 * @brief       Get steering angle [rad]
 * @return      Angle in radians
*/
steerAngleRadValue_t lldGetSteerAngleRad( void )
{
    return lldGetSteerAngleDeg() / rad_2_deg;
}


