#include <common.h>

/*============================================================================*/
/* Common EXT unit                                                            */
/*============================================================================*/

static bool extDriverInitialized = false;

/**
 * @brief   Initialize EXT driver with empty config
 * @note    Safe to call any times, it checks state of previous call
 * @note    Must be called before EXT driver work
 */
void commonExtDriverInit ( void )
{
    if ( extDriverInitialized )
        return;

    static EXTConfig extcfg;
    for ( expchannel_t ch = 0; ch < EXT_MAX_CHANNELS; ch++ )
    {
        extcfg.channels[ch].mode  = EXT_CH_MODE_DISABLED;
        extcfg.channels[ch].cb    = NULL;
    }
    extStart( &EXTD1, &extcfg );

    extDriverInitialized = true;
}

inline void range_map_init(range_map_t   *ctx, 
                           float         in_min, 
                           float         in_max, 
                           float         out_min, 
                           float         out_max)
{
    if ( !ctx )
        return;

    ctx->k = (out_max - out_min)/(in_max - in_min);
    ctx->b = (out_min - ctx->k * in_min);
}

inline void range_map_init_raw(range_map_t   *ctx, 
                               float         k, 
                               float         b)
{
    if ( !ctx )
        return;

    ctx->k = k;
    ctx->b = b;
}

inline float range_map_call(range_map_t   *ctx,
                            float         val)
{
    if ( !ctx )
        return 0;

    return (ctx->k * val + ctx->b);
}
