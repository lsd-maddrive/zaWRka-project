#include <common.h>

#include "mproto.h"
#include "link_def.h"
#include "uc_pc_link_defs.h"

static const mproto_driver_cb_ctx_t default_cb_ctx = {
    .cmd_cb                 = NULL,
    .raw_cmd_cb             = NULL,

    .set_odom_params_cb     = NULL,
    .reset_odometry_cb      = NULL
};

static mproto_driver_cb_ctx_t last_cb_ctx;

mproto_driver_cb_ctx_t mproto_driver_get_new_cb_ctx( void )
{
    return default_cb_ctx;
}

void reset_odometry_cb( void )
{
    if ( !last_cb_ctx.reset_odometry_cb )
        return;

    last_cb_ctx.reset_odometry_cb();
}

void cmd_vel_cb( float data[2] )
{
    if ( !last_cb_ctx.cmd_cb )
        return;

    float *vel_data = (float *)data;

    float cmd_speed = vel_data[0];
    float cmd_steer = vel_data[1];

    cmd_speed = CLIP_VALUE(cmd_speed, -WR_INPUT_CMD_SPEED_LIMIT_MPS, WR_INPUT_CMD_SPEED_LIMIT_MPS);
    cmd_steer = CLIP_VALUE(cmd_steer, -WR_INPUT_CMD_STEER_LIMIT_DEG, WR_INPUT_CMD_STEER_LIMIT_DEG);

    last_cb_ctx.cmd_cb( cmd_speed, cmd_steer );
}

void raw_cmd_vel_cb( float data[2] )
{
    if ( !last_cb_ctx.raw_cmd_cb )
        return;

    float *vel_data = (float *)data;

    float cmd_speed = vel_data[0];
    float cmd_steer = vel_data[1];

    cmd_speed = CLIP_VALUE(cmd_speed, -100, 100);
    cmd_steer = CLIP_VALUE(cmd_steer, -100, 100);

    last_cb_ctx.raw_cmd_cb( cmd_speed, cmd_steer );
}

void cmd_cb( mpcmd_t cmd, uint8_t *data, size_t len )
{
    if ( cmd == WR_IN_CMD_VEL ) {
        if ( len != sizeof(float[2]) )
            return;

        cmd_vel_cb( (float *)data );
    } else if ( cmd == WR_IN_CMD_RESET_ODOM) {
        reset_odometry_cb();
    } else if ( cmd == WR_IN_CMD_RAW_VEL) {
        if ( len != sizeof(float[2]) )
            return;

        raw_cmd_vel_cb( (float *)data );
    }
}

/*
 * Spin thread - used to receive messages
 */

mproto_ctx_t mproto_ctx = NULL;

static THD_WORKING_AREA(waSpinner, 256);
static THD_FUNCTION(Spinner, arg)
{
    (void)arg;
    chRegSetThreadName("Spinner");

    while (true)
    {
        systime_t time = chVTGetSystemTimeX();

        mproto_spin( mproto_ctx, 15 );
        
        chThdSleepUntilWindowed( time, time + MS2ST( 20 ) );
    }
}

//=======================================================


void mproto_driver_send_state( int8_t state )
{
    mproto_send_data(mproto_ctx, WR_OUT_CMD_STATE, (uint8_t *)&state, sizeof(state));
}

void mproto_driver_send_steering( float steer_angle )
{
    mproto_send_data(mproto_ctx, WR_OUT_CMD_STEER_ANGLE, (uint8_t *)&steer_angle, sizeof(steer_angle));
}

void mproto_driver_send_raw_steering( uint16_t raw_steering )
{
    mproto_send_data(mproto_ctx, WR_OUT_CMD_STEER_RAW, (uint8_t *)&raw_steering, sizeof(raw_steering));
}

void mproto_driver_send_pose( float x, float y, float dir, float vx, float uz )
{
    static float data[5];

    data[0] = x;
    data[1] = y;
    data[2] = dir;
    data[3] = vx;
    data[4] = uz;

    mproto_send_data(mproto_ctx, WR_OUT_CMD_ODOM_DATA, (uint8_t *)data, sizeof(data));
}

void mproto_driver_send_encoder_raw( float value )
{
    mproto_send_data(mproto_ctx, WR_OUT_CMD_ENCODER_VALUE, (uint8_t *)&value, sizeof(value));
}

void mproto_driver_send_encoder_speed( float value )
{
    mproto_send_data(mproto_ctx, WR_OUT_CMD_ENCODER_SPEED, (uint8_t *)&value, sizeof(value));
}

/*===========================================================================*/
/* SDU relative                                                              */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

extern const USBConfig usbcfg;
extern SerialUSBConfig serusbcfg;
extern SerialUSBDriver SDU1;

#ifdef __cplusplus
}
#endif

// SerialDriver    *ros_sd     = &SD5;
/* Must be named <ros_sd_ptr> for ChibiOSHardware */
static BaseChannel  *ros_sd_ptr = (BaseChannel *)&SDU1;
static float        st2ms       = 1000.0 / CH_CFG_ST_FREQUENCY;;

static int16_t read_byte(void)
{
    return chnGetTimeout(ros_sd_ptr, TIME_IMMEDIATE);
}

static void write_bytes(uint8_t *data, size_t len)
{
    chnWriteTimeout(ros_sd_ptr, data, len, TIME_IMMEDIATE);
}

static mptime_t get_time(void)
{
    /* TODO --- this function may be critical as uses floats */
    /* replaces ST2MS as it overflows on (2^32 / 1000) */
    /* dont forget to enable hardware FPU */
    return ceilf(chVTGetSystemTimeX() * st2ms);
    // return ST2MS(chVTGetSystemTimeX());
}


mproto_func_ctx_t funcs_ctx = {
    .get_byte = read_byte,
    .put_bytes = write_bytes,
    .get_time = get_time
};

void mproto_driver_init( tprio_t prio, mproto_driver_cb_ctx_t *ctx )
{
    sduObjectInit( &SDU1 );
    sduStart( &SDU1, &serusbcfg );

    if ( ctx )
        last_cb_ctx = *ctx;
    else
        last_cb_ctx = default_cb_ctx;

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus( serusbcfg.usbp );
    chThdSleepMilliseconds( 1500 );
    usbStart( serusbcfg.usbp, &usbcfg );
    usbConnectBus( serusbcfg.usbp );

    mproto_ctx = mproto_init(&funcs_ctx);

    mproto_register_command( mproto_ctx, WR_IN_CMD_VEL, cmd_cb );
    mproto_register_command( mproto_ctx, WR_IN_CMD_RESET_ODOM, cmd_cb );
    mproto_register_command( mproto_ctx, WR_IN_CMD_RAW_VEL, cmd_cb );

    chThdCreateStatic( waSpinner, sizeof(waSpinner), prio, Spinner, NULL );
}
