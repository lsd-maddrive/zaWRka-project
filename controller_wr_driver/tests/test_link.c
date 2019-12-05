#include "common.h"
#include <math.h>

#include "link_def.h"

static int32_t 	encoder_value 	= 0;
static float 	encoder_2_m		= 0.001; // [ticks/m]

static void update_encoder( void )
{
	encoder_value += 10;
}

static void update_input( float speed, float steer )
{
	dbgprintf("Control data: %d / %d\n", (int)(speed*10), (int)(steer*10));
}

/*
 * @brief   Test odometry, speed and steering control via ROS
 * @note    Frequency = 50 Hz
*/
void testLinkConnection( void )
{
	mproto_driver_cb_ctx_t cb_ctx = mproto_driver_get_new_cb_ctx();
	/*
	 * Fill callbacks here
	 */
	cb_ctx.cmd_cb = update_input;

	mproto_driver_init( NORMALPRIO+1, &cb_ctx );

	debug_stream_init();
	dbgprintf("Start mproto test\n");

	uint32_t	prev_enc_value 	= 0;
	float 		prev_time 		= 0;
	float 		d_time			= 0;
	while ( 1 )
	{	
		update_encoder();

		float time = chVTGetSystemTime();		// Just system ticks
		if ( time < prev_time )
		{
			// Overflow happened!
			if ( CH_CFG_ST_RESOLUTION == 16 )
			{
				d_time = (UINT16_MAX - prev_time) + time;
			}
			else if ( CH_CFG_ST_RESOLUTION == 32 )
			{
				d_time = (UINT32_MAX - prev_time) + time;
			}
			else
			{
				chSysHalt("Whaaaat?");
			}
		}
		else
		{
			d_time = (time - prev_time);	
		}

		d_time /= CH_CFG_ST_FREQUENCY; // Divide to convert from system ticks to seconds
		float d_enc  = encoder_value - prev_enc_value;
		prev_enc_value = encoder_value;
		prev_time	   = time;

		mproto_driver_send_encoder_raw( encoder_value );
		mproto_driver_send_encoder_speed( d_enc / d_time );

		mproto_driver_send_steering( 30 * sin( time / 100000 ) );
		mproto_driver_send_pose( 5, 4, 3, 2, 1 );

		chThdSleepMilliseconds( 100 );

		palToggleLine( LINE_LED1 );
	}
}
