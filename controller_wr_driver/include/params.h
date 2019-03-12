#pragma once

typedef struct
{
	/* Shift from zero state value */
	/* 
	 * if 1500 - zero:
	 * 		min_dc = 1500 + esc_min_dc_shift
	 * 		max_dc = 1500 + esc_max_dc_offset
	 */
	int32_t esc_min_dc_offset;
	int32_t esc_max_dc_offset;

} control_params_setup_t;
