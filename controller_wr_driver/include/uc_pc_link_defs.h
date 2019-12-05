#ifndef UC_PC_LINK_DEFS_H_
#define UC_PC_LINK_DEFS_H_

/*** Input data ***/
/*
 * Velocity command data
 * Data:
 *  float[0] = velocity [mps]
 *  float[1] = steering [deg]
 */
#define WR_IN_CMD_VEL               1
/*
 * Reset odometry command
 */
#define WR_IN_CMD_RESET_ODOM        2
/*
 * Velocity command data
 * Data:
 *  float[0] = velocity [prc] - [-100;100]
 *  float[1] = steering [prc] - [-100;100]
 */
#define WR_IN_CMD_RAW_VEL           3

/*** Output data ***/
/*
 * Odometry data
 * Data:
 *  float[0] = x [m]
 *  float[1] = y [m]
 *  float[2] = yaw [deg]
 *  float[3] = vx [mps]
 *  float[4] = uz [rad/s]
 */
#define WR_OUT_CMD_ODOM_DATA        1
/*
 * Steering ADC raw data
 * Data:
 *  float[0] = raw steering [ADC]
 */
#define WR_OUT_CMD_STEER_RAW        2
/*
 * Steering angle
 * Data:
 *  float[0] = steering angle [deg]
 */
#define WR_OUT_CMD_STEER_ANGLE      3
/*
 * State value data
 * Data:
 *  int8[0] = state value
 */
#define WR_OUT_CMD_STATE            4
/*
 * Encoder rotation value
 * Data:
 *  float[0] = rotations
 */
#define WR_OUT_CMD_ENCODER_VALUE    5
/*
 * Encoder rotation speed value
 * Data:
 *  float[0] = rotation speed [rot/s]
 */
#define WR_OUT_CMD_ENCODER_SPEED    6

#endif // UC_PC_LINK_DEFS_H_
