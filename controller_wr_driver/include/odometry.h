
#ifndef INCLUDE_ODOMETRY_H_
#define INCLUDE_ODOMETRY_H_

float getObjSpeedChangingOrientation( float speed_mps, float steer_angle );

float getObjAngleOrientation( float speed_mps, float steer_angle );

float getObjSpeedChangingX( float speed_mps );

float getObjPosX( void );

float getObjSpeedChangingY( float speed_mps );

float getObjPosY( void );

#endif /* INCLUDE_ODOMETRY_H_ */
