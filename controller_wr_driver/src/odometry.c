#include <tests.h>
#include <odometry.h>
#include <math.h>


float speed_teta    = 0;
float angle_teta    = 0;
float speed_x       = 0;
float pos_x         = 0;
float speed_y       = 0;
float pos_y         = 0;

float getObjSpeedChangingOrientation( float speed_mps, float steer_angle )
{
    speed_teta = (speed_mps * tan(steer_angle) ) / ( WHEEL_BASE_CM * 0.01 ) ;
    return speed_teta;
}


float getObjAngleOrientation( float speed_mps, float steer_angle )
{
    angle_teta += ( speed_mps * tan(steer_angle) ) / ( WHEEL_BASE_CM * 0.01 ) ;

    return angle_teta;
}

float getObjSpeedChangingX( float speed_mps )
{
    speed_x = speed_mps * cos( angle_teta );
    return speed_x;
}

float getObjPosX( void )
{
    pos_x += speed_x;
    return pos_x;
}

float getObjSpeedChangingY( float speed_mps )
{
    speed_y = speed_mps * sin( angle_teta );
    return speed_y;
}

float getObjPosY( void )
{
    pos_y += speed_y;
    return pos_y;
}

