#ifndef ROS_CHIBIOS_HARDWARE_H
#define ROS_CHIBIOS_HARDWARE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include <hal.h>

#include <math.h>

class ChibiOSHardware {
    public:

        ChibiOSHardware() {}

        void init()
        {
            extern BaseChannel* ros_sd_ptr;
            iostream    = ros_sd_ptr;

            st2ms       = 1000.0 / CH_CFG_ST_FREQUENCY;
        }

        int read()
        {
            return chnGetTimeout(iostream, TIME_IMMEDIATE);
        }

        void write(uint8_t* data, int length)
        {
            chnWrite(iostream, data, length);
        }

        uint32_t time()
        {
            /* TODO --- this function may be critical as uses floats */
            /* replaces ST2MS as it overflows on (2^32 / 1000) */
            /* dont forget to enable hardware FPU */
            return ceilf(chVTGetSystemTimeX() * st2ms);
            // return ST2MS(chVTGetSystemTimeX());
        }

    protected:
        BaseChannel *iostream;
        float       st2ms;
};

#ifdef __cplusplus
}
#endif

#endif /* ROS_CHIBIOS_HARDWARE_H */
