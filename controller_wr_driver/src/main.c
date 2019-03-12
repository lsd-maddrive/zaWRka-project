#include <common.h>
#include <tests.h>
#include <chprintf.h>

int main(void)
{
    chSysInit();
    halInit();

    #if (MAIN_PROGRAM_ROUTINE != PROGRAM_ROUTINE_MASTER)

        testsRoutines();
//        ros_driver_cb_ctx_t cb_ctx = ros_driver_get_new_cb_ctx();
//        cb_ctx.cmd_cb = cntrl_handler;
//        ros_driver_set_cb_ctx( &cb_ctx );
    #else



    #endif
}
