#!/bin/bash

SERIAL=/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_404-if00
echo ">>> Info on $SERIAL"
echo
stty -F $SERIAL -a
echo
ls -l $SERIAL
echo

SERIAL=/dev/ydlidar
echo ">>> Info on $SERIAL"
echo
stty -F $SERIAL -a
echo
ls -l $SERIAL
echo
