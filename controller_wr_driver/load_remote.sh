#!/bin/bash

# NUC_ADDR=192.168.31.99
# NUC_ADDR=10.139.20.7
# NUC_ADDR=192.168.43.159

scp build/ch.bin user-sau-nuc@$NUC_ADDR:/tmp
ssh user-sau-nuc@$NUC_ADDR 'st-flash write /tmp/ch.bin 0x8000000'
