#!/bin/bash

NUC_ADDR=192.168.31.99
NUC_ADDR=10.139.20.7

scp -P9992 build/ch.bin user-sau-nuc@$NUC_ADDR:/tmp
ssh -p9992 user-sau-nuc@$NUC_ADDR 'st-flash write /tmp/ch.bin 0x8000000'
