#!/bin/bash

NUC_ADDR=10.139.0.49

scp -P9992 build/ch.bin user-sau-nuc@$NUC_ADDR:/tmp
ssh -p9992 user-sau-nuc@$NUC_ADDR 'st-flash write /tmp/ch.bin 0x8000000'
