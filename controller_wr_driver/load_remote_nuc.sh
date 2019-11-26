#!/bin/bash

ADDR=usersaunuc-NUC7i5BNK.local
USER=user-sau-nuc

scp build/ch.bin $USER@$ADDR:/tmp
ssh $USER@$ADDR 'st-flash write /tmp/ch.bin 0x8000000'
