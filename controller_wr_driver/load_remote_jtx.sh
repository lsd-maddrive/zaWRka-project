#!/bin/bash

ADDR=usersaujtx-desktop.local
USER=user-sau-jtx

scp build/ch.bin $USER@$ADDR:/tmp
ssh $USER@$ADDR 'st-flash write /tmp/ch.bin 0x8000000'
