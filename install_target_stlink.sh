#!/bin/bash

mkdir -p deps

git -C deps/stlink pull || git -C deps clone https://github.com/texane/stlink.git

INSTALL_PATH=/usr/local

mkdir -p deps/stlink/build && cd deps/stlink/build && cmake -D CMAKE_INSTALL_PREFIX=$INSTALL_PATH .. && cmake --build . -- -j`nproc`

sudo make install

#echo "PATH=\$PATH:$INSTALL_PATH/bin"
#echo "LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/stlink/lib"
