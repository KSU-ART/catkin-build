#!/bin/bash
if ! test -d ~/opencv; then cd ~; git clone https://github.com/Itseez/opencv.git; cd ~/opencv; git checkout 2.4.12; mkdir release; cd release; cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..; make -j8; fi

