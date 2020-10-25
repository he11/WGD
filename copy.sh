#!/bin/sh
# local directory name

# path
SRC_DIR=/home/js/project/arduino/work_rep/wearable_gas_detector
LOCAL_DIR=/media/sf_sf/arduino/gas_detector
SRC_LIST="mcu_main.ino" # ASCFont.h hangul.cpp  hangul.h  KSFont.h"

# build test purpose
for src in ${SRC_LIST}; do
	cp $SRC_DIR/${src} $LOCAL_DIR/mcu_main
done

echo copy ok

