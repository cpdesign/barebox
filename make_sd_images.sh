#!/bin/bash

VCSVER=`git rev-parse --short HEAD`
echo $VCSVER

FN_NORMAL=vpr200_tester_$VCSVER.bin
FN_ALT=vpr200_tester_ALT_$VCSVER.bin
echo "Making $FN_NORMAL and $FN_ALT"

#make the ALT image
make distclean
cp vpr200_tester_ALTconfig .config
# make 
USE_MEM_BANK_1=0 make
cp barebox.bin $FN_ALT
# pad out the image so that it fits the full parition size that barebox expects
truncate -s 512k $FN_ALT 
cat barebox_default_env >> $FN_ALT

# make the standard image
make distclean
cp vpr200_tester_config .config
make
cp barebox.bin $FN_NORMAL 
truncate -s 512k $FN_NORMAL 
cat barebox_default_env >> $FN_NORMAL

echo "Made $FN_NORMAL and $FN_ALT"
