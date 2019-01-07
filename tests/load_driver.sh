#!/bin/bash

# Specifies the major device number used for the Lancero device driver.
# Choose any number that is not in use on your system 
MAJOR=130

sudo rmmod -s lancero

echo -n "(Re-) Creating device nodes in the current directory... (might ask for sudo password)"
sudo rm -f lancero_{user,control,events,sgdma}

sudo mknod -m 0666 ./lancero_user c $MAJOR 0
sudo mknod -m 0666 ./lancero_control c $MAJOR 1
sudo mknod -m 0666 ./lancero_events c $MAJOR 2
sudo mknod -m 0666 ./lancero_sgdma c $MAJOR 3
echo " DONE"

echo -n "Loading driver..."
# performance_dirs:
# 0 write engine
# 1 read engine
# 2 both
sudo insmod ../driver/lancero.ko major=$MAJOR bar_map_size=32768 #performance_dirs=1
if [ ! $? == 0 ]; then
  echo " FAILED"
  exit 1
fi
echo " DONE"
