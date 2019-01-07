#!/bin/sh

grep -rn -e 1172 /sys/bus/pci/devices/*/subsystem_vendor >/dev/null
if [ ! $? -eq 0 ]; then
  echo "Altera FPGA (vendor ID 0x1172) not found. Cannot unplug."
  exit
fi

GREPOUT=`grep -rn -e 1172 /sys/bus/pci/devices/*/subsystem_vendor`
echo $GREPOUT
DIRNAMEOUT=`dirname $GREPOUT`
echo $DIRNAMEOUT/remove
echo 1 >$DIRNAMEOUT/remove

grep -rn -e 1172 /sys/bus/pci/devices/*/subsystem_vendor >/dev/null
if [ ! $? -eq 1 ]; then
  echo "Altera FPGA board (vendor ID 0x1172) still present, unplug failed."
  exit
else
  echo "Altera FPGA board (vendor ID 0x1172) unplugged, reconfigure it."
  exit
fi
