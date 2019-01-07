#!/bin/sh

echo 1 >/sys/bus/pci/rescan


grep -rn -e 1172 /sys/bus/pci/devices/*/subsystem_vendor >/dev/null
if [ ! $? -eq 0 ]; then
  echo "Altera FPGA (vendor ID 0x1172) not found. Cannot unplug."
  exit
else
  echo "Altera FPGA (vendor ID 0x1172) found."
  exit
fi

