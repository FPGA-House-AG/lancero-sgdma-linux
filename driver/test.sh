# send debug traffic to 82.171.51.231:4444
# via gateway MAC 00:22:3f:e7:30:a1
#
#sudo modprobe netconsole \
#netconsole=@192.168.1.22/eth0,@192.168.1.81/a4:ba:db:e7:ff:99

exit_on_error ()
{
  if [ ! $? -eq 0 ]; then
    echo "Error loading driver (start-up tests failed?). Run 'dmesg'"
    exit
  else
    echo "Driver loaded and started OK."
    dmesg | tail -n 1
  fi
}

make && sync
#sudo chown $USER.$USER *

sudo rmmod lancero

# test_to_dev = 1 means "to endpoint device", thus read engine, 0 for write engine
# test_dma_runs = 1 means perform only 1 run
# test_dma_size = 128 means transfer size fixed to 128 bytes
# test_dma_boundary_offset = 0 means offset of 0 bytes against 4096 byte boundary
#

sudo insmod ./lancero.ko seed=1 test_to_dev=1 test_dma_size=128 test_dma_boundary_offset=0 desc_num=1 test_dma_runs=10
exit_on_error
#sudo insmod ./lancero.ko seed=1 test_to_dev=1 test_dma_size=128 test_dma_boundary_offset=0 desc_num=128 test_dma_runs=1
#exit_on_error
#sudo insmod ./lancero.ko seed=1 test_to_dev=1 test_dma_size=128 test_dma_boundary_offset=0 test_dma_runs=100
#exit_on_error

sudo insmod ./lancero.ko major=130 engine_first=0 engine_last=0
exit_on_error

#dd if=/dev/zero of=/248 bs=4096 count=1


#dmesg | tail -n1
#dd if=/lancero of=/dev/null bs=8 count=1

# block size
#export BS=1024
# data generator base address for write engine
#export DG=0x80000000

#dd if=/dev/urandom of=random.bin bs=$BS count=1

#dd if=random.bin of=/248 bs=$BS seek=$(($DG/$BS)) count=50

