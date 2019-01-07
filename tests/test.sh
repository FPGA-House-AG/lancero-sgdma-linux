#!/bin/sh

./dma_to_device -d ./lancero_sgdma -s $((160*4096))

exit

for seed in $(seq 1 1 2000)
do

  echo ./ram -d ./lancero -s 128 -n 1000000 -r $seed -b ...

  ./ram -d ./lancero -s 128 -n 1000000 -r $seed -b

  # failed?
  if [ ! $? -eq 0 ]; then
    exit 1
  fi
done

