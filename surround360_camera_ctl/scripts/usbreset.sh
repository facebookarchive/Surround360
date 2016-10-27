#!/bin/bash

if [[ $EUID != 0 ]] ; then
  echo This must be run as root!
  exit 1
fi

counter=0

for id in $(lspci | grep Renesas | cut -f 1 -d " ")
do
  echo "Resetting $id..."
  echo -n 0000:$id > /sys/bus/pci/drivers/xhci_hcd/unbind
  echo -n 0000:$id > /sys/bus/pci/drivers/xhci_hcd/bind
  counter=$((counter+1))
done

echo "$counter USB controllers have been reset!"
