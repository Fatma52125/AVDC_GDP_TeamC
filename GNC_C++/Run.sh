#!/bin/bash
sleep 5
sudo /bin/echo pruecapin_pu >/sys/devices/platform/ocp/ocp:P8_15_pinmux/state
sleep 5
cd /var/lib/cloud9/Development_Code
sudo chrt -r 99 ./Main
exit 0