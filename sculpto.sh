#!/bin/bash

cp ConfigSamples/Sculptoboard/config src/config.default
./BuildShell make -j8
make combined
scp LPC1768/main-combined.hex root@$1:
ssh root@$1 "/root/PrinterAPI/tools/lpc_flasher/flash.sh /root/main-combined.hex"
