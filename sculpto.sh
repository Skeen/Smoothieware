#!/bin/bash

cp ConfigSamples/Sculptoboard/config src/config.default
./BuildShell make -j8
make combined
scp LPC1768/main-combined.hex root@192.168.1.111:
