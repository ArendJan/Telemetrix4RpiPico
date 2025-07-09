#!/bin/bash
set -xe
# wait for input

sftp mirte@192.168.0.10  <<< $'put Telemetrix4RpiPico.uf2' 
ssh mirte@192.168.0.10 "sudo picotool load -f Telemetrix4RpiPico.uf2"