#!/bin/bash

set -e

DRONE_NO="01"

echo "STEP 1"
python3 01-save_device_calibration.py depthai_calib-poe-720p.json

echo "STEP 2"
python3 02-flash_device_calibration.py depthai_calib-poe-720p.json camchain-poe-720p.yaml

echo "STEP 3"
python3 03-generate-mesh-file.py 

rm depthai_calib-poe-720p.json
