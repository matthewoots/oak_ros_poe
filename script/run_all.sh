#!/bin/bash

set -e

NAME="drone02"
CALIB_TYPE="pinhole"

# IMPORTANT: for OAK-D-PRO please use 400; for OAK-D-Lite please use 480
RESOLUTION=480

echo "STEP 1"
python3 01-save_device_calibration.py depthai_calib-poe-720p.json

echo "STEP 2"
python3 02-flash_device_calibration.py depthai_calib-poe-720p.json camchain-poe-720p.yaml

echo "STEP 3"
python3 03-generate-mesh-file.py --resolution ${RESOLUTION}

rm depthai_calib-poe-720p.json
