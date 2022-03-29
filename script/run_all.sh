#!/bin/bash

set -e

NAME="drone-01"
RESOLUTION=480

echo "STEP 1"
python3 01-save_device_calibration.py depthai_calib_${NAME}.json

echo "STEP 2"
python3 02-flash_device_calibration.py depthai_calib_${NAME}.json camchain-${NAME}.yaml

echo "STEP 3"
python3 03-generate-mesh-file.py --resolution ${RESOLUTION}

rm depthai_calib_${NAME}.json
