#!/bin/bash

./scripts/setup_imx219_1640x1232.sh

k3conf set clock 208 0 1000000000

python3 apps_python/app_edgeai.py  ./configs/radar-vision-fusion-OD-demo.yaml  -n  -r

