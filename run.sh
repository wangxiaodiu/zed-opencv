#!/bin/bash
cd ~/niu/zed-opencv/build
(./ZED_with_OpenCV;killall ZED_TTS.py) &
cd ..
sleep 8 && ./ZED_TTS.py
