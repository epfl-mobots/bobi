#!/bin/bash

set -e

# Verify that PylonViewerApp is not launched
isPylonLaunched=`ps ax | grep PylonViewerApp | grep -v grep | sed 's/[[:space:]]*$//'`
if [ ! -z "$isPylonLaunched" ]; then
        echo PylonViewerApp already launched. Please quit PylonViewerApp before attempting to run this script.
        exit 1
fi

if [[ $(pidof gst-launch-1.0) ]]; then
        echo Stopping gstreamer active streams
        sudo kill -9 `pidof gst-launch-1.0`
fi
