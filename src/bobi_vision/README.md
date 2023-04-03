# bobi_vision
Behavioural Observation and Biohybrid Interaction framework - Vision module

This module contains all procedures implemented in order to track the robot and lure/fish in real-time.

## Dependencies

ROS noetic covers all dependencies

## Run
If you are using the proposed setup then start the camera streams with the suggested settings by issuing:

``$ sh scripts/init_setup_cameras.sh``

If this is succesful, start the tracking and GUI:

``$ roslaunch bobi_vision start_tracking.launch``

## Calibration 
We have implemented an auto-calibration routine to align the top and bottom camera coordinate systems.

1. Insert the LureBot in the robot arena
2. Remove everything from the experimental setup except for the magnetic lure. Note that if your are going to conduct experiments with water, then it's suggested that you fill the tank with the appropriate water level (to also account for distortion caused by it).
3. ``$ roslaunch bobi_vision camera_mapper.launch``
