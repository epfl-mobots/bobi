# bobi_control
Behavioural Observation and Biohybrid Interaction framework - Control module

This module contains motion & behavioural control procedures.

## Run
Start a PID (with velocity a priori knowledge):

``$ roslaunch bobi_control apriori_controller.launch``

then, start a behavioural model:

``$ roslaunch bobi_control burst_and_coast.launch``

## Calibration 
We have implemented an auto-calibration routine to align the top and bottom camera coordinate systems.
