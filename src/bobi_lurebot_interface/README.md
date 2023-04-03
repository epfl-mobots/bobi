# bobi_lurebot_interface
Behavioural Observation and Biohybrid Interaction framework - LureBot interface module

This module contains communication interface code for the LureBot.

## Dependencies

- SimpleBLE: This is a forked version specifically redesigned for the LureBot's operation. It will be automatically downloaded from the CMakeLists.txt.
  - SimpleBLE depends on DBUS-1 (on Ubuntu machines ``$ sudo apt install libdbus-1-dev``).

## Run
There are 3 launch files. Each one corresponds to a different LureBot:

``$ roslaunch bobi_lurebot_interface lurebot_X.launch``

## Bluetooth configuration on linux machines

The LureBot is designed to communicate with the master at 65Hz. To achieve this rate on a default Ubuntu (or linux) installation you will need to edit the bluetooth parameters of your system.

1. Copy the `scripts/main.conf` to the Bluetooth configuration directory.
2. Set the connection parameters by executing ``$ sh scripts/set_connection_params.sh``.
