# robot_diagnostic
This package track the topics using diagnostic_updater and publish system_status.

## Topic Configuration
You can configure your topics in the [yaml/topics.yaml](yaml/topics.yaml).
Data required for configuration are mentioned in the [yaml file](yaml/topics.yaml)

## system_status
Out put of the package is the system_status containing the overall system state and system level (0:GOOG, 1:WARNING, 2:ERROR).
The system_state also has list of topics in GOOD , WARNING and ERROR.