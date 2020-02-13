# Gryphon Drone Forensics Tool

Named after the mythical creature of mythology, this tool aims to extract critical events happened during the flight of an Unmanned Aerial System/Vehicle, running `Ardupilot` flight stack. This tool is part of the research paper `Gryphon: Forensics on Dataflash and Telemetry Logs`, presented in the 14th International Workshop on Security 2019 in Tokyo, Japan.

This research methodology for analysing the Dataflash logs consists of 6 steps. Their scope starts from firmware integrity, and goes through trajectory, execution, and error analysis to reach low-level hardware logs and finish with timeline analysis. More precisely, the steps of our methodology are the following:

`Check integrity of the UAV:` The goal of this step is to determine whether the firmware running on the drone has been tampered with.

`Trajectory analysis:` By visualizing the trajectory of the UAV one may determine possible differentiations in the course and decrease the timeline that has to be analyzed. For example one may notice that the trajectory of the UAV is not the expected one after a specific timeframe. Moreover, by detecting anomalies in the trajectories one may gain further insight on an incident. For instance, a sudden variation in the height of the UAV may imply collision with an object. Part of the code of mavflightview is used, and is included in the repository, to ensure proper execution.

`Command verification:` The goal of this step is to determine whether all the commands that were submitted by the pilot have been executed.

`Error analysis:` In this step all errors reported by MAVlink are collected to determine whether any of the reported errors resulted to fatal errors or warnings that affected the estimated flight capability.
Analysis of sensory measurements: The goal of this step is to determine whether all the measurements from the embedded sensors can be considered within the expected range. Anomalies in the sensor measurements may imply a hardware problem in the UAV. Note that such issues may not trigger errors.

`Timeline analysis:` A Timeline Analysis is the possess of chronologically arranging data of the flight, a crucial part of any digital forensics examination. This part enables the forensic investigator to correlate the found evidence and understand what has happened in the case under investigation.


## Execution
```
python3 gryphon.py <LOGFILE.bin>
```

### Dependencies
`Warning` Make sure you have enough space (over 3GB) on `/tmp` as `wxPython` may cause problem during download
```
apt install libgtk-3-dev python3-pip
pip3 install pymavlink mavproxy opencv-python wxPython GitPython termcolor
```
