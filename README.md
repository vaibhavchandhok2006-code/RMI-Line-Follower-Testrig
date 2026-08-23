# Line Follower — RMI TestRig Project

## Overview
Differential-drive line-following robot built as a part of RMI Testrig. Follows a 38mm black line through a hostile track (sharp corners, junctions), runs an exploration round to build a track map, then executes an optimized second run using path-planning on the discovered graph.

**Status as of [DATE]:** Design review stage — CAD assembly and circuit design complete, firmware development starting next.

## Repo Structure
- `/cad` — SolidWorks assembly + exported STL files, chassis, motor mounts, sensor mount
- `/docs` — circuit diagram, BOM
- `/firmware` — ESP-IDF firmware (in progress)
- `BUILDLOG - Mentions the project journey
- `README` - Mentions repo structure , project description, required summaries

## Hardware Summary
- ESP32 (compute + control)
- 2× N20 motors with encoders (drive + odometry)
- RLS-08 IR reflectance sensor array (line sensing)
- TB6612FNG motor driver
- 2S LiPo battery, 7.4V nominal, LM2596 buck converter for 5V rail
- Full BOM: see `/docs/bom.md`


## How to Build / Run
Firmware setup instructions will be added once ESP-IDF development begins. CAD files can be opened directly in SolidWorks or viewed via the exported STL file.

## Team
Vaibhav Chandhok, 2nd year Mechanical, RMI Robotics and Machine Intelligence Club , NIT Trichy
