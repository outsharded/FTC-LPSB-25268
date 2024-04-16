# First Tech Challenge UK-497, 25268

This is the code for the 2023-24 CENTRESTAGE team 25268. The TeamCode is under /TeamCode/src/main/java/org/firstinspires/ftc/teamcode

## The files

### MainDrive.java - The tele-op mode.

### AutoComputerVision - The autonomous mode (not tested). 
* It uses Tensorflow Lite to find the x position of the gamepeice in the webcam view. 
* It then uses RUN_TO_POSITION with PIDF to go to one of the CENTRESTAGE spike marks, based on the gamepeice position. 
* The arm is lowered, then the grip is closed.

## TODO for future 
[] Rebuild autonomous using RoadRunner
[] Add Arm Zeroing (going to its home position), using touch sensors
