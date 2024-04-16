## FTC 25268 TeamCode

This is the code for UK-497, 25268. 

### The files

MainDrive.java - The tele-op mode.

AutoComputerVision - The autonomous mode (not tested). It uses Tensorflow Lite to find the x position of the gamepeice in the webcam view. It then uses RUN_TO_POSITION with PIDF to go to one of the CENTRESTAGE spike marks, based on the gamepeice position. The arm is lowered, then the grip is closed.