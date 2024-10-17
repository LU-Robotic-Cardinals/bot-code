## Goal
The current goal of the X-Drive project is to simplify the control of X-Drive and X-Adjacent motor arrangements (drives with solely onmi-wheels and orientation-independent strafing). The math behind calculating the individual motor speed coefficients is not complicated in itself, but it is necessary to build an abstraction layer to prevent unnecessary code repetition between bots, and to aid in quick calibration and revision.
## Current State
The current state of the code has the programmer defining a struct for each motor, compiling those motors into an array, and iterating over that array to perform basic actions like fetching coefficients, finding max value, applying motor speed, etc.

## Future
Ideally, the final state of the project will be as follows. The programmer will define motor angle from a reference "front" and position from the rotational center of the robot for each robot. Then, depending on the motor layout several endpoints will be exposed to the programmer. These will include, autonomous movement, remote driving, and error testing and mitigation.
#### Autonomous Movement
The programmer will be able to supply many ways of dictating where the robot should go. These methods include, but are not limited to, Point Motion, Bezier Curves, Locking, and Tracking. Point motion means a position is given relative to the bot's current position or relative to the bot's starting point and the robot will move straight to it. This includes the 2 cardinal directions, as well as its yaw angle. Bezier Curve motion will be achieved by supplying either a curve function, or several points that define the desired curve shape. The T value will sweep from a default of 0 to a default of 1 while driving to the point that the curve dictates for the defined T value. speed Because some curves
## Ramblings
## To-do
## Ideas























