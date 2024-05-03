
#         *** GODZILLA HUNTER ***
Created by Parker, Jacob, Luca, and Collin
CPRE 288 Final Project
Completed 5/3/2024
The mission objective is to search around a field of objects and autonomously avoid them.
The robot will track its location in the field on an x,y coordinate system.
The robot will also identify the obstacle that is within the range for Godzilla, the biggest tall object on the field.
When the robot identifies Godzilla, it will eliminate the target, thus completing its objective.


# Putty Commands:
  q - runs a single scan and saves the objects, useful for testing mostly <br>
  t - Starts the overall scan process <br>
  b - while in the scan loop will break after the next scan is done if outside of the loop prints debug <br>
  r - free obs and resets coordinates for a manual restart <br>
  p - pauses and causes the program to hang for no good reason <br>
  f - frees everything USE at end of program <br>
  w,a,s,d - manual movement just in case <br>
  k - kill, ram into whatever it is looking at<br>

# Coordinate system
Our coordinate system is x,y, and a heading
 - Y is positive going forward
 - X is positive going right
 - The heading is between 0 (when facing forward) and 359. (180 is backward)
 - The heading goes up when the robot turns clockwise (until it loops back around)
We use trigonometry to use the heading to calculate the x and y of the robot
It looks a little different than your usual conversion because of how the heading tracks.
Each movement direction (forward, backward, and turning) keep track of the coordinates


# Calibration Routine:

**Defines guide**

  1. MOVEOFFSET : added to the dist use to correct over/under shoot in forward/backward movement
  2. TURNOFFSET : added to the degrees turned - for undershot turns + for overshot turns
  3. TWISTOFFSET : adjusts the power of the wheels relative to each other, positive for clockwise turn while driving forward
  4. PINGOFFSET : adds constant value to ping value
  5. START_VAL : fixed value that represents timer value at 0 degrees
  6. END_VAL : fixed value that represents timer value at 180 degrees

**1. Movement**

  1. moves 1 meter
  2. moves 2 meters
  3. turns 90 left then correct it
  4. set the movement defines

**2. Servo**

  1. pick the left bound by alinging the servo
  2. pick the right bound by aligning the servo
  3. rewrite the defines

**3. Ping**

  1. set robot on a meter stick
  2. run the calibration setup
  3. rewrite defines by determining how far of the ping reading
     
**4. IR**

1. run the movement calibration
2. set the robot 10cm away from a wall
3. robot will reverse away from wall and generate a lookup table for IR values based on wheel encoders
