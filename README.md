# godzillaHunter
CPRE288 Final mission repo

# Putty Commands:
  q - runs a single scan and saves the objects useful for testing mostly <br>
  t - Starts the overall scan process <br>
  b - while in the scan loop will break after the next scan is done if outside of the loop prints debug <br>
  r - free obs and resets coordinates for a manual restart <br>
  p - pauses and causes the program to hang for no good reason <br>
  f - frees everything USE at end of program <br>
  w,a,s,d - manual movement just in case <br>
  k - kill, ram into whatever it is looking at<br>



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
