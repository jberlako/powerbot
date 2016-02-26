Powerbot
========

Repository for work on the Powerbot.

#### 2016-2-26
- Updated the repository several days ago with a cleaned up version of the DriveTestRC code that was compiled and deployed on the Intel Edison.
- Added serial debug statements to attempt to fix direction reversal behavior at speed.
- Added neutral point calibration. The robot now follows a new procedure for calibration. This will hopefully alleviate small ghost signal. Here is the new coded calibration timing;
  - 1 second neutral calibration; DO NOT TOUCH THROTTLE OR STEERING DURING THIS TIME
  - 9 second min/max calibration; fully actuate the throttle in both forward and reverse, turn steering full left and full right.


---
#### 2015-9-18  
Added the baseline Arduino code to the repository. We've been using the first commit version of the code since Maker Faire.
