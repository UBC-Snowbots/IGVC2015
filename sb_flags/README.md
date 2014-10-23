#Calibrating
Using sliders will work but is not recommended because it is not easy at all.

**Using the function:**

1. Press C

2. using [ and ] change the size of the green box until only the color you want to calibrate to is within it

3. Press R to calibrate red or B to calibrate blue (*note:* the code doesnt actually care what color you are calibrating it to. What you tell it is red it will keep to it's right, what you tell it is blu it will try to keep to the right)

4. In the "Original" window there will be 3 lines:

Blue line: where the code beleives there is the closest part of the blue objects (highest blue density)

Red line: where the code beleives there is the closest part of the red objects (highest red density)

Green line: where the code beleives the robot should go

#Compiling 
*(Non ROS, only works on linux I think)*

 1. Have OpenCV installed
 2. cd into .../sb_flags/bin
 3. make
 4. ./Sight (stupid name I know but I am not all that creative)
 
