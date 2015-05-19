IGVC2015 
========

This is the AI branch.

To pull:

> git fetch origin

> git checkout -b AI origin/AI

To check which branch you are on:

> git branch


To only build the sb_ai package, cd into the root of the workspace:

> catkin_make -DCATKIN_WHITELIST_PACKAGES="sb_ai"

Every time catkin_make is called it will only build the sb_ai package

To revert back to building all the packages:

> catkin_make _DCATKIN_WHITELIST_PACKAGES=""

-----------------------------------------

To use the sb_gps branch: 
1. > chmod +x sb_gps/scripts/gps_receiver.py 
2. Make sure that the shebang is pointing to the correct directory on your computer 
3. >rosrun sb_gps gps_receiver.py 
The node will start streaming data
