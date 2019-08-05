# psr_smach

## Intro

This is a ROS package for state machine in PSR.  The package is recommended to be installed on an individual PC instead of BeagleBone Blue.

## Useful Reference

- actionlib:

http://wiki.ros.org/actionlib

http://wiki.ros.org/actionlib/Tutorials

- smach:

http://wiki.ros.org/smach/Tutorials

## Tips on ActionLib

- When constructing server and client, make sure they referres to the same server name.

- The return type of `acceptNewGoal()` is boost::shared_ptr<const psr_smach::PSRGoal_<std::allocator<void> > >.

- `acceptNewGoal()` can be used for only one time whenever goal callback function is called.

