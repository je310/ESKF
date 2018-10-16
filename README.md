# ESKF
An implementation of an Error State Kalman Filter (ESKF) 

The ambition of this repository is to make an estimator that can take accelerometer/gyro readings and integrate them into a 6DOF pose.
This will also be corrected by a motion capture system (or any absolute position input.). 

Where possible notation and concepts will be taken from the notes provided by Joan Sola. Found at http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf 

This implementation is intended to run on an embedded device. In my case an STM32 development board. 


# Building 
mkdir build 
cd build 
cmake .. 
make -j8
