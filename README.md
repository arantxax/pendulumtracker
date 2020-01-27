# pendulum-opencv
Script to detect and track a real simple pendulum's motion in order to draw its main physical vectors (velocity, acceleration and so on).
I'm writing a final work in Physics Teaching and the goal is to create an Augmented Reality that allows students and visitors to "see" physics vectors simultaneously with an experiment conducted in a science museum. OpenCV has good features for that and later I can wrap it in an Android SDK.
What I got so far:

0. Installation in Windows (Update: I gave up - Linux is far straightforward);
1. Capture and loop frames from the file called "20190829_150101.mp4" (aftwerwards channel it to the default camera);
2. Isolate what's moving: Background Subtraction method has an update better than Absdiff (which also deletes the stationary ball at its maximal amplitude);
3. Establish Region of Interest ROI: Object Detection can avoid taking visitors' or any other motion into account (not initialized yet);
3.1 Based on format (Circle);
3.2 Based on color (Green);
4. Track motion: meanShift is based on pixel's density displacement and its strictly dependent on the ROI;
5. Find the physical center of mass: function "Moments" calculates the centroid, which need to be correctly positioned from ROI to the actual frame;
6. Record time elapsed: using "getTickCount()" worked better than "time_t" function - remember: the shorter the time interval, the better.
7. Calculate and expose displacement: in order to have a substantial displacement - pixel quite apart from each other - store three positions in a vector and ignore the middle one (maybe there's a better solution using a fork method - I cannot say);
8. Calculate and expose velocity:  (i) calculate backwards, forward or midpoint velocity based on the future and on the past; (ii) how to expose it currently?
RESULTS SO FAR - VELOCITY IS UPDATED EVERY TWO FRAMES
counter = 0	
[x]     92	
[v]     0	

counter = 1;	
[x]      84	
[v]      0	

counter = 2;	   
[x]      79	
[v]      -152	

counter = 0;	
[x]      72	
[v]      -152

counter = 1;     
[x]      69
[v]      -152

counter = 2;
[x]       68
[v]      -66
