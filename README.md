# pendulum-opencv
Script to indentify a simple pendulum's movement (velocity, acceleration and so on).
I'm writing a final work in Physics Teaching and the goal is to create an Augmented Reality that allows students and visitors to "see" physics vectors simultaneously with an experiment. OpenCV has good features for that and later I can wrap it in an Android SDK.
What I got so far:
0. Installation in Windows (yay!);
1. Capture the frames (yay again) from the video called "20190829_150101.mp4";
2. How to detect movement: background subtraction looks better than absdiff;
3. How to track movement: meanshift has simple and good features to that - I can refine this later;
4. How to identify sphere's center: moments can normalize a polygon's position by the area (m01/m00).
