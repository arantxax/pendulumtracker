/*
@file Pendulum-v6.cpp
@brief An attempt to recognize and to track a real simple pendulum movement, and then draw physics vectors (e.g. velocity).
@author arantxax
@date Jan 27, 2020
*/
 
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/imgproc/imgproc.hpp" 
#include "opencv2/video.hpp" 
#include "opencv2/videoio.hpp" 
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

#include <iostream> 
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <cmath>

using namespace cv; 
using namespace std; 

int main(){ 
  //Load source video
  VideoCapture capture("20190829_150101.mp4");// when the input is a camera, set zero intead of string 
  if (!capture.isOpened())// check if we succeeded 
    { 
      cout <<"Error opening video strem or file"<< endl; 
      return -1; 
    }

  //Background Subtraction Method 
  String method = "mog2"; 
  Ptr<BackgroundSubtractor> model; 
  if (method == "knn") 
    model = createBackgroundSubtractorKNN(); 
  else if (method == "mog2") 
    model = createBackgroundSubtractorMOG2(); 
  if (!model) 
    { 
      cout <<"Can not create background model using provided method: '"<< method <<"'"<< endl; 
      return 3; 
    } 
  
  bool doUpdateModel = true; 
  bool doSmoothMask = false; 
  
  //Declare variables to store the frames 
  Mat frame; Mat roi; 
  Mat hsv_roi; Mat mask; 
  Mat dframe;

  //Declare variables to time elapsing and position
  //time_t start, end; // \n time (&start);
  int64 start, end;
  start = getTickCount(); //just to begin
  bool initialsecondsgot = false;
  double dt=0;

  //Physics position
  Vec3d X,Y;
  double dx=0, dy=0;
  int i = 0;

  //velocity
  double v_x = 0;
  double v_y = 0;
    
  //Take first frame of the video
  capture >> frame;

  /*//Initialize Videowriter
  int frame_width = static_cast<int>(capture.get(CAP_PROP_FRAME_WIDTH)); //get the width of frames of the video
  int frame_height = static_cast<int>(capture.get(CAP_PROP_FRAME_HEIGHT)); //get the height of frames of the video
  Size frame_size(frame_width, frame_height);
  VideoWriter video("./16foreground-para-hsv.avi", VideoWriter::fourcc('M','J','P','G'), 30, frame_size, true);*/
    
  //Setup initial location of windows for tracking 
  Rect track_window(50, 300, 50, 50); // simply hardcoded the value 

  //set up the ROI for Tracking 
  roi = frame(track_window);
  cvtColor(roi, hsv_roi, COLOR_BGR2HSV); 
  inRange(hsv_roi, Scalar(0, 60, 32), Scalar(180, 255, 255), mask);
    
  float range_[] = { 0, 180 }; 
  const float* range[] = {range_}; 
  Mat roi_hist; 
  int histSize[] = { 180 }; 
  int channels[] = { 0 }; 
  calcHist(&hsv_roi, 1, channels, mask, roi_hist, 1, histSize, range); 
  normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX); 
  //Setup the termination criteria, either 100 iteraction or move by at least 1 pt
  TermCriteria term_crit(TermCriteria::EPS | TermCriteria::COUNT, 100, 1.0); 

  
  //Loop over all frames 
  for (;;)
    { // prepare input frame
      Mat hsv, dst; 
      Mat frame, foregroundMask, foreground, background; 
      capture >> frame;

      //Conditions to continue
      if (frame.empty()){//end of capture
        break;
    	}

	  //resizing: zooming
	  const Size scaledSize(640, 640 * frame.rows / frame.cols);
	  resize(frame, frame, scaledSize, 0, 0, INTER_LINEAR);

	  // pass the frame to background model 
	  model->apply(frame, foregroundMask, doUpdateModel ? -1 : 0); 
	  		
	  // show foreground image and mask (with optional smoothing) 
	  if (doSmoothMask) 
	  {
	  GaussianBlur(foregroundMask, foregroundMask, Size(11, 11), 3.5, 3.5);
	  threshold(foregroundMask, foregroundMask, 10, 255, THRESH_BINARY);
	  } 

	  if (foreground.empty()){
	  foreground.create(scaledSize, frame.type());
	  } 
	  foreground = Scalar::all(0); 
	  frame.copyTo(foreground, foregroundMask);
	  	
	  // show background image 
	  model->getBackgroundImage(background); 
	  if (!background.empty()) {
	    //imshow("mean background image", background);
	  }
	  	  
	  cvtColor(foreground, hsv, COLOR_BGR2HSV);
	  calcBackProject(&hsv, 1, channels, roi_hist, dst, range); 
	  
	  //apply meanshift to get the new location 
	  meanShift(dst, track_window, term_crit); //Finds an object on a back projection image
	  
	  // get track_window position
	  double xmin = track_window.x;
	  double ymin = track_window.y;
	  double w = track_window.width;
	  double h = track_window.height;
	  //cout << "xmin:" << xmin << endl;

	  //ROI updated
	  roi = foreground(track_window);   

	  // Draw it on image 
	  rectangle(foreground, track_window, Scalar(0,240,255), 2); 
	  
	  ///Start Moments
	  Mat canny_output;
	  int thresh = 10;
	  	
	  //Convert image to gray and blur it (again)
	  cvtColor(roi,canny_output,COLOR_BGR2GRAY);
	  blur(canny_output, canny_output,Size(3,3));
	  	
	  /// Detect edges using canny 
	  Canny(roi, canny_output, thresh, thresh*2, 3 );
	  	
	  /// Find contours 
	  vector<vector<Point>> contours;
	  vector<Vec4i> hierarchy;
	  findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE,Point(0,0));
	  	      
	  // Get the centroid 1th-moments/0th-moments
	  Moments m=moments(canny_output,true);
	  //cout << "canny_output.x"<< canny_output.x << "\n";
	  //cout << "canny_output.y" << canny_output.y << "\n";
	  	            
	  //Get centroid considering the ROI's coordinates
	  double cx, cy, x, y;
	  Point p;
	  	
	  if(m.m00!=0){
	    cx = m.m10/m.m00;
	    cy  = m.m01/m.m00;
	    x = cx + xmin;
	    y =  cy + ymin;
	    p = Point(x,y);
	    //p((double)(m.m10 / m.m00), (double)(m.m01 / m.m00) );
	  }
	  else{
	    cout << "\n";
	    p = Point(0,0);//avoid division by zero
	  }

	  //show the image with a point mark at the centroid
	  circle(foreground,p,5,Scalar(0,240,255),-1); //BGR
	  cout << "\n";
	  cout << "coordinates (x,y)" << p <<endl;

	  //Begin Physics
	  //store three displacement's data and time elapsed for further use
	  X[i] = x;
	  Y[i] = y;
	  cout << "---------------------------------------------" << endl;
	  cout << "counter: " << i  << endl;
	  cout << "X[i]: " << X[i] << " and " << "Y[i]: " << Y[i] << endl;
	  	  
	  //displacement and time elapsed
	  if(i==2){
	  	dx = X[2] - X[0];
	  	dy = Y[2] - Y[0];
	  	//cout << "dx = " << X[2] << "-" << X[0] << " = " << dx << endl;
	  	//cout << "dy = " << Y[2] << "-" << Y[0] << " = " << dy << endl;
	  	i=0;
	  	end = getTickCount();
	  	dt = (double)(end - start) * 1.0f / getTickFrequency();
	  	initialsecondsgot = true;	  	
	  	cout << "time elapsed: " << format("%9.2f s", dt) << endl;
	  	cout << "\n";
	  	cout << "Updating velocity ..." << endl;
	  	start = getTickCount(); //reset cronometer
	  }
	  else{
	  	i++;
	  	cout << "Current velocities: " << endl;
	  	cout << "vx = " << v_x << " and " << "vy = " << v_y << endl;	  
	  }
	  if (initialsecondsgot){
	  	v_x = dx/dt;
	  	v_y = dy/dt;
	  	cout << "vx = " << v_x << " and " << "vy = " << v_y << endl;

	  	//Drawing vectors
	  	Point displacement;
	  	Point velocity;

	  	displacement = Point (dx, dy);
	  	velocity = Point (v_x, v_y);
	  	arrowedLine (foreground, p, p+displacement, Scalar(0, 255, 0), 5, 8);
	  	arrowedLine (foreground, p, p+velocity, Scalar(0, 0, 255), 2, 8);
	  	imshow("Vectors: B - displacement| R - velocity", foreground);

	  	//Backup previous information
	  	X[0]=X[2];
	  	       
	  	/// Create Window 
	  	//const char* moments_window = "Moments"; 
	  	//namedWindow( moments_window ); 
	  	//imshow("moments_window", canny_output ); 
	  	
	  	int keyboard = waitKey(30); 
	  	if (keyboard == 'q' || keyboard == 27) 
	  		break;
	  }
    }
} 