/*
@file Pendulum-v17.cpp
@brief An attempt to recognize and to track a real simple pendulum movement, and then draw physics vectors (e.g. velocity).
@author arantxax
@date Feb 17, 2020
*/
 
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/imgproc/imgproc.hpp" 
#include "opencv2/video.hpp" 
#include "opencv2/videoio.hpp" 
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

#include <iostream> 
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <cmath>
#include <fstream>

using namespace cv; 
using namespace std;

#define PI 3.14159265

int main(){ 
  //Load source video
  VideoCapture capture("20190829_150101.mp4");// when the input is a camera, set zero intead of string 
  if (!capture.isOpened())// check if we succeeded 
    { 
      cout <<"Error opening video strem or file"<< endl; 
      return -1; 
    }

  //Background Subtraction Method 
  Ptr<BackgroundSubtractor> model; 
  model = createBackgroundSubtractorMOG2(); 
  bool doUpdateModel = true; 
  bool doSmoothMask = false;
  
  //Declare variables to store the frames 
  Mat frame; Mat roi; 
  Mat hsv_roi; Mat mask;
  
  //Take first frame of the video
  capture >> frame;

  //Initialize Videowriter
  int frame_width = static_cast<int>(capture.get(CAP_PROP_FRAME_WIDTH));
  int frame_height = static_cast<int>(capture.get(CAP_PROP_FRAME_HEIGHT));
  Size frame_size(frame_width, frame_height);
  VideoWriter video("./output.avi", VideoWriter::fourcc('M','J','P','G'), 10, frame_size, true);

  ///Initialize export to text file
  ofstream outputfile;
  outputfile.open ("outputfile.dat");
    
  //Setup initial location of windows for tracking 
  Rect track_window(50, 300, 50, 50); // simply hardcoded the value 

  //set up the ROI for Tracking 
  roi = frame(track_window);
  cvtColor(roi, hsv_roi, COLOR_BGR2HSV); 
  inRange(hsv_roi, Scalar(0, 60, 32), Scalar(180, 255, 255), mask);

  //set up ROI histogram
  float range_[] = { 0, 180 }; 
  const float* range[] = {range_}; 
  Mat roi_hist; 
  int histSize[] = { 180 }; 
  int channels[] = { 0 }; 
  calcHist(&hsv_roi, 1, channels, mask, roi_hist, 1, histSize, range); 
  normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX); 
  //Setup the termination criteria, either 100 iteraction or move by at least 1 pt
  TermCriteria term_crit(TermCriteria::EPS | TermCriteria::COUNT, 100, 1.0); 

  ///Declare Physics' variables
  //time elapsing
  //time_t start, end; time (&start);
  int64 start, end;
  start = getTickCount(); //just to begin
  double dt = 0.0;
  int i = 0;
  int k = 0;
  int j = 0;

  //position
  Vec3d X,Y;
  double cx = 0.0, cy = 0.0;
  double dx = 0.0, dy = 0.0;
  double dot_tangencial = 0.0;
  double dot_radial = 0.0;

  //velocity
  double vx = 0.0;
  double vy = 0.0;
  Vec2d v {0.0, 0.0};
  Vec2d v_unit_tangencial {0.0, 0.0};
  Vec2d v_unit_radial {0.0, 0.0};

  //aceleration
  double ax = 0.0;
  double ay = 0.0;
  Vec2d a {0.0, 0.0};
  Vec2d a_radial {0.0, 0.0};
  Vec2d a_tangencial {0.0, 0.0};
  Vec2d g {0.0, 0.0};
  Vec2d T {0.0, 0.0};

  //amplitude
  double Ax_max = 0.0, Ax_min = 999.0; //+A and -A
  double Ay_max = 0.0; //Rope's lenght
  double vx_max = 0.0, vy_max = 0.0;
  double ax_max = 0.0, ay_max = 0.0;

  //Drawing vectors
  Point center;
  Point r_x, r_y;
  Point v_x, v_y;
  Point a_t, a_r;
  
  ///Loop over all frames 
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
	  
	  //ROI updated
	  roi = foreground(track_window);   

	  //Draw it on foreground 
	  rectangle(foreground, track_window, Scalar(0,240,255), 2); 
	  //get track_window position
	  double xmin = track_window.x;
	  double ymin = track_window.y;
	  double w = track_window.width;
	  double h = track_window.height;
	  
	  ///Start Moments
	  Mat canny_output;
	  int thresh = 10;
	  RNG rng(12345);
	  	
	  //Convert image to gray and blur it (again)
	  cvtColor(roi,canny_output,COLOR_BGR2GRAY);
	  blur(canny_output, canny_output,Size(3,3));
	  	
	  //Detect edges using canny 
	  Canny(roi, canny_output, thresh, thresh*2, 3 );
	  	
	  //Find contours 
	  vector<vector<Point>> contours;
	  vector<Vec4i> hierarchy;
	  findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE,Point(0,0));
	  /*vector<Point2f>centers( contours.size() );
	  vector<vector<Point> > contours_poly( contours.size() );
	  vector<float>radius( contours.size() );
	  for( size_t i = 0; i < contours.size(); i++ )
	  {
	  	approxPolyDP( contours[i], contours_poly[i], 3, true );
	  	minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
	  	cout << arcLength(contours[i], true);
	  }
	  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	  for( size_t i = 0; i< contours.size(); i++ )
	  {
	  	Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
	  	drawContours( drawing, contours_poly, (int)i, color );
	  	circle( drawing, centers[i], (int)radius[i], color, 2 );
	  }*/
	  
	  //Get the centroid 1th-moments/0th-moments
	  Moments m=moments(canny_output,true);
  	            
	  //Get centroid considering the ROI's coordinates
	  if(m.m00!=0){
	    cx = m.m10/m.m00 + xmin;
	    cy  = m.m01/m.m00 + ymin;
	    center = Point(cx,cy);
	  }
	  else{
	    center = Point(0,0);//avoid further division by zero
	  }

	  ///Begin Physics
	  //store three displacement's data and time elapsed for further use
	  X[i] = cx;
	  Y[i] = cy;
	  //cout << "counter: " << i  << endl;
	  /*cout << "current X[" << i << "]: " << X[i] << 
	  " and " << "current Y[" << i << "]: " << Y[i] << endl;*/
	  	  
	  if(i==2){
	  	/*cout << "\nENOUGH DATA - PROCEED CALCULATIONS" << endl;*/
		
		//time elapsed: dt > 0 (avoid division by zero)
	  	end = getTickCount();
	  	dt = (double)(end - start)*1.0f / getTickFrequency();
	  	cout << "time elapsed: " << format("%9.2f s", dt) << "\n";

	  	//displacement - ignore middle term
	  	dx = X[2] - X[0];
	  	dy = Y[2] - Y[0];
	  	/*cout << "dx = " << X[2] << "-" << X[0] << " = " << dx << endl;
	  	cout << "dy = " << Y[2] << "-" << Y[0] << " = " << dy << endl;*/
	  	
	  	//cout << "Updating velocity ..." << endl;
	  	vx = dx/(2*dt);
	  	vy = dy/(2*dt);
	  	v = {vx, vy};

	  	//acceleration in both x and y axis
	  	ax = (X[2] - 2*X[1] + X[0])/(dt*dt);
	  	ay = (Y[2] - 2*Y[1] + Y[0])/(dt*dt);
	  	a = {ax, ay};
	  	cout << "Acceleration in x and y axis is: " << a << endl;

	  	///Search for amplitudes
	  	//position
	  	if (cx < Ax_min){
	  		Ax_min = cx;
	  	}
	  	if (cx > Ax_max){
	  		Ax_max = cx;
	  	}
	  	//cout << "+Ax = " << Ax_max << " -Ax = " << Ax_min << endl;
	  	
	  	//velocity
	  	if (vx > vx_max){
	  		vx_max = vx;
	  	}
	  	if (vy > vy_max){
	  		vy_max = vy;
	  	}

	  	///exploring vectors' directions...
	  	//tangencial vector
	  	v_unit_tangencial = v/norm(v, NORM_L2, noArray());
	  
	  	//radial vector
	  	//swap radial and tangencial components with some conditions

	  	//preferencial direction: -y because the ceiling points\
	  	to -y in frame coordinates
	  	v_unit_radial[1] = abs(v_unit_tangencial[0]); 

	  	if ((v_unit_tangencial[0]/v_unit_tangencial[1]) > 0){
	  		v_unit_radial[0] = -abs(v_unit_tangencial[1]);
	  	}
	  	if ((v_unit_tangencial[0]/v_unit_tangencial[1]) < 0){
	  		v_unit_radial[0] = abs(v_unit_tangencial[1]);
	  	}
		cout << "v_unit_tangencial: " << v_unit_tangencial << endl;
		cout << "v_unit_radial:" << v_unit_radial << endl;
		if (v_unit_tangencial.dot(v_unit_radial)==0){
			cout << "They are perpendicular." << endl;
		}

	  	//Acceleration projection in radial and transversal directions
	  	cout << "-----------------------------------------------------" << endl;
	  	cout << "Dot product using a.dot(v)" << endl;
	  	dot_tangencial = a.dot(v_unit_tangencial);
	  	dot_radial = a.dot(v_unit_radial);
	  	cout << "dot_tangencial" << "\t" << "dot_radial" << endl;
	  	cout << dot_tangencial << "\t" << dot_radial << endl;
	  	cout << "\n" << endl;
	  	cout << "Angle between y axis and radial axis: ";
	  	cout << acos (v_unit_radial[1])*180.0 / PI << endl;
	  	
	  	cout << "-----------------------------------------------------" << endl;
	  	cout << "Project acceleration onto velocity tangencial vector:" << endl;
	  	a_tangencial = dot_tangencial*v_unit_tangencial;
	  	cout << "a_tangencial:" << "\t" << a_tangencial << endl;
	  	
	  	cout << "Project acceleration onto velocity radial vector:" << endl;
	  	a_radial = dot_radial*v_unit_radial;
	  	cout << "a_radial:" << "\t" << a_radial << endl;
	  	cout << "-----------------------------------------------------" << endl;
	  	cout << "\n" << endl;

	  	cout << "Gravity is the sum of tangencial and radial y component: ";
	  	g = {0.0, abs(a_tangencial[1]+a_radial[1])};
	  	cout << g << endl;

	  	cout << "Tension force dwells in radial axis, but in opposite orientation: ";
	  	T = v_unit_radial*(-abs(g.dot(v_unit_radial))-abs(a.dot(v_unit_radial)));
	  	cout << T << endl;
	  	cout << "\n" << endl;
	  	cout << "-----------------------------------------------------" << endl;

	  	///Drawing vectors
		//show the image with a point mark at the centroid
		circle(foreground,center,5,Scalar(0,240,255),-1); //BGR
	  	
	  	//show the image with the displacement vector in x and y axis
	  	/*r_x = Point (cx + dx , cy     );
	  	r_y = Point (cx      , cy + dy);
	  	arrowedLine (frame, center, r_x, Scalar(0, 255, 0), 3, 8);
	  	arrowedLine (frame, center, r_y, Scalar(0, 0, 255), 3, 8);*/

	  	//show the image with the velocity vector in x and y axis
	  	v_x = Point (cx + vx, cy);
	  	a_t = Point (100*v_unit_tangencial);
	  	a_r = Point (100*v_unit_radial);
	  	arrowedLine (frame, center, v_x, Scalar(0, 255, 0), 3, 8);
	  	arrowedLine (frame, center, center + a_t, Scalar(0, 0, 255), 3, 8);
	  	arrowedLine (frame, center, center + a_r, Scalar(255, 0, 0), 3, 8);
	  	arrowedLine (frame, center, Point (cx, cy + 0.01*g[1]), Scalar(255, 255, 255), 3, 8);
	  	arrowedLine (frame, center, Point (cx + 0.01*T[0], cy + 0.01*T[1]), Scalar(0, 0, 0), 3, 8);

	  	imshow("Vectors: G - vx | R - a_tangencial | B - a_radial | W - g | K - T", frame);
	  	//imshow("Radius", drawing);

	  	//write information to a file
	  	outputfile << k << "\t" << a_tangencial[0] << "\t" << a_tangencial[1] 
	  	<< "\t" << a_radial[0] << "\t" << a_radial[1] << "\n";
	  	k++;
	  	video.write(frame);


	  	//Backup previous information
	  	X[0]=X[2];
	  	/*cout << "Updated X[0]= " << X[0] << 
	  	" and " << "Updated X[0]= " << Y[0];
	  	cout << "\n";*/
	  	i=1;
	  	start = getTickCount(); //reset cronometer

	  	/// Create Window 
	  	//const char* moments_window = "Moments"; 
	  	//namedWindow( moments_window ); 
	  	//imshow("moments_window", canny_output ); 
	  }

	  else{
	  	i++;
	  	/*cout << "----------------------------------" << endl;
	  	cout << "NOT ENOUGH DATA - INCREASE COUNTER" << endl;
	  	cout << "Current velocities: " << endl;
	  	cout << "vx = " << vx << " and " << "vy = " << vy;
	  	cout << "\n";
	  	cout << "----------------------------------" << endl;*/
	  }
	 
	 int keyboard = waitKey(30); 
	 if (keyboard == 'q' || keyboard == 27) 
	 break;
	}
	outputfile.close();
} 