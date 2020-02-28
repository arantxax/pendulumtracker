/*
@file Pendulum-v23.cpp
@brief An attempt to recognize and to track a real simple pendulum movement, and then draw physics vectors (e.g. velocity).
@author arantxax
@date Feb 28, 2020
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
  int N = 0;

  //position
  vector<double> X(4), Y(4);
  double cx = 0.0, cy = 0.0;
  double dx = 0.0, dy = 0.0;

  //velocity
  double vx = 0.0;
  double vy = 0.0;
  Vec2d v {0.0, 0.0};
  Vec2d v_unit_tangencial {0.0, 0.0};
  Vec2d v_unit_radial {0.0, 0.0};
  double dot_tangencial = 0.0;
  double dot_radial = 0.0;

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

  //cartesian line and QMM method
  double m = 0.0; //slope lines r s t
  double b = 0.0; //intercept lines r s t
  double x0 = 0.0, y0 = 0.0;
  double m_sum = 0.0;
  double b_sum = 0.0;
  double m2_sum = 0.0;
  double mb_sum = 0.0;
  double m_sum2 = 0.0;
  Point P0, P1, Q;

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
	  
	  //Get the centroid 1th-moments/0th-moments
	  Moments M=moments(canny_output,true);
  	            
	  //Get centroid considering the ROI's coordinates
	  if(M.m00!=0){
	    cx = M.m10/M.m00 + xmin;
	    cy  = M.m01/M.m00 + ymin;
	    center = Point(cx,cy);
	  }
	  else{
	    center = Point(0,0);//avoid further division by zero
	  }

	  ///Begin Physics
	  //time elapsed: dt > 0 (avoid division by zero)
	  end = getTickCount();
	  dt = (double)(end - start)*1.0f / getTickFrequency();
	  cout << "\n" << endl;
	  cout << "time elapsed: " << format("%9.2f s", dt) << endl;
	  cout << "current i is: " << i << endl;

	  if (i < 4) {
	  	///Position
	  	cout << "Storing the first five displacement's data..." << endl;
	  	X[i] = cx;
	  	Y[i] = cy;
	  }

	  if (i >= 4){
	  	cout << "Updating last position' data..." << endl;
	  	X[4] = cx;
	  	Y[4] = cy;
	  	cout << "------------------------------------------" << endl;
	  	cout << "Now the X vector is: " << endl;
	  	for (int j=0; j<5; j++){
	  		cout << X[j] << "\t";
	  	}
	  	
	  	cout << endl;
	  	cout << "--------------------------------------------------" << endl;
	  	cout << "Enough data to calculate velocity and acceleration" << endl;	  	
	  	///Position
	  	dx = X[3]-X[1];
		dy = Y[3]-Y[1];		  		
		cout << "dx = X[3]-X[1] = " << X[3] << " - " << X[1] << "= " << dx << endl;

		///Velocity
		//cout << "Updating velocity ..." << endl;
		vx = dx/(2*dt);
		vy = dy/(2*dt);
		v = {vx, vy};
		cout << "vx = " << dx << "/(2*" << dt << ") = " << vx << endl;

		///Acceleration
		//acceleration in both x and y axis using second derivative with
	  	//five position's data and middle point method
	  	ax = (-X[0] + 16*X[1] - 30*X[2] + 16*X[3] - X[4])/(12*dt*dt);
	  	ay = (-Y[0] + 16*Y[1] - 30*Y[2] + 16*Y[3] - Y[4])/(12*dt*dt);
	  	a = {ax, ay};
	  	cout << "ax = (-X[0] + 16*X[1] - 30*X[2] + 16*X[3] - X[4])/(12*dt*dt)" << endl;
  		cout << "ax = (-" << X[0] << " + 16*" << X[1] << " - 30* " << X[2]
  		<< "16*" << X[3] << " - " << X[4] << "(12*" << dt*dt << ")" << endl;
  		cout << "ax = " << ax << endl;

  		//Search for amplitude
  	  	if (cx < Ax_min){
  	  		Ax_min = cx;
  	  	}
  	  	if (cx > Ax_max){
  	  		Ax_max = cx;
  	  	}

  	  	//Exploring vectors' directions...
  	  	//velocity's cartesian line and its perpendicular
	  	m = - dx / dy;    //slope (perpendicular line)
	  	b = cy - m * cx;  //intercept

	  	//proceed MMQ method in order to find the common point betwenn all lines
	  	if((i+1) % 5 == 0){ //i+1 remember: i is initialized as zero
	  		cout << "Totally brand new vector!" << endl;
	  		cout << "Let's sum up all cartesian elements" << endl;
	  		m_sum += m;
	  		b_sum += b;
	  		m2_sum += m*m;
	  		mb_sum += m*b;
	  		N = i+1;
	  	}
	  	m_sum2 += m_sum*m_sum;
	  	x0 = (N*mb_sum     - b_sum*m_sum)  / (N*m2_sum - m_sum2);
	  	y0 = (m2_sum*b_sum - mb_sum*m_sum) / (N*m2_sum - m_sum2);
	  	
	  	//tangencial vector
	  	v_unit_tangencial = v/norm(v, NORM_L2, noArray());
		  
	  	//radial vector
	  	//swap radial and tangencial components with some conditions

	  	//preferencial direction: +y because the radial vector \
	  	points to the radius' increase
	  	v_unit_radial[1] = abs(v_unit_tangencial[0]); 

	  	if ((v_unit_tangencial[0]/v_unit_tangencial[1]) > 0){
			v_unit_radial[0] = -abs(v_unit_tangencial[1]);
	  	}
	  	if ((v_unit_tangencial[0]/v_unit_tangencial[1]) < 0){
			v_unit_radial[0] = abs(v_unit_tangencial[1]);
	  	}
	  	//cout << "v_unit_tangencial: " << v_unit_tangencial << endl;
	  	//cout << "v_unit_radial:" << v_unit_radial << endl;
	  	if (v_unit_tangencial.dot(v_unit_radial)==0){
			//cout << "They are perpendicular." << endl;
	 	 }

	  	//Acceleration projection in radial and transversal directions
	  	//cout << "-----------------------------------------------------" << endl;
  	  	//cout << "Dot product using a.dot(v)" << endl;
  	  	dot_tangencial = a.dot(v_unit_tangencial);
  	  	dot_radial = a.dot(v_unit_radial);
  	  	/*cout << "dot_tangencial" << "\t" << "dot_radial" << endl;
  	  	cout << dot_tangencial << "\t" << dot_radial << endl;
  	  	cout << "\n" << endl;
  	  	cout << "Angle between y axis and radial axis: ";
  	  	cout << acos (v_unit_radial[1])*180.0 / PI << endl;
  	
  	  	cout << "-----------------------------------------------------" << endl;
  	  	cout << "Project acceleration onto velocity tangencial vector:" << endl;*/
  	  	a_tangencial = dot_tangencial*v_unit_tangencial;
  	  	//cout << "a_tangencial:" << "\t" << a_tangencial << endl;
  	
  	  	//cout << "Project acceleration onto velocity radial vector:" << endl;
  	  	a_radial = dot_radial*v_unit_radial;
  	  	/*cout << "a_radial:" << "\t" << a_radial << endl;
  	  	//cout << "-----------------------------------------------------" << endl;
  	  	//cout << "\n" << endl;

  	  	//cout << "Gravity is the sum of tangencial and radial y component: ";*/
  	  	g = {0.0, abs(a_tangencial[1]+a_radial[1])};
  	  	//cout << "Tension force dwells in radial axis, but in opposite orientation: ";
  	  	T = v_unit_radial*(-abs(g.dot(v_unit_radial))-abs(a.dot(v_unit_radial)));

  	  	cout << "Vector is full: Proceed ring effect to the left" << endl;
	  	cout << "Before:" << "\t" << "{x0 x1 x2 x3 x4}" << endl;
	  	cout << "After:" << "\t" <<  "{x1 x2 x3 x4 ..}" << endl;
	  	cout << "-----------------------------------------------------" << endl;
	  	cout << "Before: " << endl;
	  	for (int j=0; j<5; j++){
	  		cout << X[j] << "\t";
	  	}
	  	
	  	cout << endl;

	  	for (int n=0; n<4; n++){
	  		X[n]=X[n+1];
	  		Y[n]=Y[n+1];
	  	}
	  	X[4] = 0.0;

	  	cout << "After: " << endl;
	  	for (int j=0; j<5; j++){
	  		cout << X[j] << "\t";
	  	}
	  	cout << "\n" << endl;
  	  }
		  	
  	  ///Drawing vectors
	  //show the image with a point mark at the centroid
	  circle(frame,center,5,Scalar(0,240,255),-1); //BGR
  	
  	  //show the image with the displacement vector in x and y axis
  	  /*r_x = Point (cx + dx , cy     );
  	  r_y = Point (cx      , cy + dy);
  	  arrowedLine (frame, center, r_x, Scalar(0, 255, 0), 3, 8);
  	  arrowedLine (frame, center, r_y, Scalar(0, 0, 255), 3, 8);*/

  	  //show the image with the velocity vector in x and y axis
  	  //v_x = Point (cx + 0.10*vx, cy);
  	  //v_y = Point (cx,      cy + vy);
  	  //arrowedLine (frame, center, v_x, Scalar(0, 255, 0), 3, 8);
  	  //arrowedLine (frame, center, v_y, Scalar(0, 255, 0), 3, 8);

  	  //show the image with tangencial and radial vectors (scaled)
  	  //a_t = Point (100*v_unit_tangencial);
  	  //a_r = Point (100*v_unit_radial);
  	  //arrowedLine (frame, center, center + a_t, Scalar(0, 0, 255), 3, 8);
  	  //arrowedLine (frame, center, center + a_r, Scalar(255, 0, 0), 3, 8);

  	  //Draw the cartesian lines
	  //arrowedLine(frame, center, Q, Scalar (0, 240, 255), 2, 8);

  	  /*arrowedLine (frame, center, Point (cx, cy + 0.01*g[1]), Scalar(255, 255, 255), 3, 8);
  	  arrowedLine (frame, center, Point (cx + 0.01*T[0], cy + 0.01*T[1]), Scalar(0, 0, 0), 3, 8);*/
  	  imshow("Vectors: G - vx | R - v_tangencial | B - v_radial", frame);

  	  //write information to a file
  	  outputfile << i << "\t" << N << "\t" << x0 << "\t" << y0 << "\n";
  	  video.write(frame);

  	  i++;
  	  start = getTickCount(); //reset cronometer

	  int keyboard = waitKey(30); 
	  if (keyboard == 'q' || keyboard == 27) 
	  break;
	}
	outputfile.close();
} 