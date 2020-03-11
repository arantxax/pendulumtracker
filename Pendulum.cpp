/*
@file Pendulum-v28.cpp
@brief An attempt to recognize and to track a real simple pendulum movement, and then draw physics vectors (e.g. velocity).
@author arantxax
@date Mar 11, 2020
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
  //![Load source video]
  VideoCapture capture("20190829_150101.mp4");// when the input is a camera, set zero intead of string 
  if (!capture.isOpened())// check if we succeeded 
    { 
      cout <<"Error opening video strem or file"<< endl; 
      return -1; 
    }
  //![Load source video]

  //![Background Subtraction Method]
  Ptr<BackgroundSubtractor> model; 
  model = createBackgroundSubtractorMOG2(); 
  bool doUpdateModel = true; 
  bool doSmoothMask = true;
  //![Background Subtraction Method]
  
  //![Declare variables to store the frames]
  Mat frame, gray, hsv, roi; 
  Mat hsv_roi, mask, dst;
  //![Declare variables to store the frames]
  
  //![Take first frame of the video]
  capture >> frame;
  //![Take first frame of the video]

  //![Initialize Videowriter]
  int frame_width = static_cast<int>(capture.get(CAP_PROP_FRAME_WIDTH));
  int frame_height = static_cast<int>(capture.get(CAP_PROP_FRAME_HEIGHT));
  Size frame_size(frame_width, frame_height);
  VideoWriter video("./output.avi", VideoWriter::fourcc('M','J','P','G'), 10, frame_size, true);
  //![Initialize Videowriter]

  //![Initialize export to data file]
  ofstream outputfile;
  outputfile.open ("outputfile.dat");
  //![Initialize export to data file]

  //![Declare vectors and points to draw]
  Point center;
  Vec2d Center;
  Point r_x, r_y;
  Point v_x, v_y;
  Point a_t, a_r;
  Point P0;
  Vec2d P_0;
  //![Declare vectors and points to draw]

  //![Detect circles]
  Mat canny_output;
  int thresh = 100;
  int id = 0;
  double area = 0.0;
  double max_area = 0.0;

  //Convert image to hsv and blur it
  cvtColor(frame, hsv, COLOR_BGR2HSV);
  blur(hsv, hsv, Size(3,3));
  inRange(hsv, Scalar(40, 50, 10), Scalar(80, 255, 255), mask);

  //Detect edges using canny 
  Canny(mask, canny_output, thresh, thresh*2);

  //Find contours 
  vector<vector<Point>> contours;
  findContours(canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE,Point(0,0));	

  //Circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Point2f> centers( contours.size() );
  vector<float> radius( contours.size() );

  for( size_t i = 0; i < contours.size(); i++ )
  {
    approxPolyDP( contours[i], contours_poly[i], 3, true );
    minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
    area = contourArea(contours[i], false);
    id = area > max_area ? i : id;
    max_area = max(max_area, area );
  }

  drawContours(canny_output, contours_poly, id, Scalar (255, 255, 255));
  circle(canny_output, centers[id], (int)radius[id], Scalar (255, 255, 255), 2 );
  imshow("canny_output", canny_output);
  //![Detect circles]

  //![Setup ROI for tracking]
  //initial window's location
  center = centers[id];
  Point shift = Point (20, 20);
  Rect track_window (center - shift, center + shift);

  //set up the ROI for Tracking 
  roi = frame(track_window);
  cvtColor(roi, hsv_roi, COLOR_BGR2HSV); 
  inRange(hsv_roi, Scalar(0, 60, 32), Scalar(180, 255, 255), dst); //binarized

  //set up ROI histogram
  float range_[] = { 70, 150 }; 
  const float* range[] = {range_}; 
  Mat roi_hist; 
  int histSize[] = { 180 }; 
  int channels[] = { 0 }; 
  calcHist(&hsv_roi, 1, channels, dst, roi_hist, 1, histSize, range); 
  normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX); 
  //Setup the termination criteria, either 100 iteraction or move by at least 1 pt
  TermCriteria term_crit(TermCriteria::EPS | TermCriteria::COUNT, 100, 1.0);
  //![Setup ROI for tracking]

  //![Declare Physics' variables]
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
  double Ax_max = 0.0, Ax_min = 999.0;	//+A and -A
  double Ay_max = 0.0;               	//Rope's lenght
  double vx_max = 0.0, vy_max = 0.0;
  double ax_max = 0.0, ay_max = 0.0;

  //cartesian line and QMM method
  double x0 = 0.0, y0 = 0.0;
  double L = 0.0;                      	//Rope length
  Vec2d m {0.0, 0.0};                  	//slope (generic and rope lines)
  double b = 0.0;						//intercept (generic lines)
  double theta = 0.0;
  double sum_m = 0.0;
  double sum_b = 0.0;
  double sum_m2 = 0.0;
  double sum_mb = 0.0;
  double sum2_m = 0.0;
  //![Declare Physics' variables]
  
  //![Loop over all frames]
  for (;;)
    { 
      //![prepare input frames]
      Mat foregroundMask, foreground, background, dst1;
      capture >> frame;

      //Conditions to continue
      if (frame.empty()){//end of capture
        break;
	  }
	  
	  const Size scaledSize(640, 640 * frame.rows / frame.cols);
	  resize(frame, frame, scaledSize, 0, 0, INTER_LINEAR); //zooming
	  //![prepare input frames] 

	  //![Background Subtraction Model]
	  // pass the frame to background model 
	  model->apply(frame, foregroundMask, doUpdateModel ? -1 : 0); 
	  		
	  // show foreground image and mask (with optional smoothing) 
	  if (doSmoothMask) 
	  {
	  GaussianBlur(foregroundMask, foregroundMask, Size(11, 11), 3.5, 3.5);
	  threshold(foregroundMask, foregroundMask, 200, 255, THRESH_BINARY);
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
	  //![Background Subtraction Model]

	  //![Meanshift tracking]	  
	  cvtColor(foreground, hsv, COLOR_BGR2HSV);
	  calcBackProject(&hsv, 1, channels, roi_hist, dst1, range);      

	  //apply meanshift to get the new location 
	  meanShift(dst1, track_window, term_crit); //Finds an object on a back projection image
	  //ROI Updated
	  roi = foreground(track_window);
	  imshow("current roi", roi);

	  //Draw it on frame
	  rectangle(frame, track_window, Scalar(0,240,255), 2);

	  //get track_window position
	  double xmin = track_window.x;
	  double ymin = track_window.y;
	  double w = track_window.width;
	  double h = track_window.height;
	  //![Meanshift tracking]

	  //![Get the centroid 1th-moments/0th-moments]
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
	  //![Get the centroid 1th-moments/0th-moments]

	  //![Physics]
	  //![Time elapsed]
	  //dt > 0 (avoid division by zero)
	  end = getTickCount();
	  dt = (double)(end - start)*1.0f / getTickFrequency();
	  cout << "\n" << endl;
	  cout << "time elapsed: " << format("%9.2f s", dt) << endl;
	  cout << "current i is: " << i << endl;
	  //![Time elapsed]

	  //![Position]
	  if (i < 4) {
	  	cout << "Storing the first five displacement's data..." << endl;
	  	X[i] = cx;
	  	Y[i] = cy;
	  }

	  if (i >= 4){
	  	cout << "Updating last position' data..." << endl;
	  	X[4] = cx;
	  	Y[4] = cy;
	  	//cout << "------------------------------------------" << endl;
	  	cout << "Now the X vector is: " << endl;
	  	for (int j=0; j<5; j++){
	  	//cout << X[j] << "\t";
	  	}
	  	//![Position]

	  	//![Displacement]
	  	cout << endl;
	  	cout << "--------------------------------------------------" << endl;
	  	cout << "Enough data to calculate velocity and acceleration" << endl;	  	
	  	///Position
	  	dx = X[3]-X[1];
		dy = Y[3]-Y[1];		  		
		cout << "dx = X[3]-X[1] = " << X[3] << " - " << X[1] << "= " << dx << endl;
		//![Displacement]

		//![Velocity]
		//cout << "Updating velocity ..." << endl;
		vx = dx/(2*dt);
		vy = dy/(2*dt);
		v = {vx, vy};
		cout << "vx = " << dx << "/(2*" << dt << ") = " << vx << endl;
		//![Velocity]

		//![Acceleration]
		//acceleration in both x and y axis using second derivative with
	  	//five position's data and middle point method
	  	ax = (-X[0] + 16*X[1] - 30*X[2] + 16*X[3] - X[4])/(12*dt*dt);
	  	ay = (-Y[0] + 16*Y[1] - 30*Y[2] + 16*Y[3] - Y[4])/(12*dt*dt);
	  	a = {ax, ay};
	  	cout << "ax = (-X[0] + 16*X[1] - 30*X[2] + 16*X[3] - X[4])/(12*dt*dt)" << endl;
  		cout << "ax = (-" << X[0] << " + 16*" << X[1] << " - 30* " << X[2]
  		<< "16*" << X[3] << " - " << X[4] << "(12*" << dt*dt << ")" << endl;
  		cout << "ax = " << ax << endl;
  		//![Acceleration]

  		//![Amplitude]
  	  	if (cx < Ax_min){
  	  		Ax_min = cx;
  	  		cout << "-A reached!" << endl;
  	  	}
  	  	if (cx > Ax_max){
  	  		Ax_max = cx;
  	  		cout << "+A reached!" << endl;
  	  	}
  	  	//![Amplitude]

  	  	//![Velocity vector's direction]
  	  	//Cartesian method
	  	m[0] = - dx / dy;    //slope (perpendicular line)
	  	b    = cy - m[0] * cx;  //intercept

	  	//proceed MMQ method in order to find the common point betwenn all lines
	  	if((i+1) % 5 == 0){ //i+1 remember: i is initialized as zero
	  		cout << "Totally brand new vector!" << endl;
	  		cout << "Let's sum up all cartesian elements" << endl;
	  		sum_m += m[0];
	  		sum_b += b;
	  		sum_m2 += m[0]*m[0];
	  		sum_mb += m[0]*b;
	  		N = i+1;
	  	}
	  	sum2_m += sum_m*sum_m;
	  	x0 = (N*sum_mb       -  sum_m*sum_b) / (-N*sum_m2 + sum2_m);
	  	y0 = (-sum_m2*sum_b  + sum_mb*sum_m) / (-N*sum_m2 + sum2_m);
	  	P0 = Point(x0,y0);

	  	//Rope lenght (L) and line equation
	  	L = norm(Center, P_0, NORM_L2);
	  	m[1] = (y0-cy)/(x0-cx);

	  	//Angle between rope and y axis
	  	theta = atan(abs((m[0]-m[1])/(1+m[0]*m[1])));
	  	cout << "Angle between y axis and rope axis: ";
	  	cout << theta << endl;
	  		  	
	  	//Vector methods
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
	 	//![Velocity vector's direction]

	 	//![Acceleration projection in radial and transversal directions]
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
  	  	cout << "Project acceleration onto velocity tangencial vector" << endl;
  	  	a_tangencial = dot_tangencial*v_unit_tangencial;
  	  	cout << "a_tangencial:" << "\t" << a_tangencial << endl;
  	
  	  	//cout << "Project acceleration onto velocity radial vector" << endl;
  	  	a_radial = dot_radial*v_unit_radial;
  	  	//![Acceleration projection in radial and transversal directions]

  	  	//![Forces]
  	  	//cout << "Gravity is the sum of tangencial and radial y component: ";
  	  	g = {0.0, abs(a_tangencial[1]+a_radial[1])};
  	  	//cout << "Tension force dwells in radial axis, but in opposite orientation: ";
  	  	T = v_unit_radial*(-abs(g.dot(v_unit_radial))-abs(a.dot(v_unit_radial)));
  	  	//![Forces]
  	  	//![Physics]

  	  	//![Position's update]
  	  	cout << "Before:" << "\t" << "{x0 x1 x2 x3 x4}" << endl;
	  	cout << "After:" << "\t" <<  "{x1 x2 x3 x4 ..}" << endl;
	  	cout << "-----------------------------------------------------" << endl;
	  	cout << "Before: " << endl;
	  	for (int j=0; j<5; j++){
	  		//cout << X[j] << "\t";
	  	}
	  	
	  	cout << endl;

	  	for (int n=0; n<4; n++){
	  		X[n]=X[n+1];
	  		Y[n]=Y[n+1];
	  	}
	  	X[4] = 0.0;

	  	//cout << "After: " << endl;
	  	for (int j=0; j<5; j++){
	  		//cout << X[j] << "\t";
	  	}
	  	cout << "\n" << endl;
  	  }
  	  //![Position's update]

  	  //![Drawing]
  	  //show the image with a point mark at the centroid
	  circle(frame,center,5,Scalar(0,240,255),-1); //BGR
  	
  	  //show the image with the displacement vector in x and y axis
  	  //r_x = Point (cx + dx , cy     );
  	  //r_y = Point (cx      , cy + dy);
  	  //arrowedLine (frame, center, r_x, Scalar(0, 255, 0), 3, 8);
  	  //arrowedLine (frame, center, r_y, Scalar(0, 0, 255), 3, 8);

  	  //show the image with the velocity vector in x and y axis
  	  v_x = Point (cx + 0.10*vx, cy);
  	  //v_y = Point (cx,      cy + vy);
  	  //arrowedLine (frame, center, v_x, Scalar(0, 255, 0), 3, 8);
  	  //arrowedLine (frame, center, v_y, Scalar(0, 255, 0), 3, 8);

  	  //show the image with tangencial and radial vectors (scaled)
  	  a_t = Point (100*v_unit_tangencial);
  	  a_r = Point (100*v_unit_radial);
  	  //arrowedLine (frame, center, center + a_t, Scalar(0, 0, 255), 3, 8);
  	  //arrowedLine (frame, center, center + a_r, Scalar(255, 0, 0), 3, 8);
  	  
  	  //Draw the cartesian lines  
	  line (frame, center, P0, Scalar(255, 0, 0), 1, 8, 0);
	  //Mat drawing = Mat::zeros(frame.size(), CV_8UC3);
	  //ellipse(drawing, Point (100, 100), Size (50, 50), 90, 0, 50*theta, Scalar(180, 180, 180), 1, 8, 0);

	  /*arrowedLine (frame, center, Point (cx, cy + 0.01*g[1]), Scalar(255, 255, 255), 3, 8);
  	  arrowedLine (frame, center, Point (cx + 0.01*T[0], cy + 0.01*T[1]), Scalar(0, 0, 0), 3, 8);*/
  	  //imshow("Rope", frame);
  	  //![Drawing]

  	  //![write information to a file]
  	  outputfile << i << "\t" << theta << "\t" << acos (v_unit_radial[1])*180.0 / PI << "\n";
  	  imshow("frame", frame);
  	  video.write(frame);
  	  //![write information to a file]

  	  i++;
  	  start = getTickCount(); //reset cronometer

	  int keyboard = waitKey(30); 
	  if (keyboard == 'q' || keyboard == 27) 
	  break;
	}

	//outputfile.close();
  //![Loop over all frames]
} 