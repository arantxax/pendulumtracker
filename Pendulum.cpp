//This source code is an attempt to draw physics vectors  
//(e.g. velocity) based on a simple pendulum's movement. 


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

using namespace cv; 
using namespace std; 


/** @function main */ 
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
  
  cout <<"Press <space> to toggle background model update"<< endl; 
  cout <<"Press 's' to toggle foreground mask smoothing"<< endl; 
  cout <<"Press ESC or 'q' to exit"<< endl; 
  bool doUpdateModel = true; 
  bool doSmoothMask = false; 
  
  //Declare variables to store the frames 
  Mat frame; Mat roi; 
  Mat hsv_roi; Mat mask; 
  Mat dframe; 
  
  //Take first frame of the video 
  capture >> frame; 
  
  //Setup initial location of windows for tracking 
  Rect track_window(50, 300, 50, 50); // simply hardcoded the value 

  //set up the ROI for Tracking 
  roi = frame(track_window);
  cvtColor(roi, hsv_roi, COLOR_BGR2HSV); 
  inRange(hsv_roi, Scalar(0, 60, 32), Scalar(180, 255, 255), mask);
  imshow ("initial roi", roi);
  
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
    {
      // prepare input frame 
      Mat hsv, dst; 
      Mat frame, foregroundMask, foreground, background; 
      capture >> frame;
      if (frame.empty())//end of capture          
	break;
	  
      const Size scaledSize(640, 640 * frame.rows / frame.cols); 
      resize(frame, frame, scaledSize, 0, 0, INTER_LINEAR); //resizing: zooming 

      // pass the frame to background model 
      model->apply(frame, foregroundMask, doUpdateModel ? -1 : 0); 
      
      // show resized frame 
      imshow("frame", frame);
      
      // show foreground image and mask (with optional smoothing) 
      if (doSmoothMask) 
	{ 
	  GaussianBlur(foregroundMask, foregroundMask, Size(11, 11), 3.5, 3.5); 
	  threshold(foregroundMask, foregroundMask, 10, 255, THRESH_BINARY);
	} 
      if (foreground.empty()) 
	foreground.create(scaledSize, frame.type()); 
      foreground = Scalar::all(0); 
      frame.copyTo(foreground, foregroundMask); 
      
      imshow("foregroundMask", foregroundMask);
      imshow("foregroundImage", foreground);
    
      // show background image 
      model->getBackgroundImage(background); 
      if (!background.empty()) {
	imshow("mean background image", background);
      }
      cvtColor(foreground, hsv, COLOR_BGR2HSV); 
      calcBackProject(&hsv, 1, channels, roi_hist, dst, range); 
      
      //apply meanshift to get the new location 
      meanShift(dst, track_window, term_crit); //Finds an object on a back projection image
      imshow("dst", dst);
      
      // get track_window position// little reminder: Mat( 50, 50, CV_8UC3)
      double xmin = track_window.x;
      double ymin = track_window.y;
      double w = track_window.width;
      double h = track_window.height;
      cout << "xmin:" << xmin << endl;

      //ROI updated
      roi = foreground(track_window);   
      imshow("New ROI", roi);

      // Draw it on image 
      rectangle(foreground, track_window, Scalar(0,240,255), 2); 
      imshow("foreground with tracker", foreground);
      
      ///Start Moments
      Mat canny_output;
      int thresh = 10;
      
      //Convert image to gray and blur it (again)
      cvtColor(roi,canny_output,COLOR_BGR2GRAY);
      blur(canny_output, canny_output,Size(3,3));
      imshow("blurred New ROI", canny_output);
      
      /// Detect edges using canny 
      Canny(roi, canny_output, thresh, thresh*2, 3 );
      
      /// Find contours 
      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;
      findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE,Point(0,0));
      imshow("contoursROI",canny_output);
            
      // Get the centroid 1th-moments/0th-moments
      Moments m=moments(canny_output,true);
      //cout << "canny_output.x"<< canny_output.x << "\n";
      //cout << "canny_output.y" << canny_output.y << "\n";
                  
      //Get centroid considering the ROI's coordinates
      //int nFrames = capture.get(CAP_PROP_FRAME_COUNT);
      //vector<double> cx[i];
      //vector<double> cy[i];
      //vector<double> x[i];
      //vector<double> y[i];
      Point p;

	//for(int i=0; i<nFrames; i++){} //considering
      
      if(m.m00!=0){
	cx = m.m10/m.m00;
	cy = m.m01/m.m00;
	x = cx + xmin;
	y =  cy + ymin;
	p = Point(x,y);
		//p((double)(m.m10 / m.m00), (double)(m.m01 / m.m00) );
      }
      else{
	cout << "\n";
	p = Point(0,0);//avoid division by zero
      }
      
      cout << "coordinates (x,y)" << p <<endl;
  
/*    
      //Begin Physics //midpoint or backwards?
      //Declare variables
      vector<double> dx[i];
      time_t start, end;
      vector<double> dt[i];
      vector<double> v[i];


      //Initial conditions
      time_t start;
      
      //displacement and time elapsed
	//for(int i=0; i<nFrames; i++){} // considering
      dx[i] = x[i+2] - x[i];
      time_t end;
      dt[i] = 2*difftime(end,start);
      cout << "displacement:" << dx << "[" << i << "]" << endl;
      cout << "time elapsed:" << dt << "[" << i << "]" << endl;
      double v[i] = dx/dt;
      cout << "velocity" << v << "[" << i << "]" << endl;

      x[i]=x[i+2];


*/
             
      /// Create Window 
      //const char* moments_window = "Moments"; 
      //namedWindow( moments_window ); 
      //imshow("moments_window", canny_output ); 

      //show the image with a point mark at the centroid
      circle(foreground,p,5,Scalar(0,240,255),-1); //BGR
      imshow("Center", foreground);
      
      int keyboard = waitKey(30); 
      if (keyboard == 'q' || keyboard == 27) 
	break;    
    } 
 } 