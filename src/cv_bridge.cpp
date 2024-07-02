#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <math.h>

//serial library
#include "serial/serial.h"

//threading libraries
#include <thread>
#include <mutex> 


using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

mutex mtx;

int steerAng = 90;

serial::Serial my_serial("/dev/ttyACM0", 19200, serial::Timeout::simpleTimeout(3000));


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  cv::Mat R;
  cv::Mat L;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(200, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    //cv::Size s = cv_ptr->image.size();
    
    cv::Mat A ;//= cv_ptr->image.clone();
    cv::flip(cv_ptr->image.clone(),A,-1);
    L = A(cv::Range(0,480), cv::Range(0,640)); //left
    R = A(cv::Range(0,480), cv::Range(640,1280)); //right
    //v::Mat Lflip ;
    //cv::flip(L, Lflip, 0);

    //cv::imshow(OPENCV_WINDOW, A);
    //cv::imshow(OPENCV_WINDOW, L);
    //cv::imshow(OPENCV_WINDOW, R);

    //(0:480, 0:640);
 
    //std::cout << "Width :" << s << std::endl;
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    

    
  }

  void Getstereo(cv::Mat &left, cv::Mat &right)
  {
    left = L;
    right = R;
  }
};

int getMaxAreaContourId(vector <vector<cv::Point>> contours) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } // End for
    return maxAreaContourId;
} // End function

float angleBetween(const Point &v1, const Point &v2)
{
    float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

    float dot = v1.x * v2.x + v1.y * v2.y;

    float a = dot / (len1 * len2);

    if (a >= 1.0)
        return 0.0;
    else if (a <= -1.0)
        return M_PI;
    else
        return acos(a); // 0..PI
}

Point2f contourbouding(Mat &image, Mat &drawing, vector<vector<Point>> &contours, vector<Vec4i> &hierarchy, cv::Point &pt1, cv::Point &pt2, Scalar color, Vec4f &fitline)
{
  findContours(image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  vector<vector<Point> > contours_poly( contours.size() );
  vector<RotatedRect> boundRect(contours.size());

  float d, t;

  Point2f rect_points[4];

  int i = getMaxAreaContourId(contours); //find biggest contour 


  if(i >= 0){
    approxPolyDP( contours[i], contours_poly[i], 3, true );
    boundRect[i] = minAreaRect( contours[i] );

    fitLine(contours[i],fitline, DIST_L1 ,0, 0.01, 0.01);

    
    boundRect[i].points( rect_points );

    // ... and the long enough line to cross the whole image
    d = sqrt((double)fitline[0] * fitline[0] + (double)fitline[1] * fitline[1]);
    fitline[0] /= d;
    fitline[1] /= d;
    t = (float)(drawing.cols + drawing.rows);
    pt1.x = cvRound(fitline[2] - fitline[0] * t);
    pt1.y = cvRound(fitline[3] - fitline[1] * t);
    pt2.x = cvRound(fitline[2] + fitline[0] * t);
    pt2.y = cvRound(fitline[3] + fitline[1] * t);
    cv::line(drawing, pt1, pt2, cv::Scalar(0, 255, 0));//, 0, LINE_AA, 0);

    //cout << "Vx "<< fitline[0] << " Vy "<< fitline[1] <<" angle " << atan2(fitline[1], fitline[0])*(180/M_PI) + 90<< " " << endl;

    //cout << "pont1 "<< fitline[2] <<" " << "point2"<< fitline[3]<<endl;

    for ( int j = 0; j < 4; j++ )
    {
      line( drawing, rect_points[j], rect_points[(j+1)%4], color );

    }

  }
  return rect_points[4];
}

float linearea(Vec4f fitline)
{
  float angle = atan2(fitline[1], fitline[0])*(180/M_PI) + 90;

  if (angle > 90){
    return angle = -180 + angle;
  }
  else{
    return angle;
  }

}

void serialThread()
{
  while (1){

  
  mtx.lock();

  string steerAngstr = to_string(90 - steerAng)+"\n";

  size_t bytesWritten = my_serial.write(steerAngstr);

   std::cout << "run is true: "<< std::endl;
  mtx.unlock();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  //my_serial.flush();
  }
}




 int main( int argc, char** argv )
 {
   
    VideoCapture cap(1); //capture the video from web cam

    


    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control"); //create a window called "Control"


//serial::Serial my_serial("/dev/ttyACM0", 9600, serial::Timeout::simpleTimeout(3000));

  if (my_serial.isOpen())
  {
    std::cout << "port opened successfully" << std::endl;
  }
  else
  {
    cout << "Port failed to open" << endl;
  }

my_serial.flushOutput();
std::this_thread::sleep_for(std::chrono::milliseconds(1000));





 int iLowH = 0;
 int iHighH = 255;

 int iLowHB = 0;
 int iHighHB = 255;
 
 int iLowS = 0; 
 int iHighS = 255;

 int iLowV = 0;
 int iHighV = 255;

 //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 255);

 createTrackbar("LowHB", "Control", &iLowHB, 255); //Hue (0 - 179)
 createTrackbar("HighHB", "Control", &iHighHB, 255);

 createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

 createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);

thread serialcntrl_ = thread(serialThread);



//serialcntrl_.join();



    while (true)
    {
     
        //Mat imgOriginal;

        //bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        //Mat temp;

       
        Mat temp; //(cv::Range(0,480), cv::Range(0,640)); //left
        cap >> temp;

        

        flip(temp ,temp,-1);

        Mat leftcap = temp(cv::Range(0,240), cv::Range(0,320)); //left
        //Mat rightcap = temp(cv::Range(0,480), cv::Range(640,1280)); //right

        //  if (!bSuccess) //if not success, break loop
        // {
        //      cout << "Cannot read a frame from video stream" << endl;
        //      break;
        // }

  Mat imgHSV;

  cvtColor(leftcap, imgHSV, COLOR_RGB2HSV); //Convert the captured frame from BGR to HSV
 
  Mat imgThresholdedYellow;
  Mat imgThresholdedBlue;

  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholdedYellow); //Threshold the image yellow
  inRange(imgHSV, Scalar(iLowHB, iLowS, iLowV), Scalar(iHighHB, iHighS, iHighV), imgThresholdedBlue); //Threshold the blue
  
  //blur( imgThresholdedYellow, imgThresholdedYellow, Size(10,10) );

  //morphological opening (remove small objects from the foreground)
  erode(imgThresholdedYellow, imgThresholdedYellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholdedYellow, imgThresholdedYellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  erode(imgThresholdedBlue, imgThresholdedBlue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholdedBlue, imgThresholdedBlue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate( imgThresholdedYellow, imgThresholdedYellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholdedYellow, imgThresholdedYellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  dilate( imgThresholdedBlue, imgThresholdedBlue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholdedBlue, imgThresholdedBlue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


//Find contours and lines of best fit
  vector<vector<Point>> contoursY;
  vector<Vec4i> hierarchyY;

  vector<vector<Point>> contoursB;
  vector<Vec4i> hierarchyB;


  Vec4f fitlineY;
  cv::Point pt1y, pt2y;

  Vec4f fitlineB;
  cv::Point pt1b, pt2b;



  Scalar colorY = Scalar( 255, 0, 0 );


  Scalar colorB = Scalar( 0, 0, 255 );

  Mat drawing = leftcap.clone();


  Point2f rect_pointsY[4];
  Point2f rect_pointsB[4];

  rect_pointsY[4] = contourbouding(imgThresholdedYellow, drawing, contoursY, hierarchyY, pt1y, pt2y, colorY, fitlineY  );

  rect_pointsB[4] = contourbouding(imgThresholdedBlue, drawing, contoursB, hierarchyB, pt1b, pt2b, colorB, fitlineB  );

  imshow( "Contours", drawing );

  float angY;
  float angB;

  angY = linearea(fitlineY);
  angB = linearea(fitlineB);

  //cout << "YellowAng: "<< angY <<" " << "BlueAng: "<< angB <<endl;

  //mtx.lock();

  steerAng = (angY+angB)/2;

  //string steerAngstr = to_string(90 - steerAng)+"\n";

  // draw contours on the original image
  Mat image_copy = leftcap.clone();

  drawContours(image_copy, contoursY, -1, Scalar(0, 255, 0), 2);
  drawContours(image_copy, contoursB, -1, Scalar(0, 0, 255), 2);

  imshow("None approximation", image_copy);

  imshow("Thresholded Image Yellow", imgThresholdedYellow); //show the thresholded image
  imshow("Thresholded Image Blue", imgThresholdedBlue); //show the thresholded image
  //imshow("Original", leftcap); //show the original image


  //size_t bytesWritten = my_serial.write(steerAngstr);

  cout << "val "<< 90 - steerAng<< endl;
  //std::this_thread::sleep_for(std::chrono::milliseconds(100));

  //my_serial.flushOutput();

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }

       

    }
  //mtx.unlock();

   return 0;

}

