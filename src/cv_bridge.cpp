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
int throttle = 0;
int lastSteer = 0;


//parameters
int maxThrottle = 80;
int linelenght = 75;
int contArea = 250;
int maxArea = 700;

int serialTog = 0;

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

int getMaxAreaContourId(vector <vector<cv::Point>> contours, bool &thresh) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
            //cout << "Area: " << maxArea << endl;
        } // End if
    } // End for
    if (maxArea > contArea && contours.size() > 0){
     
        thresh = true;
      
    }
    else{
      thresh = false;
    }
    return maxAreaContourId;
} // End function

char showImages(string title, vector<Mat>& imgs, Size cellSize) 
{
char k=0;
    namedWindow(title);
    float nImgs=imgs.size();
    int   imgsInRow=ceil(sqrt(nImgs));     // You can set this explicitly
    int   imgsInCol=ceil(nImgs/imgsInRow); // You can set this explicitly

    int resultImgW=cellSize.width*imgsInRow;
    int resultImgH=cellSize.height*imgsInCol;

    Mat resultImg=Mat::zeros(resultImgH,resultImgW,CV_8UC3);
    int ind=0;
    Mat tmp;
    for(int i=0;i<imgsInCol;i++)
    {
        for(int j=0;j<imgsInRow;j++)
        {
            if(ind<imgs.size())
            {
            int cell_row=i*cellSize.height;
            int cell_col=j*cellSize.width;
            imgs[ind].copyTo(resultImg(Range(cell_row,cell_row+tmp.rows),Range(cell_col,cell_col+tmp.cols)));
            }
            ind++;
        }
    }
    imshow(title,resultImg);
    k=waitKey(10);
    return k;
}


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

Point2f contourbouding(Mat &image, Mat &drawing, vector<vector<Point>> &contours, vector<Vec4i> &hierarchy, cv::Point &pt1, cv::Point &pt2, Scalar color, Vec4f &fitline, bool &threshold)
{
  findContours(image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  vector<vector<Point> > contours_poly( contours.size() );
  vector<RotatedRect> boundRect(contours.size());

  float d, t;

  Point2f rect_points[4];

  int i = getMaxAreaContourId(contours, threshold); //find biggest contour 


  if(i >= 0 && threshold){
    approxPolyDP( contours[i], contours_poly[i], 3, true );
    boundRect[i] = minAreaRect( contours[i] );

    fitLine(contours[i],fitline, DIST_L1 ,0, 0.01, 0.01);

    
    boundRect[i].points(rect_points);

    int longlength = 0;
    int newlength = 0;
    int shorterlength = 0;
    int longIndex = 0;

    for(int u = 0; u < 3; u++)
    {

      
        newlength = sqrt(pow((rect_points[u].x - rect_points[u+1].x), 2) + pow((rect_points[u].y - rect_points[u+1].y),2));



      if(newlength > longlength){
        longlength = newlength;
        longIndex = u;
      }
      else{

      }
      
    }
    
    shorterlength = sqrt(pow((rect_points[longIndex].x - rect_points[longIndex+2].x), 2) + pow((rect_points[longIndex].y - rect_points[longIndex+2].y),2));
    // note longlenght is actually returning the short lenght (cant be fucked fixing)
    if(shorterlength < linelenght && shorterlength != 1.3*longlength){
       Point2f noPoints[4];
       return noPoints[4];
    }

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

    //cout << "Vx "<< fitline[0] << " Vy "<< fitline[1] <<" angle " << atan2(fitline[1], fitline[0])*(180/M_PI) + 90<< " " << "threshold: "<< threshold <<endl;

    //cout << "Lngline "<< longlength <<" " << "short "<< shorterlength << " u: " << longIndex <<endl;

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
  int tempthrot = 0;
  while (1){

  


  mtx.lock();

  if(serialTog == 0){
    tempthrot = 0;
    serialTog = 1;
  }
  // else if (serialTog == 1){
  //   tempthrot = 0;
  //   serialTog = 2;
  // }
  else{
    tempthrot = throttle;
    serialTog = 0;

  }

  string steerAngstr = to_string((90 - steerAng)) +","+ to_string(tempthrot) + "\n";

  size_t bytesWritten = my_serial.write(steerAngstr);

  //std::cout << "run is true: "<< std::endl;
  mtx.unlock();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  //my_serial.flush();
  }
}

void throttleThread(){
  char inpt;
  while(1){
    cin >> inpt;
    if (inpt == 'w'){
      mtx.lock();
      throttle = maxThrottle;
      mtx.unlock();
    }
    else if (inpt == 's'){
      mtx.lock();
      throttle = 0;
      mtx.unlock();
    }
  }
}


///////////////////////////////////////////////

 int main( int argc, char** argv )
 {
   
  bool contourThreshY = false;
  bool contourThreshB = false;

    VideoCapture cap(0); //capture the video from web cam

    


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





 int iLowH = 64;
 int iHighH = 95;

 int iLowHB = 15;
 int iHighHB = 20;
 
 int iLowS = 18; 
 int iHighS = 255;

 int iLowV = 204;
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
thread input_ = thread(throttleThread);



//serialcntrl_.join();

char input;

while (true)
{
  
  //Mat imgOriginal;

  //bool bSuccess = cap.read(imgOriginal); // read a new frame from video

  //Mat temp;
  
  Mat temp; //(cv::Range(0,480), cv::Range(0,640)); //left
  cap >> temp;

  //flip(temp ,temp,-1);

  Mat leftcap = temp(cv::Range(0,240), cv::Range(0,320)); //left
  Mat rightcap = temp(cv::Range(0,240), cv::Range(320,640)); //right

  //  if (!bSuccess) //if not success, break loop
  // {
  //      cout << "Cannot read a frame from video stream" << endl;
  //      break;
  // }

  Mat imgHSV;
  Mat imgHSVR;

  cvtColor(leftcap, imgHSV, COLOR_RGB2HSV); //Convert the captured frame from BGR to HSV
  cvtColor(rightcap, imgHSVR, COLOR_RGB2HSV); 
 
  Mat imgThresholdedYellow;
  Mat imgThresholdedBlue;

  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholdedYellow); //Threshold the image yellow
  inRange(imgHSVR, Scalar(iLowHB, iLowS, iLowV), Scalar(iHighHB, iHighS, iHighV), imgThresholdedBlue); //Threshold the blue
  
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

  rect_pointsY[4] = contourbouding(imgThresholdedYellow, drawing, contoursY, hierarchyY, pt1y, pt2y, colorY, fitlineY, contourThreshY );

  rect_pointsB[4] = contourbouding(imgThresholdedBlue, drawing, contoursB, hierarchyB, pt1b, pt2b, colorB, fitlineB, contourThreshB );

  

  float angY;
  float angB;

  angY = -abs(linearea(fitlineY));
  angB = abs(linearea(fitlineB));

  //cout << "YellowAng: "<< angY <<" " << "BlueAng: "<< angB << " Throttle: "<<throttle <<endl;

  //mtx.lock();
  if (contourThreshY && contourThreshB){
    steerAng = (angY+angB)/2;
    lastSteer = steerAng;
  }
  else if (contourThreshY && !contourThreshB){
    //steerAng = angY*1.3;
    steerAng = -60;
    lastSteer = steerAng;
    //-50
  }
  else if (!contourThreshY && contourThreshB){
    // steerAng = angB*1.3;
    steerAng = 60;
    lastSteer = steerAng;
    //50
  }
  else {
    steerAng = lastSteer;
  }

  if (steerAng > 70){
    steerAng = 70;
    lastSteer = steerAng;
  }
  else if (steerAng < -70){
    steerAng = -70;
    lastSteer = steerAng;
  }

  //RIGHT = above 90
  //Left = below 90

  //string steerAngstr = to_string(90 - steerAng)+"\n";

  // draw contours on the original image


  string steerAngstr = to_string(steerAng);
  string throttlestr = to_string(throttle);

  putText(drawing, steerAngstr, cv::Point(10, drawing.rows / 2), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 0, 0), 2);
  putText(drawing, throttlestr, cv::Point(10, drawing.rows / 3), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 0, 0), 2);



  // drawContours(drawing, contoursY, -1, Scalar(0, 255, 0), 2);
  // drawContours(drawing, contoursB, -1, Scalar(0, 0, 255), 2);


  // vector<Mat> imgs;
  // imgs.push_back(imgThresholdedYellow);
  // imgs.push_back(imgThresholdedBlue);
  // imgs.push_back(image_copy);
  // imgs.push_back(drawing);
  
  // showImages("Thresholded Image Yellow / Blue", imgs, imgThresholdedYellow.size());

  imshow( "Contours", drawing );

  hconcat(imgThresholdedYellow,imgThresholdedBlue,imgThresholdedYellow); //combine images

  imshow("Thresholded Image Yellow / Blue", imgThresholdedYellow); //show the thresholded image
  //imshow("Thresholded Image Blue", imgThresholdedBlue); //show the thresholded image
  //imshow("Original", leftcap); //show the original image


  //size_t bytesWritten = my_serial.write(steerAngstr);

  //cout <<"val; " << steerAng << " normalizsed val "<< 90 - steerAng<< endl;
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

