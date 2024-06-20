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

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

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


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "image_converter");
//   ImageConverter ic;
//   ros::spin();
  
//   cv::Mat right;
//   cv::Mat left;

//   namedWindow("Control"); //create a window called "Control"


//  int iLowH = 0;
//  int iHighH = 179;

//  int iLowS = 0; 
//  int iHighS = 255;

//  int iLowV = 0;
//  int iHighV = 255;

//  //Create trackbars in "Control" window
//  createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
//  createTrackbar("HighH", "Control", &iHighH, 179);

//  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
//  createTrackbar("HighS", "Control", &iHighS, 255);

//  createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
//  createTrackbar("HighV", "Control", &iHighV, 255);

//   while (true)
//   {
    
//     ic.Getstereo(left, right);
//     //     bool bSuccess = right; // read a new frame from video

//     //      if (!bSuccess) //if not success, break loop
//     //     {
//     //          cout << "Cannot read a frame from video stream" << endl;
//     //          break;
//     //     }

//     Mat imgHSV;

//     cvtColor(right, imgHSV, COLOR_RGB2HSV); //Convert the captured frame from BGR to HSV
 
//     Mat imgThresholded;

//     inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
//     //morphological opening (remove small objects from the foreground)
//     erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
//     dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

//     //morphological closing (fill small holes in the foreground)
//     dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
//     erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

//     imshow("Thresholded Image", imgThresholded); //show the thresholded image
//     imshow("Original", right); //show the original image

//         if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
//        {
//             cout << "esc key is pressed by user" << endl;
//             break; 
//        }
//   }


//   return 0;
// }

 int main( int argc, char** argv )
 {
    VideoCapture cap; //capture the video from web cam

    


    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control"); //create a window called "Control"

 int iLowH = 0;
 int iHighH = 255;

 int iLowS = 0; 
 int iHighS = 255;

 int iLowV = 0;
 int iHighV = 255;

 //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 255);

 createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

 createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);

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
  //inRange(imgHSV, Scalar(9, iLowS, iLowV), Scalar(18, iHighS, iHighV), imgThresholdedBlue); //Threshold the blue
  
  //blur( imgThresholdedYellow, imgThresholdedYellow, Size(10,10) );

  //morphological opening (remove small objects from the foreground)
  erode(imgThresholdedYellow, imgThresholdedYellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholdedYellow, imgThresholdedYellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  // erode(imgThresholdedBlue, imgThresholdedBlue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  // dilate( imgThresholdedBlue, imgThresholdedBlue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate( imgThresholdedYellow, imgThresholdedYellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholdedYellow, imgThresholdedYellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  // dilate( imgThresholdedBlue, imgThresholdedBlue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  // erode(imgThresholdedBlue, imgThresholdedBlue, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


  // for (int i = 0; i < imgThresholdedYellow.rows; i++ ){
  //   for (int j = 0; j < imgThresholdedYellow.cols; j++){


  //     cout << imgThresholdedYellow.at<double>(j, i) << " ";

  //   }
  //   cout << endl;
  // }

  // cv::MatIterator_<double> _it = imgThresholdedYellow.begin<double>();
  // for(;_it!=imgThresholdedYellow.end<double>(); _it++){
  //   std::cout << *_it << std::endl;
  // }

  //cout << imgThresholdedYellow.cols << endl;

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(imgThresholdedYellow, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  

  vector<vector<Point> > contours_poly( contours.size() );
  vector<RotatedRect> boundRect(contours.size());
  Vec4f fitline;
   cv::Point pt1, pt2;
       float d, t;
  

  int i = getMaxAreaContourId(contours); //find biggest contour

   Mat drawing = leftcap.clone();

  if(i >= 0){
  approxPolyDP( contours[i], contours_poly[i], 3, true );
  boundRect[i] = minAreaRect( contours[i] );

  Scalar color = Scalar( 255, 0, 0 );

  fitLine(contours[i],fitline, DIST_L1 ,0, 0.01, 0.01);

  Point2f rect_points[4];
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

  cout << "Vx "<< fitline[0] << " Vy "<< fitline[1] <<" angle " << atan2(fitline[1], fitline[0])*(180/M_PI) + 90<< " " << endl;

  //cout << "pont1 "<< fitline[2] <<" " << "point2"<< fitline[3]<<endl;

  for ( int j = 0; j < 4; j++ )
  {
    line( drawing, rect_points[j], rect_points[(j+1)%4], color );

  }

  }
  imshow( "Contours", drawing );

  // draw contours on the original image
  Mat image_copy = leftcap.clone();
  drawContours(image_copy, contours, -1, Scalar(0, 255, 0), 2);
  imshow("None approximation", image_copy);

  imshow("Thresholded Image Yellow", imgThresholdedYellow); //show the thresholded image
  // imshow("Thresholded Image Blue", imgThresholdedBlue); //show the thresholded image
  imshow("Original", leftcap); //show the original image


        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }

       

    }


   return 0;

}

