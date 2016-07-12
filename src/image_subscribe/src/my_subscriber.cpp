#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/video/tracking.hpp"
#include <ctime>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
namespace enc = sensor_msgs::image_encodings;


cv::Mat bcimg;
cv::Mat Thresholded;
static cv::Mat currentImage;
int ROI_y, ROI_width, ROI_height;

int iLowH = 60;
int iHighH = 79;

int iLowS = 229;
int iHighS = 255;

int iLowV = 51;
int iHighV = 255;

int nancheck_offset_x, nancheck_offset_y;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    // currentImage = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  // if (!cv_ptr->image.empty())
  // {
  currentImage = cv_ptr->image;
  // }
}

void color_detection(cv::Mat img)
{
  cv::Mat imgROI = img;
  cv::Mat imgCanny;
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  cv::Mat imgHSV;
  float offset_x;
  float offset_y;
  float area = 0.0;
  int largest_area = 0;
  int largest_contour_index = 0;
  cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  cv::cvtColor(imgROI, imgHSV, CV_RGB2HSV); //Convert the captured frame from BGR to HSV

  cv::Mat imgThresholded;

  cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

  //morphological opening (remove small objects from the foreground)
  cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
  // cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

  //morphological closing (fill small holes in the foreground)
  // cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
  // cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

  cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image

  // ROI_y = img.rows*13.0/100.00;    //Percentage by which orgin of the ROI has to be shifted in y direction.
  // ROI_width = img.cols; //Width of the ROI
  // ROI_height = img.rows*10.0/100.00;

  // imgROI=imgThresholded(cv::Rect(0, ROI_y, ROI_width, ROI_height)); //Get the region of interest(ROI)
  // // Find the oplygon in third order
  // cv::Moments mu = moments(imgThresholded, true);
  // // Find the center of coordinate

  // cv::Canny( imgThresholded, imgCanny, 100, 500, 3 );

  // cv::imshow("Canny Image", imgCanny); //show the thresholded image

  findContours( imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  // /// Get the moments
  // vector<cv::Moments> mu(contours.size() );
  // for ( int i = 0; i < contours.size(); i++ )
  // { mu[i] = moments( contours[i], false );}

  // // ///  Get the mass centers:
  // vector<cv::Point2f> mc( contours.size() );
  // for ( int i = 0; i < contours.size(); i++ )
  // { mc[i] = cv::Point2f( mu[i].m10 / mu[i].m00 , mu[i].m01 / mu[i].m00 ); }

  // cv::Moments mu2 = moments(imgThresholded, true);
  // cv::Point2f center(mu2.m10 / mu2.m00, mu2.m01 / mu2.m00);


  //   Mat drawing = Mat::zeros( imgCanny.size(), CV_8UC3 );
  // for( int i = 0; i< contours.size(); i++ )
  //    {
  //      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
  //      drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
  //      circle( drawing, mc[i], 4, color, -1, 8, 0 );
  //    }
  // vector<cv::Point2f> mc( contours.size() );
  for ( int i = 0; i < contours.size(); i++ )
  {
    // area += mu2.m00;
    double area = contourArea( contours[i] );  //  Find the area of contour

    if ( area > largest_area )
    {
      largest_area = area;
      largest_contour_index = i;               //Store the index of largest contour
      // bounding_rect = boundingRect( contours[i] ); // Find the bounding rectangle for biggest contour
      // ROS_INFO(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
      // cv::Moments mu = moments(contours[largest_contour_index], true);
    }
    else
    {
      // break();
    }
// double area = cv::contourArea(contours);
  }
  if (largest_area > 0) {
    cv::Moments mu = moments(contours[largest_contour_index], true);
    // vector<cv::Moments> mu(contours[largest_contour_index] );
    // vector<cv::Point2f> mc( contours.size() );
    // mu[largest_contour_index] = moments( contours[largest_contour_index], false);
    cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
    offset_x = (1 - 2 * center.x / imgThresholded.cols);
    offset_y = (1 - 2 * center.y / imgThresholded.rows);
  }
  else
  {
    largest_area = 0;
  }
// cv::circle(imgROI, cv::Point(imgThresholded.cols / 2, imgThresholded.rows / 2), 10, CV_RGB(255, 0, 0));

  ROS_INFO("Area = %d", largest_area);
  ROS_INFO("offset_x = %f ", offset_x);
  ROS_INFO("offset_y = %f ", offset_y);
// nancheck_offset_x = isnan(offset_x);
// nancheck_offset_y = isnan(offset_y);
  if (nancheck_offset_x || nancheck_offset_y)
  {
    ROS_INFO("have a nan number");
  }
  else if (largest_area <= 5000)
  {
    ROS_INFO("Area of interest is not enough to calculate");
  }
  else
  {
    cv::circle(imgROI, cv::Point(320 - (offset_x * 320), 240 - (offset_y * 240)), 20, CV_RGB(255, 0, 255));
    cv::line(imgROI, cv::Point( imgThresholded.cols / 2, imgThresholded.rows / 2 ), cv::Point(320 - (offset_x * 320), 240 - (offset_y * 240)), CV_RGB(255, 0, 100), 3, 2 );
  }
// cv::circle(imgThresholded,Point((offset_x*,imgThresholded.rows/2),4,cv::Scalar( 0, 0,255),10,8);
// if (offset_x == 'nan')
// int pixel_to_
// cv::circle(imgROI,center,2,cv::Scalar(255,100,255),-1,2);

// threshold(imgROI,imgROI, thresh, 255, is_black);
// cv::GaussianBlur(imgThresholded, imgBlur,cv::Size(9, 9), 2, 2);
// cv::imshow("Blur Image", imgBlur);
// cv::Canny(img,imgBlur,100,200,3);
// List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
// vector<vector<Point> > contours;
// vector<Vec4i> hierarchy;
// cv::findContours(imgThresholded,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


  cv::imshow("ROI", imgROI); //show the original image
// return imgThresholded;
}

cv::Mat brightness_and_contrast(cv::Mat img)
{
/// Initialize values
  cv::Mat new_image = cv::Mat::zeros( img.size(), img.type() );
  // cv::cvtColor(img, new_image, CV_BGR2GRAY); //Convert the captured frame from BGR to HSV
  float alpha = 1.5;
  int beta = -20;
/// Do the operation new_image(i,j) = alpha*image(i,j) + beta
  // for ( int y = 0; y < img.rows; y++ )
  // { for ( int x = 0; x < img.cols; x++ )
  //   { for ( int c = 0; c < 3; c++ )
  //     {
  //       new_image.at<cv::Vec3b>(y, x)[c] =
  //         cv::saturate_cast<uchar>( alpha * ( img.at<cv::Vec3b>(y, x)[c] ) + beta );
  //     }
  //   }
  // }
// use this method instead
  img.convertTo(new_image, -1, alpha, beta);
/// Create Windows
  // namedWindow("Original Image", 1);
  cv::namedWindow("New Image", 1);

/// Show stuff
  // imshow("Original Image", img);
  // cv::imshow("New Image", new_image);
  return new_image;
/// Wait until user press some key

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  // ros::NodeHandle local_nh("~");
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  ros::TransportHints ros_hints;
  image_transport::TransportHints hints("jpeg", ros_hints.udp());
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback, hints);
  // image_transport::Subscriber sub = it.subscribe("camera/image",1, imageCallback);
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    if ( !currentImage.empty() )
    {
      // ROS_INFO("Copy Completed");
      // cv::imshow("view", currentImage);
      bcimg = brightness_and_contrast(currentImage);
      color_detection(bcimg);
      cv::waitKey(51);
    }
    ros::spinOnce();
    // ros::spin();
    loop_rate.sleep();
  }
  cv::destroyWindow("view");
}