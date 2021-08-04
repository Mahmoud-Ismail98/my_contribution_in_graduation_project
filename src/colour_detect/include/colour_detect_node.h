

#ifndef colour_DETECT_NODE_H
#define colour_DETECT_NODE_H

// Dependencies
#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Bool.h"
//#include "CameraObjectInfo.h"
#include "colour_detect/CameraObjectInfo.h"
#include <sstream>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <colour_detect/colourParamsConfig.h>

// Quality of life
using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace image_transport; 	
static const std::string OPENCV_WINDOW = "Binary image, colour";
static const std::string WINDOW2 = "Bounding boxes, colour";
static const std::string CAMERA_FRAME = "manta/camerafront_link";


class colourFinder{

private:

	 // Setting up ROS handles, subsribers and publishers	
	 ros::NodeHandle nh_;
	 ros::NodeHandle n_;
	 ImageTransport it_;

	 // ros topics
	 Subscriber image_sub_;
	 Publisher image_pub_;
	 Publisher red_image_pub_;
	 ros::Publisher detect_pub_;
	  
	 // Dynamic reconfigure
	 dynamic_reconfigure::Server<colour_detect::colourParamsConfig> server;
	 dynamic_reconfigure::Server<colour_detect::colourParamsConfig>::CallbackType f;
	  
	 // hsv variables
	 int minhue,maxhue,minval,maxval,minsat,maxsat;

	 // colour height treshold for detection
	 double height;

	 // image weights
	 double alpha, beta, gamma;

	 // Gaussian kernel standard deviation
	 double sigmaX, sigmaY;

	 // object distance
	 float distance;

	 // image frame coordinates
	 double x1, x2, y1, y2,x11,x22,y11,y22;

	 // cv::Mat - n-dimensional dense array class 
	 // This is an actual image in matrix form
	 cv::Mat hsv_image, detected_edges, blurred_image, lower_red_temp_image, upper_red_temp_image, red_image;


	 cv::Rect2d bbox, bbox_big; 			 //Bounding boxes
	 vector<Rect2d> act_bbox; 				 //Vector with bounding boxes
	 colour_detect::CameraObjectInfo detected; //Message to be published

	  
	 //-------------- HAVE TO BE TUNED --------------
	 float width_colour = 0.4; // Height pixels of an object 1m from camera
	 float focal_length = 332.5; // F = (PxD) / W (P - Pixle width, D - Distance, W - width of colour)


public:

    // Constructor runs run() function
    colourFinder(int argc, char** argv)
      : it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/camera/image", 1, &colourFinder::run, this);
	  image_pub_ = it_.advertise("/camera/colour_detect",1);
	  red_image_pub_ = it_.advertise("/camera/colour_tuning",1);
      detect_pub_ = n_.advertise<colour_detect::CameraObjectInfo>("colour_midpoint",1000);
      cv::namedWindow(OPENCV_WINDOW);
    }

    // Destructor
    ~colourFinder(){
      cv::destroyWindow(OPENCV_WINDOW);
    }


    /**** FUNCTIONS  ****/

    // dynamic reconfigure
    void configCallback(const colour_detect::colourParamsConfig &config, uint32_t level);

    // CvImagePtr is a shared ptr of the CvImage class that is interoperable with
    // sensor_msgs/Image, but uses a more convenient cv::Mat representation for the
    // Image data
    void init_msg(cv_bridge::CvImagePtr cv_ptr);

    // red filtering and edge detection in image
    void redFilterAndEgde(cv_bridge::CvImagePtr cv_ptr);

    // finds contours in a binary image
    void Contours(cv_bridge::CvImagePtr cv_ptr);

    
    void findDistance(cv_bridge::CvImagePtr cv_ptr);

    void drawOnImage(cv_bridge::CvImagePtr cv_ptr);

    void run(const sensor_msgs::ImageConstPtr& msg);

};

#endif
