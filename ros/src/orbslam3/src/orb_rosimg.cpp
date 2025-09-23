#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "System.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
 
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>
 
using namespace std;
 
string paramterfile = "/home/py/orb_slam/ORB_SLAM3_Ubuntu20.04/ros/src/orbslam3/config/myslam_640_480.yaml";
string vocFile = "/home/py/orb_slam/ORB_SLAM3_Ubuntu20.04/Vocabulary/ORBvoc.txt";
 
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}
 
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
 
    ORB_SLAM3::System* mpSLAM;
};
 
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // cout << "hahaha" << endl;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
            cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
 
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}
 
int main(int argc, char *argv[])
{

 
    ros::init(argc, argv, "Mono");
    ros::start();
 

    ORB_SLAM3::System SLAM(vocFile, paramterfile, ORB_SLAM3::System::MONOCULAR, true);

    ImageGrabber igb(&SLAM);
 
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage, &igb);
    
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();
    
    return 0;
}