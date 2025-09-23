#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "System.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <chrono>
#include <string>
#include <iostream>
#include <fstream>
 
using namespace std;

// Paths to configuration and vocabulary files
string parameterFile = "/home/py/orb_slam/ORB_SLAM3_Ubuntu20.04/ros/src/orbslam3/config/myslam_640_480.yaml";
string vocFile = "/home/py/orb_slam/ORB_SLAM3_Ubuntu20.04/Vocabulary/ORBvoc.txt";

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, cv::VideoCapture& cap, ros::Publisher& pose_pub)
        : mpSLAM(pSLAM), mCap(cap), mPosePub(pose_pub) {}

    bool GrabImage(); // Returns false if frame capture fails or user quits

    ORB_SLAM3::System* mpSLAM;
private:
    cv::VideoCapture& mCap; // Reference to the VideoCapture object
    ros::Publisher& mPosePub; // Reference to the pose publisher
};

bool ImageGrabber::GrabImage()
{
    cv::Mat frame;
    mCap >> frame;
    if (frame.empty()) {
        std::cerr << "ERROR: Failed to capture frame" << std::endl;
        return false;
    }

    // Get current timestamp (in seconds)
    double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // Process frame with ORB-SLAM3 and get pose
    cv::Mat pose = mpSLAM->TrackMonocular(frame, timestamp);

    // Check if pose is valid (not empty and 4x4)
    if (!pose.empty() && pose.rows == 4 && pose.cols == 4) {
        // Extract rotation (3x3) and translation (3x1) from pose matrix
        cv::Mat R = pose(cv::Rect(0, 0, 3, 3));
        cv::Mat t = pose(cv::Rect(3, 0, 1, 3));

        // Convert rotation matrix to quaternion
        cv::Mat Rvec;
        cv::Rodrigues(R, Rvec); // Convert rotation matrix to rotation vector
        double theta = cv::norm(Rvec);
        cv::Mat axis = Rvec / (theta + 1e-6); // Avoid division by zero
        double angle = theta / 2.0;
        double sinAngle = sin(angle);

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time(timestamp);
        pose_msg.header.frame_id = "map"; // Adjust frame_id as needed

        // Set position
        pose_msg.pose.position.x = t.at<float>(0);
        pose_msg.pose.position.y = t.at<float>(1);
        pose_msg.pose.position.z = t.at<float>(2);

        // Set orientation (quaternion)
        pose_msg.pose.orientation.x = sinAngle * axis.at<float>(0);
        pose_msg.pose.orientation.y = sinAngle * axis.at<float>(1);
        pose_msg.pose.orientation.z = sinAngle * axis.at<float>(2);
        pose_msg.pose.orientation.w = cos(angle);

        // Publish the pose
        mPosePub.publish(pose_msg);
        ROS_INFO_STREAM("Published pose: x=" << pose_msg.pose.position.x
                        << ", y=" << pose_msg.pose.position.y
                        << ", z=" << pose_msg.pose.position.z);
    } else {
        ROS_WARN("Invalid or empty pose returned by ORB-SLAM3");
    }

    // Display the frame for visualization
    // cv::imshow("Camera Feed", frame);

    // Return false if 'q' is pressed to exit
    if (cv::waitKey(1) == 'q') {
        return false;
    }

    return true;
}

int main(int argc, char *argv[])
{
    // Initialize ROS
    ros::init(argc, argv, "orb_slam3_mono");
    ros::start();
    ros::NodeHandle nh;

    // Create publisher for pose
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam3/pose", 10);

    // Initialize ORB-SLAM3 system (Monocular mode)
    ORB_SLAM3::System SLAM(vocFile, parameterFile, ORB_SLAM3::System::MONOCULAR, false);

    // Initialize OpenCV VideoCapture
    cv::VideoCapture cap(4, cv::CAP_V4L2); // Camera index 4 with V4L2 backend
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        ros::shutdown();
        return -1;
    }

    // Set camera resolution
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // Verify camera settings
    std::cout << "Camera opened with resolution: "
              << cap.get(cv::CAP_PROP_FRAME_WIDTH)
              << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

    // Initialize ImageGrabber with SLAM, VideoCapture, and publisher
    ImageGrabber igb(&SLAM, cap, pose_pub);

    // Main loop to capture and process frames
    while (ros::ok() && igb.GrabImage()) {
        ros::spinOnce(); // Process ROS callbacks
    }

    // Release the camera
    cap.release();
    cv::destroyAllWindows();

    // Stop ORB-SLAM3 threads
    SLAM.Shutdown();

    // Optionally save the trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();
    return 0;
}