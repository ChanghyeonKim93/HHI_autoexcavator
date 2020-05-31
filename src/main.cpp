#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// custom msgs
#include "hhi_autoexcavator/hhi_msgs.h" // dedicated msgs for HHI project.

#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <mutex>
#include <condition_variable> // for cv.

using namespace std;

int main(int argc, char **argv) {
    std::cout << "hhi_gcs is running..." << std::endl;
    ros::init(argc, argv, "hhi_gcs");
    ros::NodeHandle nh("~");
    
    ros::Publisher pub_hhi_msgs = 
        nh.advertise<hhi_autoexcavator::hhi_msgs>("/hhi/msg",1);
    
    while(ros::ok()){

        ros::spinOnce();
    }

ROS_INFO_STREAM("GCS off\n");
return -1;
}
