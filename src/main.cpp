#include <iostream>
#include <termios.h> // UNIX LINUX.
#include <ros/ros.h>

#include "keyinput.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// custom msgs
// ref: https://steemit.com/kr-dev/@jacobyu/1303-ros-custom-message-generation
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
    
    // publish message topic to control all module nodes.
    ros::Publisher pub_hhi_msgs = 
        nh.advertise<hhi_autoexcavator::hhi_msgs>("/hhi/msg",1);
    
    // 
    string common_phrase =
    "\n==================\n Press a key...\n   i: query one scene.\n   q: cease the program \n Select an input: \n";
    cout << common_phrase;
    while(ros::ok())
    {
        int c = getch(); // call my own non-blocking input function
        if(c == 'i'){
            cout << "\n\n snapshot the current scene.\n";
            // Do something here!

            cout << common_phrase;
        }
        else if(c == 'q'){
            cout <<"\n\n quit program\n";
            return -1;
        }
        else if((c != 0))
        {
            cout << ": unknown command...\n";
            cout << common_phrase;
        }

        ros::spinOnce();
    }

ROS_INFO_STREAM("GCS off\n");
return -1;
}
