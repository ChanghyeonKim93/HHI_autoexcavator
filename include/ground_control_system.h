#ifndef _HHI_GROUND_CONTROL_SYSTEM_H_
#define _HHI_GROUND_CONTROL_SYSTEM_H_

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// for subscribe
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// custom msgs
// ref: https://steemit.com/kr-dev/@jacobyu/1303-ros-custom-message-generation
#include "hhi_autoexcavator/hhi_msgs.h" // dedicated msgs for HHI project.

using namespace std;

class HHIGCS{
public:
    HHIGCS(ros::NodeHandle& nh, 
        int n_cams_cabin, int n_lidars_cabin,
        int n_cams_arm, int n_lidars_arm);
    ~HHIGCS();
    
    void sendSingleQueryToAllSensors();
    void saveAllData();

private:
    // node handler
    ros::NodeHandle nh_;

    // publishers
    ros::Publisher pub_cmd_msg_;
    hhi_autoexcavator::hhi_msgs cmd_msg_; // user command msg.
    
    // subscribers


    // state variables
    int n_cams_cabin_;
    int n_cams_arm_;

    int n_lidars_cabin_;
    int n_lidars_arm_;

    // data container
    cv::Mat* imgs_cabin;
    cv::Mat* imgs_arm;     

    // private methods
    void clearCmdMsg(){cmd_msg_.id = 0; cmd_msg_.flag_query = false; };
    
};

HHIGCS::HHIGCS(ros::NodeHandle& nh,
    int n_cams_cabin, int n_lidars_cabin,
    int n_cams_arm, int n_lidars_arm)
: nh_(nh),
 n_cams_cabin_(n_cams_cabin), n_cams_arm_(n_cams_arm),
 n_lidars_cabin_(n_lidars_cabin), n_lidars_arm_(n_lidars_arm)
{
    pub_cmd_msg_ = nh_.advertise<hhi_autoexcavator::hhi_msgs>("/hhi/msg",1);
    
    // initialize image container.
    if(n_cams_cabin > 0){
        imgs_cabin = new cv::Mat[n_cams_cabin];
    } 
    else {
        imgs_cabin = nullptr;
    }
    if(n_cams_arm > 0){
        imgs_arm = new cv::Mat[n_cams_arm];
    } 
    else {
        imgs_arm = nullptr;
    }

    // informations
};

// ! all allocation needs to be freed.
HHIGCS::~HHIGCS()
{
    if( imgs_cabin != nullptr ) delete[] imgs_cabin;
    if( imgs_arm != nullptr ) delete[] imgs_arm;
};

void HHIGCS::sendSingleQueryToAllSensors()
{   
    // fill out control msg
    cmd_msg_.id = 0;
    cmd_msg_.name = "GCS_COMMAND";
    cmd_msg_.flag_query = true;

    pub_cmd_msg_.publish(cmd_msg_);

    clearCmdMsg();
};

void HHIGCS::saveAllData(){

};



#endif