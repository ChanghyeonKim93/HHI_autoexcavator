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
    HHIGCS(ros::NodeHandle& nh, int n_cams, int n_lidars);
    ~HHIGCS();
    bool sendSingleQueryToAllSensors();
    void saveAllData();

private:
    // node handler
    ros::NodeHandle nh_;

    // publishers
    ros::Publisher pub_cmd_msg_;
    hhi_autoexcavator::hhi_msgs cmd_msg_; // user command msg.
    
    // subscribers
    image_transport::ImageTransport it_;
    vector<image_transport::Subscriber> subs_imgs_;

    vector<string> topicnames_imgs_;
    vector<string> topicnames_lidars_;

    // state variables
    // numbering rule(cams)- 0) cabin left, 1) cabin right, 2) boom frontal, 3) boom rear
    int n_cams_;
    // numbering rule(lidars)- 0) cabin, 1) boom
    int n_lidars_;

    // transmittion flags.
    bool* flag_imgs_; // 'true' when image data is received.
    bool* flag_lidars_; // 'true' when lidar data is received.
    bool flag_mcu_; // 'true' when arduino data is received. 

    // data container (buffer)
    cv::Mat* buf_imgs_;

    // private methods
    void bufferAllData();
    void clearCmdMsg(){cmd_msg_.id = 0; cmd_msg_.flag_query = false; };

};

HHIGCS::HHIGCS(ros::NodeHandle& nh,
    int n_cams, int n_lidars)
: nh_(nh), it_(nh_), n_cams_(n_cams), n_lidars_(n_lidars)
{
    // default.
    flag_lidars_ = nullptr;
    flag_imgs_ = nullptr;
    buf_imgs_ = nullptr;

    // command message publisher.
    pub_cmd_msg_ = nh_.advertise<hhi_autoexcavator::hhi_msgs>("/hhi/msg",1);

    // initialize image container & subscribers.
    if(n_cams_ > 0){
        buf_imgs_  = new cv::Mat[n_cams_];
        flag_imgs_ = new bool[n_cams_];
        for(int i = 0; i < n_cams_; i++){
            // subs_imgs_cabin.push_back(it_.subscribe<sensor_msgs::Image>());
        }
    }
    else {
        buf_imgs_  = nullptr;
        flag_imgs_ = nullptr;
    }

    // initialize lidar container & subscribers.
    flag_lidars_ = nullptr;
    
    // initialize arduino container & subscriber.
    flag_mcu_ = false;
  
    // informations
};

HHIGCS::~HHIGCS() {
    // ! all allocation needs to be freed.
    if( buf_imgs_ != nullptr ) delete[] buf_imgs_;
    if( flag_imgs_ != nullptr ) delete[] flag_imgs_;
};

bool HHIGCS::sendSingleQueryToAllSensors()
{   
    // fill out control msg
    cmd_msg_.id = 0;
    cmd_msg_.name = "GCS_COMMAND";
    cmd_msg_.flag_query = true;

    // query sensor data for all sensors
    pub_cmd_msg_.publish(cmd_msg_);
    clearCmdMsg();

    // (timeout) Wait for obtaining and transmitting all sensor data. 
    // Considering exposure time and lidar gathering time, set 50 ms
    cout << "wating 110 ms for data transmission...\n"; // because a rate of a lidar is about 10 Hz.
    ros::Duration(0.11).sleep();
    ros::spinOnce();
    
    // Check whether all data is received.
    bool transmit_success = true;
    if(flag_imgs_ != nullptr)
        for(int i = 0; i < n_cams_; i++)   transmit_success = transmit_success & flag_imgs_[i];
    if(flag_lidars_ != nullptr)
        for(int i = 0; i < n_lidars_; i++) transmit_success = transmit_success & flag_lidars_[i];
    transmit_success = transmit_success & flag_mcu_;

    if(transmit_success) cout <<" Transmission successes!\n";
    else cout << "Fail to transmit! Please retry...\n";

    return transmit_success;
};

void HHIGCS::bufferAllData(){

};

void HHIGCS::saveAllData(){

};



#endif