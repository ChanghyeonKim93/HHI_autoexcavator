#ifndef _HHI_GROUND_CONTROL_SYSTEM_H_
#define _HHI_GROUND_CONTROL_SYSTEM_H_
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

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

#include <std_msgs/Int32.h> // command msg
#include <sensor_msgs/TimeReference.h> // arduino time

// custom msgs
// ref: https://steemit.com/kr-dev/@jacobyu/1303-ros-custom-message-generation
#include "std_msgs/Int32.h"
#include "hhi_autoexcavator/hhi_msgs.h" // dedicated msgs for HHI project.

using namespace std;

string dtos(double x)
{
	stringstream s;
	s << setprecision(6) << fixed << x;
	return s.str();
};

string itos(double x)
{
	stringstream s;
	s << x;
	return s.str();
};

class HHIGCS{
public:
    HHIGCS(ros::NodeHandle& nh, int n_cams, int n_lidars, const string& save_dir);
    ~HHIGCS();
    bool sendSingleQueryToAllSensors();
    void sendQuitMsgToAllSensors();
    void saveAllData();

private:
    // node handler
    ros::NodeHandle nh_;

    // publishers
    ros::Publisher pub_cmd_msg_;
    std_msgs::Int32 cmd_msg_; // user command msg.
    
    // subscribers
    image_transport::ImageTransport it_;
    vector<image_transport::Subscriber> subs_imgs_;
    ros::Subscriber sub_timestamp_; // from arduino.

    // topic names
    vector<string> topicnames_imgs_;
    vector<string> topicnames_lidars_;
    string topicname_timestamp_;

    // state variables
    int n_cams_; // numbering rule(cams) 0,1) cabin left,right, 2,3) boom frontal,rear
    int n_lidars_;// numbering rule(lidars)- 0) cabin, 1) boom

    // transmittion flags.
    bool* flag_imgs_; // 'true' when image data is received.
    bool* flag_lidars_; // 'true' when lidar data is received.
    bool flag_mcu_; // 'true' when arduino data is received. 

    // data container (buffer)
    cv::Mat* buf_imgs_; // Images from mvBlueCOUGAR-X cameras.
    double buf_time_; // triggered time stamp from Arduino. (second)
    

    // private methods
    void initializeAllFlags();
    void callbackImage(const sensor_msgs::ImageConstPtr& msg, const int id);
    void callbackTime(const sensor_msgs::TimeReference::ConstPtr& t_ref);
    void clearCmdMsg(){cmd_msg_.data = 0; };

    string save_dir_;
};


HHIGCS::HHIGCS(ros::NodeHandle& nh,
    int n_cams, int n_lidars, const string& save_dir)
: nh_(nh), it_(nh_), n_cams_(n_cams), n_lidars_(n_lidars),save_dir_(save_dir)
{
    // default.
    flag_lidars_ = nullptr;
    flag_imgs_ = nullptr;
    buf_imgs_ = nullptr;

    // command message publisher.
    pub_cmd_msg_ = nh_.advertise<std_msgs::Int32>("/hhi/msg",1);

    // initialize image container & subscribers.
    if(n_cams_ > 0){
        buf_imgs_  = new cv::Mat[n_cams_];
        flag_imgs_ = new bool[n_cams_];
        for(int i = 0; i < n_cams_; i++) {
            flag_imgs_[i] = false;
            string name_temp = "/" + itos(i) + "/image_raw";
            topicnames_imgs_.push_back(name_temp);
            subs_imgs_.push_back(it_.subscribe(topicnames_imgs_[i], 1, boost::bind(&HHIGCS::callbackImage, this, _1, i)));
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
    buf_time_ = -1.0;
    sub_timestamp_ = nh_.subscribe("/trigger_time", 1, &HHIGCS::callbackTime, this);

};

HHIGCS::~HHIGCS() {
    // ! all allocation needs to be freed.
    if( buf_imgs_ != nullptr ) delete[] buf_imgs_;
    if( flag_imgs_ != nullptr ) delete[] flag_imgs_;
};

bool HHIGCS::sendSingleQueryToAllSensors()
{   
    // initialize all flags
    initializeAllFlags();

    // fill out control msg
    cmd_msg_.data = 1;

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
        for(int i = 0; i < n_cams_; i++)   
            transmit_success = transmit_success & flag_imgs_[i];
    if(flag_lidars_ != nullptr)
        for(int i = 0; i < n_lidars_; i++) 
            transmit_success = transmit_success & flag_lidars_[i];
    transmit_success = transmit_success & flag_mcu_;

    if(transmit_success) cout <<" Transmission successes!\n";
    else cout << "Fail to transmit! Please retry...\n";

    return transmit_success;
};

void HHIGCS::initializeAllFlags(){
    for(int i = 0; i < n_cams_; i++) flag_imgs_[i] = false;
    for(int i = 0; i < n_lidars_; i++) flag_lidars_[i] = false;
    flag_mcu_ = false;
};

void HHIGCS::sendQuitMsgToAllSensors(){
    cmd_msg_.data = -1;
    pub_cmd_msg_.publish(cmd_msg_);
    clearCmdMsg();
};

void HHIGCS::callbackImage(const sensor_msgs::ImageConstPtr& msg, const int id){
    cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	*(buf_imgs_ + id) = cv_ptr->image;
    cout << "  GCS get! [" << id << "] image.\n";
    flag_imgs_[id] = true;
};

void HHIGCS::callbackTime(const sensor_msgs::TimeReference::ConstPtr& t_ref){
    buf_time_ = (double)t_ref->header.stamp.sec + (double)t_ref->header.stamp.nsec/(double)1000000000.0;
    int seq = t_ref->header.seq;

    cout << "  GCS get! [" << buf_time_ <<"] time ref."<<" seg: " << seq << "\n";
    flag_mcu_ = true;
};

void HHIGCS::saveAllData(){
    // initialize folder directory
    std::string folder_create_command;
    folder_create_command = "sudo rm -rf " + save_dir_;
	system(folder_create_command.c_str());
    folder_create_command = "mkdir " + save_dir_;
	system(folder_create_command.c_str());

    bool static png_param_on = false;
	vector<int> static png_parameters;
	if (png_param_on == false)
	{
		png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
		png_parameters.push_back(0);
		png_param_on = true;
	}
	string file_name = save_dir_ + "a.png";
	//cv::imwrite(file_name, img, png_parameters);
	//this->file_single_image << curr_time << " " << curr_time << ".png" << "\n"; // association save
};



#endif