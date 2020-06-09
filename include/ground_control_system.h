#ifndef _HHI_GROUND_CONTROL_SYSTEM_H_
#define _HHI_GROUND_CONTROL_SYSTEM_H_
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream> // for lidar pcd files

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
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// custom msgs
// ref: https://steemit.com/kr-dev/@jacobyu/1303-ros-custom-message-generation
#include "hhi_autoexcavator/hhi_command.h" // dedicated msgs for HHI project.

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
    void saveLidarData(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_lidar);
    void saveAllData();

    // related to camera settings.
    void setExposureTime();
    void setCameraGrayscaleGain();

    
private:
    // node handler
    ros::NodeHandle nh_;

    // publishers
    ros::Publisher pub_cmd_msg_;
    std_msgs::Int32 cmd_msg_; // user command msg.
    
    // subscribers
    image_transport::ImageTransport it_;
    vector<image_transport::Subscriber> subs_imgs_; // from mvBlueCOUGAR-X cameras
    ros::Subscriber sub_timestamp_; // from arduino.
    vector<ros::Subscriber> subs_lidars_; // from Velodyne lidars

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
    pcl::PointCloud<pcl::PointXYZI>::Ptr* buf_lidars_; // point clouds (w/ intensity) from Velodyne VLP16 


    // private methods
    void initializeAllFlags();
    void callbackImage(const sensor_msgs::ImageConstPtr& msg, const int& id);
    void callbackLidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id);
    void callbackTime(const sensor_msgs::TimeReference::ConstPtr& t_ref);
    void clearCmdMsg(){cmd_msg_.data = 0; };

    string save_dir_;
    int current_seq_;
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
    if(n_lidars_ > 0){
        buf_lidars_ = new pcl::PointCloud<pcl::PointXYZI>::Ptr[n_lidars_];
        flag_lidars_ = new bool[n_lidars_];
        for(int i = 0; i < n_lidars_; i++){
            flag_lidars_[i] = false;
            *(buf_lidars_ + i) = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            string name_temp = "/velodyne_points";
            topicnames_lidars_.push_back(name_temp);
            subs_lidars_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(topicnames_lidars_[i], 1, boost::bind(&HHIGCS::callbackLidar, this, _1, i)));
        }
    }
    else{
        buf_lidars_  = nullptr;
        flag_lidars_ = nullptr;
    }
    
    // initialize arduino container & subscriber.
    flag_mcu_ = false;
    buf_time_ = -1.0;
    sub_timestamp_ = nh_.subscribe("/trigger_time", 1, &HHIGCS::callbackTime, this);


    // generate save folder
    std::string folder_create_command;
    folder_create_command = "sudo rm -rf " + save_dir_;
	system(folder_create_command.c_str());
    folder_create_command = "mkdir " + save_dir_;
	system(folder_create_command.c_str());

    // make image saving directories
    for(int i = 0; i< n_cams_; i++){
        folder_create_command = "mkdir " + save_dir_ + "cam" + itos(i) + "/";
	    system(folder_create_command.c_str());
    }

    // make lidar data saving directories
    for(int i = 0; i < n_lidars_; i++){
        folder_create_command = "mkdir " + save_dir_ + "lidar" + itos(i) + "/";
	    system(folder_create_command.c_str());
    }
};

HHIGCS::~HHIGCS() {
    // ! all allocation needs to be freed.
    if( buf_imgs_ != nullptr ) delete[] buf_imgs_;
    if( flag_imgs_ != nullptr ) delete[] flag_imgs_;

    if( buf_lidars_ != nullptr) delete[] buf_lidars_;
    if( flag_lidars_ != nullptr) delete[] flag_lidars_;
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

void HHIGCS::callbackImage(const sensor_msgs::ImageConstPtr& msg, const int& id){
    cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	*(buf_imgs_ + id) = cv_ptr->image;

    cout << "  GCS get! [" << id << "] image.\n";
    flag_imgs_[id] = true;
};

void HHIGCS::callbackLidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id){
    msg_lidar->header.stamp; // timestamp

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...
    output = *msg_lidar;

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp = *(buf_lidars_ + id);
    temp->clear();
    pcl::fromROSMsg(output, *temp);

    int n_pts = temp->points.size();
    cout <<"n_pts lidar: " <<n_pts<<endl;
    flag_lidars_[id] = true;
};

void HHIGCS::callbackTime(const sensor_msgs::TimeReference::ConstPtr& t_ref){
    buf_time_ = (double)t_ref->header.stamp.sec + (double)t_ref->header.stamp.nsec/(double)1000000000.0;
    current_seq_ = t_ref->header.seq;

    cout << "  GCS get! [" << buf_time_ <<"] time ref."<<" seg: " << current_seq_ << "\n";
    flag_mcu_ = true;
};

void HHIGCS::saveLidarData(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_lidar){
    int n_pts = pc_lidar->points.size();

    std::ofstream output_file(file_name, std::ios::trunc);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);

    if(output_file.is_open()){
        output_file << "# test saving!\n";
        output_file << "# .PCD v.7 - Point Cloud Data file format\n";
        output_file << "VERSION .7\n";
        output_file << "FIELDS x y z Intensity\n";
        output_file << "SIZE 8 8 8 8\n";
        output_file << "TYPE F F F F\n";
        output_file << "COUNT 1 1 1 1\n";
        output_file << "WIDTH " << n_pts << "\n";
        output_file << "HEIGHT 1\n";
        output_file << "VIEWPOINT 0 0 0 1 0 0\n";
        output_file << "POINTS " << n_pts<< "\n";
        output_file << "DATA ascii\n";
        for(int i = 0; i < n_pts; i++){
            output_file << pc_lidar->points[i].x<<" ";
            output_file << pc_lidar->points[i].y<<" ";
            output_file << pc_lidar->points[i].z<<" ";
            output_file << pc_lidar->points[i].intensity<<"\n";
        }
    }
    
};

void HHIGCS::saveAllData(){
    // initialize folder directory
    bool static png_param_on = false;
	vector<int> static png_parameters;
	if (png_param_on == false)
	{
		png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
		png_parameters.push_back(0);
		png_param_on = true;
	}
    for(int id = 0; id < n_cams_; id++){
        string file_name = save_dir_ + "/cam" + itos(id) + "/" + itos(current_seq_) + ".png";
	    cv::imwrite(file_name, *(buf_imgs_ + id), png_parameters);
    };

    for(int id = 0; id <n_lidars_; id++){
        string file_name = save_dir_ + "/lidar" + itos(id) + "/" + itos(current_seq_) + ".pcd";
        saveLidarData(file_name, *(buf_lidars_ + id));
    }
};



#endif