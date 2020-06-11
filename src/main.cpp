#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>

#include "ground_control_system.h"

// keyboard input tool
#include "keyinput.h"

#include <dynamic_reconfigure/server.h>
#include <hhi_autoexcavator/paramReconfigDynConfig.h>


void callback(hhi_autoexcavator::paramReconfigDynConfig &config, uint32_t level) {
      ROS_INFO("Dynamic parameter: %d", config.test_param);
}

// Get current data/time, format is yyyy-mm-dd.hh:mm:ss
const std::string currentDateTime(){
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about data/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H_%M_%S", &tstruct);

    return buf;
}

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "hhi_gcs");
    ros::NodeHandle nh("~");

    int n_cams   = -1;
    int n_lidars = -1;
    string dir;    
    ros::param::get("~n_cameras", n_cams);
    ros::param::get("~n_lidars", n_lidars);
    ros::param::get("~save_directory", dir);
    
    
    // Ground control system class
    
    stringstream ss1;
    ss1 << dir << currentDateTime() << "/";
    string save_dir = ss1.str();
    cout << "save directory:[" << save_dir << "]\n";

    HHIGCS* gcs = new HHIGCS(nh, n_cams, n_lidars, save_dir);


    dynamic_reconfigure::Server<hhi_autoexcavator::paramReconfigDynConfig> server;
    dynamic_reconfigure::Server<hhi_autoexcavator::paramReconfigDynConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    cout << "GCS: waiting sensor nodes for 6 seconds...\n";
    ros::Duration(6.0).sleep();

    // user input manual.
    string user_manual;
    stringstream ss;
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|    i: get data & start all algorithms"
    << "\n|    f: continuous shot mode"
    << "\n|    s: query & save one scene" 
    << "\n|    q: cease the program"
    << "\n|    m: camera parameter configuration mode"
    << "\n|  Select an input: \n";
    user_manual = ss.str();
    cout << user_manual;

    string cam_configure_manual;
    ss.clear();
    ss.flush();

    ss << "\n |\n L\n";

    int cnt = 0;
    while(ros::ok())
    {
        // no need 'spinOnce()' for all loop!!
        int c = getch(); // call my own non-blocking input function
        if(c == 's') {
            cout << "\n\n[Operation]: snapshot & save the current scene.\n";

            // send single query to all sensors.
            bool is_query_ok = gcs->sendSingleQueryToAllSensors();
            
            // Save all data
            if(is_query_ok){
                // save and purge the current data!
                gcs->saveAllData();
            }
            else {
                cout << "   fail to save...\n";
            }
            cout << user_manual;
        }
        else if(c == 'i') {
            cout << "\n\n[Operation]: All algorithm.\n";
            
            // send single query to all sensors.

            // TODO!

            // Do something here! (3-D recon -> path planning)
            cout << user_manual;
        }
        else if(c == 'q') {
            gcs->sendQuitMsgToAllSensors();
            cout << "\n\n[Operation]: quit program\n\n";
            break;
        }
        else if(c == 'f'){
            cout << "\n\n[Operation]: free running... press 'f' to stop streaming.\n\n";
            cv::namedWindow("control", CV_WINDOW_NORMAL);
            for(int i = 0; i < gcs->getNumCams(); i++){
                string win_name = "img" + itos(i);
                cv::namedWindow(win_name,CV_WINDOW_NORMAL);
            }

            int expose_us = 0;
            cv::createTrackbar("exposure_us", "control", &expose_us, 5000000);

            int cc = 0;
            while(1){
                gcs->streamingMode();
                for(int i = 0; i < gcs->getNumCams(); i++){
                    string win_name = "img" + itos(i);
                    cv::Mat a =gcs->getBufImage(i);
                    cv::imshow(win_name, a);
                }

                if(cv::waitKey(1) == 'f'){
                    cv::destroyAllWindows();
                    break;
                }
            }
            cout << " streaming stops.\n";
            cout << user_manual;
        }
        else if((c != 0)) {
            cout << ": Un-identified command...\n";
            cout << user_manual;
        }       
    }

// delete allocation.
delete gcs;

ROS_INFO_STREAM("End of the program.\n");
return -1;
}
