#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>

#include "ground_control_system.h"

// keyboard input tool
#include "keyinput.h"

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
    
    // Ground control system class
    int n_cams   = 2;
    int n_lidars = 0;
    stringstream ss;
    ss << "~/hhi_data/" << currentDateTime() << "/";
    string save_dir = ss.str();
    cout << "save directory:[" << save_dir << "]\n";

    HHIGCS* gcs = new HHIGCS(nh, n_cams, n_lidars, save_dir);

    cout << "GCS: waiting sensor nodes for 6 seconds...\n";
    ros::Duration(6.0).sleep();

    // user input manual.
    string user_manual;
    ss.clear();
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|    i: get data & start all algorithms"
    << "\n|    s: query & save one scene." 
    << "\n|    q: cease the program"
    << "\n|    c: camera parameter configuration mode"
    << "\n|  Select an input: \n";
    user_manual = ss.str();
    cout << user_manual;

    string cam_configure_manual;
    ss.clear();

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
            system("rosnode kill arduino_TCPIP");   

            cout << user_manual;
        }
        else if(c == 'q') {
            gcs->sendQuitMsgToAllSensors();
            cout << "\n\n[Operation]: quit program\n\n";
            break;
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
