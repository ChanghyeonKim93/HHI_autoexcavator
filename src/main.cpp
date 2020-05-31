#include <iostream>
#include <ros/ros.h>

#include "ground_control_system.h"

// keyboard input tool
#include "keyinput.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "hhi_gcs");
    ros::NodeHandle nh("~");
    
    // Ground control system class
    int n_cams_cabin   = 2;
    int n_cams_arm     = 0;
    int n_lidars_cabin = 1;
    int n_lidars_arm   = 0;
    HHIGCS* gcs = new HHIGCS(nh, n_cams_cabin, n_cams_arm, n_lidars_cabin, n_lidars_arm);

    // user input manual.
    string user_manual = 
    "\n==================================\n| Press a key...\n|    i: get data & start all algorithms\n|    s: query & save one scene.\n|    q: cease the program \n|  Select an input: \n";
    cout << user_manual;

    int cnt = 0;
    while(ros::ok())
    {
        int c = getch(); // call my own non-blocking input function
        if(c == 's') {
            cout << "\n\n[Operation]: snapshot & save the current scene.\n";

            // send single query to all sensors.
            gcs->sendSingleQueryToAllSensors();

            // Wait for new snapshot data.
            
            // Save all data

            cout << user_manual;
        }
        else if(c == 'i') {
            cout << "\n\n[Operation]: All algorithm.\n";
            
            // send single query to all sensors.
            // gcs->sendSingleQueryToAllSensors();

            // Wait for new snapshot data.

            // TODO!

            // Do something here! (3-D recon -> path planning)


            cout << user_manual;
        }
        else if(c == 'q') {
            cout <<"\n\n[Operation]: quit program\n\n";
            break;
        }
        else if((c != 0)) {
            cout << ": Un-identified command...\n";
            cout << user_manual;
        }

        ros::spinOnce();
    }

// delete allocation.
delete gcs;

ROS_INFO_STREAM("End of the program.\n");
return -1;
}
