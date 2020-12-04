/* This intend to connect to an Arduino Ethernet Shield
 * and a rosserial socket server.
 * You can launch the rosserial socket server with
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 * We set default IP for Arduino as 192.168.0.76.
 */ 
// rosrun rosserial_python serial_node.py tcp
// TCP/IP- https://github.com/ros-drivers/rosserial/blob/dd76994c67c5e4997ef64837c07afb4eb0d4df27/rosserial_arduino/src/ros_lib/examples/TcpHelloWorld/TcpHelloWorld.ino#L15
// TIMER- http://www.hardcopyworld.com/gnuboard5/bbs/board.php?bo_table=lecture_pract&wr_id=12
// ROS cam and IMU- http://grauonline.de/wordpress/?page_id=1951
#include <SPI.h>
#include <Ethernet.h>

// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/TimeReference.h>

// Setup a trigger digitalOutput pin & shield settings
#define PIN_TRIGGER 2
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,0,76);
IPAddress server(192,168,0,7);
const uint16_t serverPort = 11411; // rosserial socket server port, 11411

//
//
//
//

// timestamp when the trigger signal is fired.
bool trigger_state = false;
volatile unsigned long trigger_time = 0;
unsigned long time_sec = 0;
unsigned long time_nsec = 0;
// the number of firing trigger pin. 
// It can be used as an 'identifier' for each query.
volatile unsigned long triggerCounter = 0;

// node handler
ros::NodeHandle nh;

// publisher for timestamp
sensor_msgs::TimeReference msg_triggertime;
ros::Publisher pub_triggertime("/trigger_time", &msg_triggertime);

// subscriber for PC command (trigger request ...)
void command_callback(const std_msgs::Int32& cmd_msg){
    // msg "trg_on": trigger all, otherwise: unidentified
    if( cmd_msg.data == 1) {
      // Log timestamp.
      Serial.println("on! ");
      trigger_time = micros(); // microseconds
      time_sec  = trigger_time/1000000;
      time_nsec = trigger_time - time_sec*1000000;
      ++triggerCounter;

      // activate trigger signal      
      digitalWrite(PIN_TRIGGER, HIGH);
      delayMicroseconds(200); // 200 us delay.
      digitalWrite(PIN_TRIGGER, LOW); // is it okay?    
        
      msg_triggertime.header.seq = triggerCounter;
      msg_triggertime.header.stamp.sec  = time_sec;
      msg_triggertime.header.stamp.nsec = time_nsec;
      msg_triggertime.header.frame_id = "MCU_arduino";
      pub_triggertime.publish( &msg_triggertime );
    } 
    else {
      Serial.println("? unknown command.");
      digitalWrite(PIN_TRIGGER, LOW); // stay low.
    }
}
ros::Subscriber<std_msgs::Int32> sub_command("/hhi/msg",command_callback);


void setup()
{
  // Use serial to monitor the process
  Serial.begin(115200);
  
  // Setup pin
  digitalWrite(PIN_TRIGGER, LOW);  //  drive it low without temporarily driving it high
  pinMode(PIN_TRIGGER, OUTPUT);

  // Connect the Ethernet
  Ethernet.begin(mac, ip);

  // Let some time for the Ethernet Shield to be initialized
  delay(1000);

  Serial.println("");
  Serial.println("Ethernet connection info...");
  Serial.println("IP address: ");
  Serial.println(Ethernet.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("MY IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Setup subscriber
  nh.subscribe(sub_command);
  
  // Start to be polite
  nh.advertise(pub_triggertime);
}

void loop() {
  if (nh.connected()) {  
    //Serial.println("Connected");
  } 
  else {
    //Serial.println("TCP/IP connection is lost!");
  }
  nh.spinOnce();
  delay(1); // it need to be sufficiently low. 
  //Error message: Lost sync with device, restarting...
  // Solution: Do not use any works in loop.
}
