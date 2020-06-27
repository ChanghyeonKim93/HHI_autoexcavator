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
// for mkr zero, ethernet library must be update to new version! (later than 2.0.0)
// can bus library : 
#include <SPI.h>
#include <Ethernet.h>
#include <CAN.h>

// Union for floating-point CAN bus communication.
union FloatUnion {
  float f;
  byte bytes[4];  
};

volatile boolean flag_can = false;
long can_id;
int can_length;
FloatUnion can_float1;
FloatUnion can_float2;


// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/Int32.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sensor_msgs/TimeReference.h>


// Setup a trigger digitalOutput pin & shield settings
#define PIN_TRIGGER 6
#define CS_CANMODULE 3
#define PIN_CAN_INTERRUPT 7

byte mac[] = {  }; //physical mac address
IPAddress ip(192, 168, 1, 2);
IPAddress server(192,168,1,1);
const uint16_t serverPort = 11411; // rosserial socket server port, 11411

//
//
//
//

// timestamp when the trigger signal is fired.
bool trigger_state = false;
volatile unsigned long trigger_time = 0;
volatile unsigned long can_time = 0;
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
std_msgs::Float32MultiArray msg_canfloats;
ros::Publisher pub_canfloats("/can_floats", &msg_canfloats);

// subscriber for PC command (trigger request ...)
void command_callback(const std_msgs::Int32& cmd_msg){
    // msg "trg_on": trigger all, otherwise: unidentified
    if( cmd_msg.data == 1) {
      // Log timestamp.
      trigger_time = micros(); // microseconds
      time_sec  = trigger_time/1000000;
      time_nsec = trigger_time - time_sec*1000000;
      ++triggerCounter;

      // activate trigger signal      
      digitalWrite(PIN_TRIGGER, HIGH);
      delayMicroseconds(10); // 10 us delay.
      digitalWrite(PIN_TRIGGER, LOW); // is it okay?    
        
      msg_triggertime.header.seq = triggerCounter;
      msg_triggertime.header.stamp.sec  = time_sec;
      msg_triggertime.header.stamp.nsec = time_nsec;
      msg_triggertime.header.frame_id = "MCU_arduino";
      pub_triggertime.publish( &msg_triggertime );
      
      Serial.println("trigger on.");
    } 
    else {
      Serial.println("? unknown command.");
      digitalWrite(PIN_TRIGGER, LOW); // stay low.
    }
};
ros::Subscriber<std_msgs::Int32> sub_command("/hhi/msg",command_callback);

char dim0_label[] = "bodyroll";
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

  // start the CAN bus at 500kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
  }
  else{
    CAN.setPins(CS_CANMODULE, PIN_CAN_INTERRUPT); // about 200 us ISR signal
    Serial.println("CAN bus is initialized at 500 kbps");
  }
  // register the receive callback
  CAN.onReceive(callbackCANReceive);

  // Setup subscriber
  nh.subscribe(sub_command);
  
  // Start to be polite
  nh.advertise(pub_triggertime);


  msg_canfloats.data = (float *)malloc(sizeof(float)*2); // neccessary
  msg_canfloats.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension) * 2);

  msg_canfloats.data_length = 2;
  msg_canfloats.layout.dim_length = 0;
  msg_canfloats.layout.dim[0].label = dim0_label;
  msg_canfloats.layout.dim[0].size  = 1;
  msg_canfloats.layout.dim[0].stride = 1;
  msg_canfloats.layout.dim[1].label = dim0_label;
  msg_canfloats.layout.dim[1].size  = 1;
  msg_canfloats.layout.dim[1].stride = 1;
  nh.advertise(pub_canfloats);
}

void loop() {
  nh.spinOnce();
  delay(1); // it need to be sufficiently low. 
  //Error message: Lost sync with device, restarting...
  // Solution: Do not use any works in loop.
}

void callbackCANReceive(int packetSize) {
  // received a packet
  if (CAN.packetExtended()) {
    // Serial.print("extended ");
  }

  if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    Serial.print("RTR ");
  }

  if (CAN.packetRtr()) {
    Serial.println(CAN.packetDlc());
  } else {    
    // only print packet data for non-RTR packets
    can_time = micros(); // microseconds
    time_sec  = can_time/1000000;
    time_nsec = can_time - time_sec*1000000;
    can_id = CAN.packetId();

    while (CAN.available()) {
      can_float1.bytes[0] = CAN.read();
      can_float1.bytes[1] = CAN.read();
      can_float1.bytes[2] = CAN.read();
      can_float1.bytes[3] = CAN.read();
      can_float2.bytes[4] = CAN.read();
      can_float2.bytes[5] = CAN.read();
      can_float2.bytes[6] = CAN.read();
      can_float2.bytes[7] = CAN.read();
    }
    Serial.print("0x");
    Serial.print(CAN.packetId());
    Serial.print(",");
    Serial.print(can_float1.f);
    Serial.print(",");
    Serial.print(can_float2.f);
    Serial.println();
    
    
    msg_canfloats.data[0] = can_float1.f;
    msg_canfloats.data[1] = can_float2.f;
    pub_canfloats.publish( &msg_canfloats );
  }
}


