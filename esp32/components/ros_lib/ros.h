/*
* ros.h
*/
#ifndef _ROS_H_
#define _ROS_H_
#include "ros/node_handle.h"
//9 #include "HardwareImpl.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

class ROSSerialHWImpl
{
public:
 ROSSerialHWImpl();

 // any initialization code necessary to use the serial port
 void init(char*&);
 // read a byte from the serial port. -1 = failure
 int read();

 bool ok()
 {
   return connected;
 }

 // write data to the connection to ROS
 void write(uint8_t* data, int length);

 // returns milliseconds since start of program
 unsigned long time();

 bool tryConnect();

private:
   int sock;
   bool connected;

   struct sockaddr_in destAddr;
   char roscorehostname[32];
   unsigned char rxfifo[1024];
   unsigned int rxfifo_used;
   unsigned int rxfifo_rdidx;
};

 namespace ros
 {
   typedef ros::NodeHandle_<ROSSerialHWImpl> NodeHandle;
 }

 #endif
