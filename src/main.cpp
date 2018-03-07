/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <iostream>

// ROS Libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"

ros::Publisher pubIMU, pubMag, pubGPS, pubTemp, pubPres;


// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

std::string frame_id;

int main(int argc, char *argv[])
{

  // ROS node init
  ros::init(argc, argv, "vectornav");
  ros::NodeHandle n;
  pubIMU = n.advertise<sensor_msgs::Imu>("/Imu", 1000);
  pubGPS = n.advertise<sensor_msgs::NavSatFix>("/GPS", 1000);

  n.param<std::string>("frame_id", frame_id, "vectornav");

  // Serial Port Settings
	string SensorPort;	
	int SensorBaudrate;
	
	n.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
	n.param<int>("serial_baud", SensorBaudrate, 115200);
	
  ROS_INFO("Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

	// Create a VnSensor object and connect to sensor
	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	// Query the sensor's model number.
	string mn = vs.readModelNumber();	
  ROS_INFO("Model Number: %s", mn.c_str());
  mat3f reference_rotation = mat3f(0.9997205,0.0166264,0.0168087,
                                   -0.0169886,0.9996215,0.0216404,
                                   0.0216404,-0.0219199,0.9996245);

  vs.writeReferenceFrameRotation(reference_rotation,true);
  //vs.writeSettings(false);
  ros::Duration(0.5).sleep(); 
  //vs.reset(false);
	// Set Data output Freq [Hz]
  ros::Duration(0.5).sleep(); 
	int async_output_rate;
	n.param<int>("async_output_rate", async_output_rate, 40);
	vs.writeAsyncDataOutputFrequency(async_output_rate);
  
  
	// Configure binary output message
	BinaryOutputRegister bor(
		ASYNCMODE_PORT1,
		1000 / async_output_rate,  // update rate [ms]
		COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE);

	vs.writeBinaryOutput1(bor);
	vs.registerAsyncPacketReceivedHandler(NULL, BinaryAsyncMessageReceived);


  // You spin me right round, baby
  // Right round like a record, baby
  // Right round round round
  ros::spin();


  // Node has been terminated
	vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();
	return 0;
}


//
// Callback function to process data packet from sensor
//
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
	
	if (p.type() == Packet::TYPE_BINARY)
	{
		// First make sure we have a binary packet type we expect since there
		// are many types of binary output types that can be configured.
		if (!p.isCompatible(
			COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE  | COMMONGROUP_ACCEL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
			GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE))
			// Not the type of binary packet we are expecting.
			return;


		// Unpack the packet
		vec4f q = p.extractVec4f();
		vec3f ar = p.extractVec3f();
		vec3f al = p.extractVec3f();

		
		// Publish ROS Message
		
		// IMU
    sensor_msgs::Imu msgIMU;
		
		msgIMU.header.stamp = ros::Time::now();
		msgIMU.header.frame_id = frame_id;
		
		msgIMU.orientation.x = q[0];
		msgIMU.orientation.y = q[1];
		msgIMU.orientation.z = q[2];
		msgIMU.orientation.w = q[3];
		
		msgIMU.angular_velocity.x = ar[0];
		msgIMU.angular_velocity.y = ar[1];
		msgIMU.angular_velocity.z = ar[2];
	
		msgIMU.linear_acceleration.x = al[0];
		msgIMU.linear_acceleration.y = al[1];
		msgIMU.linear_acceleration.z = al[2];
		
    pubIMU.publish(msgIMU);
    
    
	}
}

