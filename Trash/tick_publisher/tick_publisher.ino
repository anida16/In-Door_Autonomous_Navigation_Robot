
#include <QuadratureEncoder.h>
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

// must also have enableInterrupt.h library

// Use any 2 pins for interrupt, this utilizes EnableInterrupt Library.
// Even analog pins can be used. A0 = 14,A1=15,..etc for arduino nano/uno

// Max number of Encoders object you can create is 4. This example only uses 2.

Encoders leftEncoder(2, 3); // Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders rightEncoder(18, 19); // Encoder object name rightEncoder using analog pin A0 and A1

std_msgs::Int32 currentRightEncoderCount;
ros::Publisher rightPub("right_ticks", &currentRightEncoderCount);

std_msgs::Int32 currentLeftEncoderCount;
ros::Publisher leftPub("left_ticks", &currentLeftEncoderCount);

void setup() {
  //Serial.begin(115200);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
}


unsigned long lastMilli = 0;

void loop() {
  // put your main code here, to run repeatedly:
  // print encoder count every 50 millisecond
  if (millis() - lastMilli > 50) {

    long currentLeftEncoderCount = leftEncoder.getEncoderCount();
    long currentRightEncoderCount = rightEncoder.getEncoderCount();

    rightPub.publish(&currentRightEncoderCount);
    leftPub.publish(&currentRightEncoderCount);
    nh.spinOnce();
    
    //Serial.print(leftEncoder.getEncoderCount());
    //Serial.print(" , ");
    //Serial.println(rightEncoder.getEncoderCount());

    lastMilli = millis();
  }

}
