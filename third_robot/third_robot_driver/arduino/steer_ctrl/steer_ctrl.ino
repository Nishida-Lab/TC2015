/**
Copyright (c) 2015, Yusuke Doi
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*/
#include <ros.h> // Use ros_lib.
#include <geometry_msgs/Twist.h> // Use Twist.msg. -> [linear(x,y,z), angular(x,y,z)]

/* Define pin numbers. */
#define ACCEL 9
#define BACK_GEAR 8
#define CW 3
#define CCW 4
/* Define pulse width time of the stepping motor. */
#define PULSE_WIDTH_MICRO_SECOND 500
/* Define max speed to 220. */
#define MAX_SPEED 220
/* Define direction of angular z. */
#define LEFT 0
#define RIGHT 1
#define KEEP 2
/* Define direction of back gear. */
#define FORWARD 0
#define BACK 1

const int cw_plus = 3;
const int cw_minus = 4;
const int ccw_plus = 5;
const int ccw_minus = 6;

/* Declare proto type functions. */
void gen_pulse(const char direction); // Write pulse to the stepping motor.
void steerCb(const geometry_msgs::Twist& msg); // Call back function of the motor_driver topic.

/* Declare global constants. */
const unsigned int PULSE_FREQUENCY = 1000l * 1000 / (PULSE_WIDTH_MICRO_SECOND * 2); // Get frequency of pulse.
/* Declare global variables. */
ros::NodeHandle nh; // The nodeHandle.
ros::Subscriber<geometry_msgs::Twist> sub("steer_ctrl", &steerCb); // Set subscribe the motor_driver topic.

void setup() {
  /* Set pins Mode. */
  for (int i = cw_plus; i < ccw_minus + 1; i++) { // 3: CW+, 4: CW-, 5: CCW+, 6:CCW-
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW); // Default pin status is LOW.
  }
  /* Node handle setting. */
  nh.initNode(); // First setup the node handle.
  nh.subscribe(sub); // Start subscribe the "steer_ctrl" topic.
}

void loop() {
  nh.spinOnce(); // Check topic and if change it, run the call back function.
}

/**
 * Call back function of the motor_driver topic.
 * This funcion is moter driver unit.
 *
 * @author "Yusuke Doi"
 * @param msg This param is msg object of the motor_driver topic.
 */
void steerCb(const geometry_msgs::Twist& msg) {
  if (msg.angular.z < 0) gen_pluse(LEFT); // Minus mean CCW or right.
  else if (msg.angular.z > 0) gen_pluse(RIGHT); // Plus mean CW or left.
  else gen_pluse(KEEP); // Zero mean keep steer.
   /* move task */
}

/**
 * Steering function of the stepping motor.
 *
 * @author "Yusuke Doi"
 * @param direction Steer to direction.
 */
void gen_pluse(const char direction) {
  switch (direction) {
  case LEFT: // Task steer left.
      noTone(cw_plus); // Unset tone. If don't running tone, not happen.
      tone(ccw_plus, PULSE_FREQUENCY, 200); // Write pulse to CCW pin. turn to CW.
      break;
  case RIGHT: // Task steer right.
      noTone(ccw_plus); // Unset tone. If don't running tone, not happen.
      tone(cw_plus, PULSE_FREQUENCY, 200); // Write pulse to CW pin. turn to CCW.
      break;
  case KEEP: default: // set CW and CCW to low.
      noTone(cw_plus); // Unset tone. If don't running tone, not happen.
      noTone(ccw_plus); // Unset tone. If don't running tone, not happen.
      break;
  }
}

