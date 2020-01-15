/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "hcrobot_motor_driver.h"

DFRobotVeyronBrushed HCRVeyronBrushed;

HCRMotorDriver::HCRMotorDriver()
{
}

HCRMotorDriver::~HCRMotorDriver()
{
  close();
}

bool HCRMotorDriver::init()
{
  DEBUG_SERIAL.begin(57600);
  Serial2.begin(57600);
  HCRVeyronBrushed.begin(Serial2);

  String VeyronType = HCRVeyronBrushed.readType(1);   //Read the type of the Veyron
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print(VeyronType);
  
  //Open the DEBUG_SERIAL monitor.
  if (VeyronType.startsWith("BDC")) {
    //If "BDC" is received, the connection is working.
    //DEBUG_SERIAL.println("VeyronType_Left Connection Good!");
  }
  else{
    //If "Time Out!" is received, please check the connection.
    //DEBUG_SERIAL.println("VeyronType Connection Wrong!");
    //DEBUG_SERIAL.println("Recheck the electrical connection?");
    //DEBUG_SERIAL.println("Change the baudrate?");
    //DEBUG_SERIAL.println("Change the ID?");
    //DEBUG_SERIAL.println("This is not DFRobot Veyron Brushed motor controller?");
    return false;
  }
  HCRVeyronBrushed.clearEncoderPulses(1, 1);//The id of the Veyron module, left
  HCRVeyronBrushed.clearEncoderPulses(1, 2);//The id of the Veyron module, right

  limit_max_velocity = 265;

  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

void HCRMotorDriver::close(void)
{
  // Close port
  Serial1.end();
  DEBUG_SERIAL.end();
}

bool HCRMotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  left_value = atoi(HCRVeyronBrushed.readEncoderPulses(1, 1));
  right_value = atoi(HCRVeyronBrushed.readEncoderPulses(1, 2));
  return true;
}

bool HCRMotorDriver::writeVelocity(int32_t left_value, int32_t right_value)
{
  
  //set conditional speed
  HCRVeyronBrushed.setConditionalSpeed(1, 1, left_value); //id, left, speed
  HCRVeyronBrushed.setConditionalSpeed(1, 2, right_value); //id, right, speed

  //use conditional speed to run the motor
  //boolean isStartConditionalSpeedM1 = true;
  //boolean isStartConditionalSpeedM2 = true;
  HCRVeyronBrushed.startConditionalSpeed(1, true,  true);

  return true;
}

bool HCRMotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{ 
  float wheel_velocity_cmd[2];

  float lin_vel = value[LEFT];
  float ang_vel = value[RIGHT];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -limit_max_velocity, limit_max_velocity);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -limit_max_velocity, limit_max_velocity);

  return writeVelocity((int32_t)wheel_velocity_cmd[LEFT], (int32_t)wheel_velocity_cmd[RIGHT]);

  //return true;
}
