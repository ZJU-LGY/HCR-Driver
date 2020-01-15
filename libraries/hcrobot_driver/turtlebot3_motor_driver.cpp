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

#include "turtlebot3_motor_driver.h"

DFRobotVeyronBrushed HCRVeyronBrushed;

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: left_wheel_id_(0x01),
  right_wheel_id_(0x02)
{
  dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
}

bool Turtlebot3MotorDriver::init(String turtlebot3)
{
  DEBUG_SERIAL.begin(57600);
  Serial2.begin(57600);
  HCRVeyronBrushed.begin(Serial2);

  int id = 1;       //The id of the Veyron module
  String VeyronType = HCRVeyronBrushed.readType(id);   //Read the type of the Veyron
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print(VeyronType);
  
  //Open the DEBUG_SERIAL monitor.
  if (VeyronType.startsWith("BDC")) {
    //If "BDC" is received, the connection is working.
    DEBUG_SERIAL.println("VeyronType_Left Connection Good!");
  }
  else{
    //If "Time Out!" is received, please check the connection.
    DEBUG_SERIAL.println("VeyronType Connection Wrong!");
    DEBUG_SERIAL.println("Recheck the electrical connection?");
    DEBUG_SERIAL.println("Change the baudrate?");
    DEBUG_SERIAL.println("Change the ID?");
    DEBUG_SERIAL.println("This is not DFRobot Veyron Brushed motor controller?");
    return false;
  }
  HCRVeyronBrushed.clearEncoderPulses(id, left_wheel_id_);
  HCRVeyronBrushed.clearEncoderPulses(id, right_wheel_id_);
  
  if (turtlebot3 == "Burger")
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
  else if (turtlebot3 == "Waffle or Waffle Pi")
    dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
  else
    dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;

  DEBUG_SERIAL.println("Success to init Motor Driver");
  return true;
}

void Turtlebot3MotorDriver::close(void)
{
  // Close port
  Serial1.end();
  DEBUG_SERIAL.end();
}

bool Turtlebot3MotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  int id = 1;
  left_value = atoi(HCRVeyronBrushed.readEncoderPulses(id, left_wheel_id_));
  right_value = atoi(HCRVeyronBrushed.readEncoderPulses(id, right_wheel_id_));
  return true;
}

bool Turtlebot3MotorDriver::writeVelocity(int32_t left_value, int32_t right_value)
{
  int id = 1;
  
  //set conditional speed
  HCRVeyronBrushed.setConditionalSpeed(id, 1, left_value);
  HCRVeyronBrushed.setConditionalSpeed(id, 2, right_value);

  //use conditional speed to run the motor
  boolean isStartConditionalSpeedM1 = true;
  boolean isStartConditionalSpeedM2 = true;
  HCRVeyronBrushed.startConditionalSpeed(id, isStartConditionalSpeedM1,  isStartConditionalSpeedM2);

  return true;
}

bool Turtlebot3MotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity_cmd[2];

  float lin_vel = value[LEFT];
  float ang_vel = value[RIGHT];

  wheel_velocity_cmd[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity_cmd[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
  wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

  dxl_comm_result = writeVelocity((int32_t)wheel_velocity_cmd[LEFT], (int32_t)wheel_velocity_cmd[RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}
