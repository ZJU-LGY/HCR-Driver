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

#ifndef TURTLEBOT3_MOTOR_DRIVER_H_
#define TURTLEBOT3_MOTOR_DRIVER_H_

#include <DFRobotVeyronBrushed.h>
#include <HardwareSerial.h>

// Limit values (XM430-W210-T and XM430-W350-T)
//#define BURGER_DXL_LIMIT_MAX_VELOCITY            265     // MAX RPM is 61 when XL is powered 12.0V
//#define WAFFLE_DXL_LIMIT_MAX_VELOCITY            330     // MAX RPM is 77 when XM is powered 12.0V

#define LEFT                            0
#define RIGHT                           1

#define VELOCITY_CONSTANT_VALUE         41.69988758  // V = r * w = r     *        (RPM             * 0.10472)
                                                     //           = r     * (0.229 * Goal_Velocity) * 0.10472
                                                     //
                                                     // Goal_Velocity = V / r * 41.69988757710309

#define DEBUG_SERIAL  Serial



class HCRMotorDriver
{
 public:
  HCRMotorDriver();
  ~HCRMotorDriver();
  bool init(void);
  void close(void);
  bool readEncoder(int32_t &left_value, int32_t &right_value);
  bool writeVelocity(int32_t left_value, int32_t right_value);
  bool controlMotor(const float wheel_radius, const float wheel_separation, float* value);

 private:
  uint16_t limit_max_velocity;
};

#endif // TURTLEBOT3_MOTOR_DRIVER_H_
