/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */

#include "hcrobot_core_config.h"

/*******************************************************************************
  Setup function
*******************************************************************************/
void setup()
{
  DEBUG_SERIAL.begin(57600);
  
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
//  nh.subscribe(sound_sub);
//  nh.subscribe(motor_power_sub);
//  nh.subscribe(reset_sub);

  nh.advertise(sensor_state_pub);
//  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
//  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
//  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);

  Wire.begin();

  sixDOF.init(); //begin the IMU
  
  tf_broadcaster.init(nh);

  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for IMU
//  sensors.init();

  // Init diagnosis
//  diagnosis.init();

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
//  controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  prev_update_time = millis();

//  pinMode(LED_WORKING_CHECK, OUTPUT);

  setup_end = true;
}

/*******************************************************************************
  Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();
//  updateVariable(nh.connected());
//  updateTFPrefix(nh.connected());

  if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
//    DEBUG_SERIAL.print("Update Velocity: ");
//    Serial.print(goal_velocity[0]);
//    Serial.print(" | ");  
//    Serial.println(goal_velocity[1]);
    motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
    tTime[0] = t;
  }

//  if ((t - tTime[1]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
//  {
//    publishCmdVelFromRC100Msg();
//    tTime[1] = t;
//  }

  if ((t - tTime[1]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishSensorStateMsg();
//    publishBatteryStateMsg();
    publishDriveInformation();
    tTime[1] = t;
  }

  if ((t - tTime[2]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    publishMagMsg();
//    DEBUG_SERIAL.print("Quatenions:");
//    serialPrintFloatArr(q, 4);
//    DEBUG_SERIAL.println();
    tTime[2] = t;
  }

//  if ((t - tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
//  {
//    publishVersionInfoMsg();
//    tTime[4] = t;
//  }

//#ifdef DEBUG
//  if ((t - tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))
//  {
//    sendDebuglog();
//    tTime[5] = t;
//  }
//#endif

  // Send log message after ROS connection
//  sendLogMsg();

  // Receive data from RC100
//  bool clicked_state = controllers.getRCdata(goal_velocity_from_rc100);
//  if (clicked_state == true)
//    tTime[6] = millis();

  // Check push button pressed for simple test drive
//  driveTest(diagnosis.getButtonPress(3000));

  // Update the IMU unit
//  sensors.updateIMU();
  sixDOF.getQ(q);

  // TODO
  // Update sonar data
  // sensors.updateSonar(t);

  // Start Gyro Calibration after ROS connection
//  updateGyroCali(nh.connected());

  // Show LED status
//  diagnosis.showLedStatus(nh.connected());

  // Update Voltage
//  battery_state = diagnosis.updateVoltageCheck(setup_end);

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
//  waitForSerialLink(nh.connected());
}

/*******************************************************************************
  Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;
//  DEBUG_SERIAL.print("Callback CMD Velocity: ");
//  Serial.print(cmd_vel_msg.linear.x);
//  Serial.print(" | ");  
//  Serial.println(cmd_vel_msg.linear.x);

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
//  tTime[6] = millis();
}

/*******************************************************************************
  Publish msgs (CMD Velocity data from RC100 : angular velocity, linear velocity)
*******************************************************************************/
//void publishCmdVelFromRC100Msg(void)
//{
//  cmd_vel_rc100_msg.linear.x  = goal_velocity_from_rc100[LINEAR];
//  cmd_vel_rc100_msg.angular.z = goal_velocity_from_rc100[ANGULAR];
//
//  cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);
//}

/*******************************************************************************
  Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  sixDOF.getValues(val);
  imu_msg.angular_velocity.x = val[3];
  imu_msg.angular_velocity.y = val[4];
  imu_msg.angular_velocity.z = val[5];
  
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = val[0];
  imu_msg.linear_acceleration.y = val[1];
  imu_msg.linear_acceleration.z = val[2];

  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = q[0];
  imu_msg.orientation.x = q[1];
  imu_msg.orientation.y = q[2];
  imu_msg.orientation.z = q[3];

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
  Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  sixDOF.getValues(val);
  mag_msg.magnetic_field.x = val[6];
  mag_msg.magnetic_field.y = val[7];
  mag_msg.magnetic_field.z = val[8];

  mag_msg.magnetic_field_covariance[0] = 0.0048;
  mag_msg.magnetic_field_covariance[1] = 0;
  mag_msg.magnetic_field_covariance[2] = 0;
  mag_msg.magnetic_field_covariance[3] = 0;
  mag_msg.magnetic_field_covariance[4] = 0.0048;
  mag_msg.magnetic_field_covariance[5] = 0;
  mag_msg.magnetic_field_covariance[6] = 0;
  mag_msg.magnetic_field_covariance[7] = 0;
  mag_msg.magnetic_field_covariance[8] = 0.0048;

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}

/*******************************************************************************
  Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  sensor_state_msg.header.stamp = rosNow();
//  sensor_state_msg.battery = sensors.checkVoltage();

  dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true)
    updateMotorInfo(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
  else
    return;

//  sensor_state_msg.bumper = sensors.checkPushBumper();
//
//  sensor_state_msg.cliff = sensors.getIRsensorData();
//
//  // TODO
//  // sensor_state_msg.sonar = sensors.getSonarData();
//
//  sensor_state_msg.illumination = sensors.getIlluminationData();
//
//  sensor_state_msg.button = sensors.checkPushButton();

//  sensor_state_msg.torque = motor_driver.getTorque();

  sensor_state_pub.publish(&sensor_state_msg);
}

/*******************************************************************************
  Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
  Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
  Update the joint states
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
  CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
  Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};

  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
  Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  orientation = q;
  theta       = atan2f(orientation[1] * orientation[2] + orientation[0] * orientation[3],
                       0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}

/*******************************************************************************
  Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
  ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
  Time Interpolation function (deprecated)
*******************************************************************************/
//ros::Time addMicros(ros::Time & t, uint32_t _micros)
//{
//  uint32_t sec, nsec;
//
//  sec  = _micros / 1000 + t.sec;
//  nsec = _micros % 1000000000 + t.nsec;
//
//  return ros::Time(sec, nsec);
//}

/*******************************************************************************
  Start Gyro Calibration
*******************************************************************************/
//void updateGyroCali(bool isConnected)
//{
//  static bool isEnded = false;
//  char log_msg[50];
//
//  (void)(isConnected);
//
//  if (nh.connected())
//  {
//    if (isEnded == false)
//    {
//      sprintf(log_msg, "Start Calibration of Gyro");
//      nh.loginfo(log_msg);
//
//      sensors.calibrationGyro();
//
//      sprintf(log_msg, "Calibration End");
//      nh.loginfo(log_msg);
//
//      isEnded = true;
//    }
//  }
//  else
//  {
//    isEnded = false;
//  }
//}

/*******************************************************************************
  Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
  Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}

/*******************************************************************************
  Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}
