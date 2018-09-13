/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Markus Lamprecht,
 *                      Technische Universität Darmstadt
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Technische Universität Darmstadt nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef BB_CONTROL_NODE_H__
#define BB_CONTROL_NODE_H__

#include <ros/ros.h>
#include <ros/console.h>
//#include <bb_control/bb_control2D.hpp>

// ROS messages:
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

#include <eigen3/Eigen/Dense>  /* for matrix multiplication */
#include <math.h>       /* sin, sqrt */

//#include <std_msgs/String.h>
//#include <std_msgs/Int32.h>

namespace ballbot
{
class ControlNode
{
public:
  ControlNode(ros::NodeHandle& nh);

  void update();

  double update_rate_;
  double update_time_;

protected:
  // helper
  double convertDistanceToAngle(double dis);
  std::vector<double> convertVectorToAngle(std::vector<double> input_vec);

  void calc2DMotorCommands();
  void calc2MotorCommands_withoutOdometry();
  void calc2MotorCommands_withOdometry();
  std::vector<double> toEulerAngle(double x, double y, double z, double w);

  std::vector<double> convertToAngleVel(std::vector<double> input_vec);
  double linearVelToAngleVel(double linear_vel);

  // UI callbacks

  // ROS API callbacks
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void jointsCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg);


  // class members
  //Control2D control2D_;

  // helper variables:
  sensor_msgs::Imu previous_imu_msg_;
  bool imu_updated_=true;
  sensor_msgs::JointState previous_joint_state_msg_;
  bool joint_state_updated_;
  std::vector<double> imu_phi_, imu_dphi_;
  std::vector<double> joints_position_;
  std::vector<double> joints_velocity_;
  std::vector<double> realT_;
  geometry_msgs::Vector3 desired_torques_;

  // parameters from the parameter server
  std::vector<double> gains_2D_Kxz_;
  std::vector<double> gains_2D_Kyz_;
  double Kr_;
  double Tv_;

  std::string controller_type_;
  std::string motors_controller_type_;

  double alpha_;
  double beta_;
  double sa_;
  double sb_;
  double ca_;
  double cb_;

  int balancing_time_;


   // subscriber
  ros::Subscriber imu_sub_;
  ros::Subscriber joints_sub_;

  //publisher:
  ros::Publisher joint_commands_1_pub_;
  ros::Publisher joint_commands_2_pub_;
  ros::Publisher joint_commands_3_pub_;
  ros::Publisher rpy_pub_;
  ros::Publisher calc_ball_odom_pub_;
  ros::Publisher desired_torques_pub_;    // calculated torques in NM!

  // action server


  //for control:
  std::vector<double> psi_ball_actual_;
  std::vector<double> psi_ball_last_;

  std::vector<double> phi_ball_actual_;
  std::vector<double> phi_ball_last_;
  std::vector<double> imu_phi_last_;
};

} // end namespace
#endif
