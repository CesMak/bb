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

#include <bb_control/bb_control_node.hpp>

namespace ballbot
{
  //helper functions:
  double ControlNode::convertDistanceToAngle(double dis)
  {
     // std::cout<<dis<<" m"<<std::endl;
      double a =  (double) 360/(2*3.14159265359*0.06) *dis;

      //std::cout<<a<<" grad"<<std::endl;
      double b = (double) a/180 * 3.14159265359;
      //stdcout<<b<<" rad"<<std::endl;


      int c = a/360;
      double d = a-(c*360);

      //std::cout<<d<<" grad angepasst"<<std::endl;

      double e = (double) d/180 * 3.14159265359;
      std::cout<<e<<" rad angepasst"<<std::endl;

      // return valueebetwenn 0 and 2 pi!
      return e;
  }

  std::vector<double> ControlNode::convertToAngleVel(std::vector<double> input_vec)
  {
    std::vector<double> output_vec(input_vec.size());
    for(int i=0; i<input_vec.size(); i++)
    {
      output_vec.at(i)=linearVelToAngleVel(input_vec.at(i));
    }
    return output_vec;
  }

  double ControlNode::linearVelToAngleVel(double linear_vel)
  {
    //w=v*r
    double res = (double) linear_vel/(0.06);
    std::cout<<"linear_vel "<<linear_vel<<std::endl;
    std::cout<<"w= "<<res<<std::endl;
    return res;
  }

  std::vector<double> ControlNode::convertVectorToAngle(std::vector<double> input_vec)
  {
    std::vector<double> output_vec(input_vec.size());
    for(int i=0;i<input_vec.size();i++)
    {
      output_vec.at(i)=convertDistanceToAngle(input_vec.at(i));
    }
    return output_vec;
  }

  //2D Control calculation:
  std::vector<double> ControlNode::calc2DMotorCommands()
  {
    //0. use previous_joint_state_msg_ to calculate ball orientation!

    //std::cout<<previous_joint_state_msg_.effort.at(0)<<std::endl;
    std::cout<<"vel: "<<previous_joint_state_msg_.velocity.at(0)<<std::endl;

    //1. virtual motor torques:
    //states matrix:
    Eigen::RowVectorXd x(4);
    Eigen::RowVectorXd y(4);
    Eigen::RowVectorXd z(4);
//    x << imu_phi_[0], imu_dphi_[0], joints_position_[0], joints_velocity_[0];
//    y << imu_phi_[1], imu_dphi_[1], joints_position_[1], joints_velocity_[1];
//    z << imu_phi_[2], imu_dphi_[2], joints_position_[2], joints_velocity_[2];

//    std::cout<<" x-state: "<<x<<std::endl;
//    std::cout<<" y-state: "<<y<<std::endl;
//    std::cout<<" z-state: "<<z<<std::endl;


//    double* ptr1 = &gains_2D_Kxz[0];
//    Eigen::Map<Eigen::RowVectorXd> Kxz(ptr1, 4);

//    double* ptr2 = &gains_2D_Kyz[0];
//    Eigen::Map<Eigen::RowVectorXd> Kyz(ptr2, 4);

//    double* ptr3 = &gains_2D_Kxy[0];
//    Eigen::Map<Eigen::RowVectorXd> Kxy(ptr3, 4);

//    double Tx_=-Kxz.dot(x);
//    double Ty_=-Kyz.dot(y);
//    double Tz_=-Kxy.dot(z);
//    ROS_INFO("[bb_control2D] virtual u=-Kx: [%f, %f, %f]",Tx_,Ty_,Tz_);

//    //2. real motor torques:
//    // Formulas see p. 70
//    double T1_=0.33333333*(Tz_+2.0/(cos(alpha))*(Tx_*cos(beta1)-Ty_*sin(beta1)) );
//    double T2_=0.33333333*(Tz_+1.0/(cos(alpha)) *(sin(beta2)*(-sqrt(3.0)*Tx_+Ty_) - cos(beta2)*(Tx_+sqrt(3.0)*Ty_)) );
//    double T3_=0.33333333*(Tz_+1.0/(cos(alpha)) *(sin(beta3)*(sqrt(3.0)*Tx_+Ty_) + cos(beta3)*(-Tx_+sqrt(3.0)*Ty_)) );
//    ROS_INFO("[bb_control2D] real u: [%f, %f, %f]",T1_,T2_,T3_);

    std::vector<double> T_ {0.0, 0.0, 0.0};

    return T_;
  }

  //constructor is called only once:
  ControlNode::ControlNode(ros::NodeHandle& nh)
  {
    //get Params:
    controller_type_ = nh.param("control_constants/controller_type",std::string("2D"));
    motors_controller_type_ = nh.param("control_constants/motors_controller_type",std::string("VelocityJointInterface"));

    // get 2D control gains:
    gains_2D_Kxz_=nh.param("control_constants/gains_2D_Kxz",(std::vector<double> ) {1.0,2.0,3.0,4.0});
    gains_2D_Kyz_=nh.param("control_constants/gains_2D_Kyz", (std::vector<double> ) {1.0,2.0,3.0,4.0});
    gains_2D_Kxy_=nh.param("control_constants/gains_2D_Kxy", (std::vector<double> ) {1.0,2.0,3.0,4.0});

    alpha_=nh.param("control_constants/alpha",45.0);
    beta_=nh.param("control_constants/beta1",120.0);

    update_rate_ = nh.param("ballbot/control_constants/update_rate", 10.0);

    //Subscriber:
    imu_sub_ = nh.subscribe("/ballbot/sensor/imu", 5, &ControlNode::imuCallback,this);
    joints_sub_ = nh.subscribe("/ballbot/joints/joint_states", 5, &ControlNode::jointsCallback,this);

    //Publisher:
    if(motors_controller_type_=="PositionJointInterface")
      motors_controller_type_="position";
    if(motors_controller_type_=="VelocityJointInterface")
      motors_controller_type_="velocity";
    if(motors_controller_type_=="EffortJointInterface")
      motors_controller_type_="effort";
    joint_commands_1_pub_ = nh.advertise<std_msgs::Float64>("/ballbot/joints/wheel1_"+motors_controller_type_+"_controller/command", 5);
    joint_commands_2_pub_ = nh.advertise<std_msgs::Float64>("/ballbot/joints/wheel2_"+motors_controller_type_+"_controller/command", 5);
    joint_commands_3_pub_ = nh.advertise<std_msgs::Float64>("/ballbot/joints/wheel3_"+motors_controller_type_+"_controller/command", 5);

    //action server:

    // init vectors:
    imu_phi_.push_back(0.0);
    imu_phi_.push_back(0.0);
    imu_phi_.push_back(0.0);
    imu_dphi_.push_back(0.0);
    imu_dphi_.push_back(0.0);
    imu_dphi_.push_back(0.0);
    realT_.push_back(0.0);
    realT_.push_back(0.0);
    realT_.push_back(0.0);
  }

  // ROS API Callbacks:
  void ControlNode::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
  {
    previous_imu_msg_ = *imu_msg;

    imu_phi_.at(0)=previous_imu_msg_.orientation.x;
    imu_phi_.at(1)=previous_imu_msg_.orientation.y;
    imu_phi_.at(2)=previous_imu_msg_.orientation.z;
    imu_dphi_.at(0)=previous_imu_msg_.angular_velocity.x;
    imu_dphi_.at(1)=previous_imu_msg_.angular_velocity.y;
    imu_dphi_.at(2)=previous_imu_msg_.angular_velocity.z;

    imu_updated_=true;

    if(controller_type_=="2D")
    {
      if(imu_updated_ && joint_state_updated_)
      {
        realT_=calc2DMotorCommands();
        imu_updated_=false;
      }
    }
    else if(controller_type_=="drive")
    {
      realT_={0.1,0.0,0.0};
    }
  }

  void ControlNode::jointsCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
  {
  previous_joint_state_msg_=*joint_state_msg;
  joint_state_updated_=true;

  if(imu_updated_ && joint_state_updated_)
     {
      realT_=calc2DMotorCommands();
      joint_state_updated_=false;
     }
  }

  //update (publish messages...)
  void ControlNode::update()
  {
    // note that the wheel torque is limited to ca. 4.1Nm.
    std_msgs::Float64 realT1;
    std_msgs::Float64 realT2;
    std_msgs::Float64 realT3;

    realT1.data=1.0;//realT_.at(0);
    realT2.data=realT_.at(1);
    realT3.data=realT_.at(2);
    joint_commands_1_pub_.publish(realT1);
    joint_commands_2_pub_.publish(realT2);
    joint_commands_3_pub_.publish(realT3);
  }

}  // end namespace ballbot

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ballbot_control");
  ROS_INFO("start ballbot_control_node:");

  ros::NodeHandle nh;

  // this wait is needed to ensure this ros node has gotten
  // simulation published /clock message, containing
  // simulation time.
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  ballbot::ControlNode node(nh);
  ros::Rate loop_rate(nh.param("ballbot/control_constants/update_rate", 10.0));


  // TODO wait here to start publishing nodes information
  // until first non zero value from gazebo is received!
  while (ros::ok())
  {
    ros::spinOnce();
    node.update();
    loop_rate.sleep();
  }

  return 0;
}
