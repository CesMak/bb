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

#include <bb_control/bb_control.hpp>

#include <controller_manager/controller_manager.h>

ros::Publisher joint_commands_1_pub;
ros::Publisher joint_commands_2_pub;
ros::Publisher joint_commands_3_pub;

sensor_msgs::Imu previous_imu_msg;
sensor_msgs::JointState previous_joints_msg;

bool imu_updated=false;
bool joints_updated=false;

double deg_to_rad(double x)
{
  return x*M_PI/180;
}


double scalar_product(std::vector<double> a, std::vector<double> b)
{
    if( a.size() != b.size() ) // error check
    {
        puts( "Error a's size not equal to b's size" ) ;
        return -1 ;  // not defined
    }

    // compute
    double product = 0;
    for (int i = 0; i <= a.size()-1; i++)
       product += (a[i])*(b[i]); // += means add to product
    return product;
}

// u = Fw - Kx = [Tx,Ty,Tz]
// if ball should balance on the spot w=0?
std::vector<double> get_virtual_motor_torques(std::vector<std::vector<double>> K,std::vector<std::vector<double>> states)
{
  std::vector<double>K1 = K.at(0);
  std::vector<double>K3 = K.at(1);
  std::vector<double>K2 = K.at(2);

  std::vector<double>x = states.at(0);
  std::vector<double>y = states.at(1);
  std::vector<double>z = states.at(2);

  double Tx= scalar_product(K1,x);
  double Ty= scalar_product(K2,y);
  double Tz=scalar_product(K3,z);
  std::vector<double> virtual_mortor_torques(3);
  virtual_mortor_torques.at(0)=Tx;
  virtual_mortor_torques.at(1)=Ty;
  virtual_mortor_torques.at(2)=Tz;
  ROS_INFO("[bb_control] virtual u: [%f, %f, %f]",Tx,Ty,Tz);
  return virtual_mortor_torques;
}

// alpha=  [rad]  normally should be 45deg
// beta=   [rad]  normally should be 0,120,240
// Tx,Ty,Tz virtual motor torques
std::vector<double> get_real_motor_torques(double alpha,std::vector<double> beta,std::vector<double> virtual_u)
{
    double Tx=virtual_u.at(0);
    double Ty=virtual_u.at(1);
    double Tz=virtual_u.at(2);

    double beta1=beta.at(0);
    double beta2=beta.at(1);
    double beta3=beta.at(2);

    std::vector<double> real_motor_torques(3);
    // Formulas see p. 70
    real_motor_torques.at(0)=0.33333333*(Tz+2.0/(cos(alpha))*(Tx*cos(beta1)-Ty*sin(beta1)) );
    real_motor_torques.at(1)=0.33333333*(Tz+1.0/(cos(alpha)) *(sin(beta2)*(-sqrt(3.0)*Tx+Ty) - cos(beta2)*(Tx+sqrt(3.0)*Ty)) );
    real_motor_torques.at(2)=0.33333333*(Tz+1.0/(cos(alpha)) *(sin(beta3)*(sqrt(3.0)*Tx+Ty) + cos(beta3)*(-Tx+sqrt(3.0)*Ty)) );
    return real_motor_torques;
}


void calc_motor_commands()
{
  imu_updated,joints_updated=false;
  std::vector<double> position = previous_joints_msg.position;
  std::vector<double> velocity = previous_joints_msg.velocity;

  //IMU Values:
  double thetax=previous_imu_msg.orientation.x;
  double thetay=previous_imu_msg.orientation.y;
  double thetaz=previous_imu_msg.orientation.z;

  double dthetax=previous_imu_msg.angular_velocity.x; // angluar velocity
  double dthetay=previous_imu_msg.angular_velocity.y;
  double dthetaz=previous_imu_msg.angular_velocity.z;

  // Motor values:
  double phix=position.at(0);
  double phiy=position.at(1);
  double phiz=position.at(2);

  double dphix=velocity.at(0);
  double dphiy=velocity.at(1);
  double dphiz=velocity.at(2);

  // inputs:
  double alpha=deg_to_rad(45);
  std::vector<double> beta={deg_to_rad(0),deg_to_rad(120),deg_to_rad(240)};

  std::vector<std::vector<double>> K;
  std::vector<double>K1= { -0.3162  , 24.8947,   -0.5533,    7.1600 }; //  Kyz
  std::vector<double>K2= {  -0.3162 ,  24.8947,   -0.5533,    7.1600 }; // Kxz
  std::vector<double>K3= {0,2.68,0,0};
  K.push_back(K1);
  K.push_back(K2);
  K.push_back(K3);

  //state x=[phix,thetax,dphix,dthetax]
  std::vector<std::vector<double>> states;
  std::vector<double>x= {phix,thetax,dphix,dthetax};
  std::vector<double>y= {phiy,thetay,dphiy,dthetay};
  std::vector<double>z= {phiz,thetaz,dphiz,dthetaz};
  states.push_back(x);
  states.push_back(y);
  states.push_back(z);

  std::vector<double> virtual_u =get_virtual_motor_torques(K,states);
  std::vector<double> real_u =get_real_motor_torques(alpha,beta,virtual_u);

  // print input values:
  ROS_INFO("[bb_control] Orientation x,y,z: [%f, %f, %f]", previous_imu_msg.orientation.x,previous_imu_msg.orientation.y,previous_imu_msg.orientation.z);
  ROS_INFO("[bb_control] Joints/Position 1,2,3: [%f, %f, %f]",position.at(0),position.at(1),position.at(2));
  ROS_INFO("[bb_control] Joints/Velocity 1,2,3: [%f, %f, %f]",velocity.at(0),velocity.at(1),velocity.at(2));

  std_msgs::Float64 real_u_1;
  std_msgs::Float64 real_u_2;
  std_msgs::Float64 real_u_3;

  real_u_1.data=real_u.at(0);
  real_u_2.data=real_u.at(1);
  real_u_3.data=real_u.at(2);

  ROS_INFO("[bb_control] send motor command: [%f, %f, %f]",real_u_1,real_u_2,real_u_3);
  // note that the wheel torque is limited to ca. 4.1Nm.
  joint_commands_1_pub.publish(real_u_1);
  joint_commands_2_pub.publish(real_u_2);
  joint_commands_3_pub.publish(real_u_3);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  previous_imu_msg = *msg;
  imu_updated=true;

  if(imu_updated && joints_updated)
    calc_motor_commands();
}

void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  previous_joints_msg=*msg;
  joints_updated=true;

  if(imu_updated && joints_updated)
    calc_motor_commands();
}

int main(int argc, char** argv)
{
   ROS_INFO("[bb_control] starting the motor controller");
   ros::init(argc, argv, "ballbot_motor_controller");
   ros::NodeHandle nh;

   //Subscriber:
   ros::Subscriber imu_sub = nh.subscribe("/ballbot/sensor/imu", 5, imuCallback);
   ros::Subscriber joints_sub = nh.subscribe("/ballbot/joints/joint_states", 5, jointsCallback);

   //Publisher:
   joint_commands_1_pub = nh.advertise<std_msgs::Float64>("/ballbot/joints/wheel1_position_controller/command", 5);
   joint_commands_2_pub = nh.advertise<std_msgs::Float64>("/ballbot/joints/wheel2_position_controller/command", 5);
   joint_commands_3_pub = nh.advertise<std_msgs::Float64>("/ballbot/joints/wheel3_position_controller/command", 5);

          // joint_cmd_pubs_[joint_name_vector_[i]] = nh_.advertise<std_msgs::Float64>("joints/" + joint_name_vector_[i] + "_position/command", 5);

    ros::spin();
    return 0;
}




