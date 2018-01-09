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

ros::Publisher joint_commands_1_pub;
ros::Publisher joint_commands_2_pub;
ros::Publisher joint_commands_3_pub;

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

void calc_motor_commands()
{

  std_msgs::Float64 real_u_1;
  std_msgs::Float64 real_u_2;
  std_msgs::Float64 real_u_3;

  // Beobachtung wenn mu2 > 0 dann fährt der roboter zumindest ein stückchen auch in 0 bzw. 180 grad
  // dafür fährt er dann aber nicht mehr so gut in 0, 90°.

  // für 180°:
  real_u_1.data=-1;//*cos(deg_to_rad(150-0));
  real_u_2.data=0;//*cos(deg_to_rad(270-0));
  real_u_3.data=1;//*cos(deg_to_rad(30-0));

  //ROS_INFO("[bb_control] send motor command: [%f, %f, %f]",real_u_1,real_u_2,real_u_3);
  joint_commands_1_pub.publish(real_u_1);
  joint_commands_2_pub.publish(real_u_2);
  joint_commands_3_pub.publish(real_u_3);
}

void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //calc_motor_commands();
}


int main(int argc, char** argv)
{
   ROS_INFO("[omniwheel_control] starting the motor controller");
   ros::init(argc, argv, "omniwheel_motor_controller");
   ros::NodeHandle nh;

  // ros::Subscriber joints_sub = nh.subscribe("/ballbot/joints/joint_states", 5, jointsCallback);

   //Publisher:
   joint_commands_1_pub = nh.advertise<std_msgs::Float64>("/ballbot/joints/wheel1_position_controller/command", 10);
   joint_commands_2_pub = nh.advertise<std_msgs::Float64>("/ballbot/joints/wheel2_position_controller/command", 10);
   joint_commands_3_pub = nh.advertise<std_msgs::Float64>("/ballbot/joints/wheel3_position_controller/command", 10);

   // joint_cmd_pubs_[joint_name_vector_[i]] = nh_.advertise<std_msgs::Float64>("joints/" + joint_name_vector_[i] + "_position/command", 5);

   ros::Rate loop_rate(1);

   int count = 0;
   while (ros::ok())
   {
     calc_motor_commands();
     ros::spinOnce();
     loop_rate.sleep();
     ++count;
   }
    return 0;
}




