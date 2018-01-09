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

//#include <bb_control/bb_control_node.hpp>
#include <bb_control/bb_control2D.hpp>

namespace ballbot
{
  //helper functions:
  std::vector<double> Control2D::calc2DMotorCommands(std::vector<double> imu_phi, std::vector<double> imu_dphi,
                                      std::vector<double> joints_position,std::vector<double> joints_velocity,
                                      std::vector<double> gains_2D_Kxz, std::vector<double> gains_2D_Kyz,
                                      std::vector<double> gains_2D_Kxy, double alpha, double beta1,
                                      double beta2, double beta3)
  {
    std::clock_t begin = std::clock();

    //1. virtual motor torques:
    //states matrix:
    Eigen::RowVectorXd x(4);
    Eigen::RowVectorXd y(4);
    Eigen::RowVectorXd z(4);
    x << imu_phi[0], imu_dphi[0],joints_position[0],joints_velocity[0];
    y << imu_phi[1],imu_dphi[1],joints_position[1],joints_velocity[1];
    z << imu_phi[2],imu_dphi[2],joints_position[2],joints_velocity[2];

    //std::cout<<" x-state: "<<x<<std::endl;
    //std::cout<<" y-state: "<<y<<std::endl;
    //std::cout<<" z-state: "<<z<<std::endl;


    double* ptr1 = &gains_2D_Kxz[0];
    Eigen::Map<Eigen::RowVectorXd> Kxz(ptr1, 4);

    double* ptr2 = &gains_2D_Kyz[0];
    Eigen::Map<Eigen::RowVectorXd> Kyz(ptr2, 4);

    double* ptr3 = &gains_2D_Kxy[0];
    Eigen::Map<Eigen::RowVectorXd> Kxy(ptr3, 4);

    double Tx_=-Kxz.dot(x);
    double Ty_=-Kyz.dot(y);
    double Tz_=-Kxy.dot(z);
    //ROS_INFO("[bb_control2D] virtual u=-Kx: [%f, %f, %f]",Tx_,Ty_,Tz_);

    //2. real motor torques:
    // Formulas see p. 70
    double T1_=0.33333333*(Tz_+2.0/(cos(alpha))*(Tx_*cos(beta1)-Ty_*sin(beta1)) );
    double T2_=0.33333333*(Tz_+1.0/(cos(alpha)) *(sin(beta2)*(-sqrt(3.0)*Tx_+Ty_) - cos(beta2)*(Tx_+sqrt(3.0)*Ty_)) );
    double T3_=0.33333333*(Tz_+1.0/(cos(alpha)) *(sin(beta3)*(sqrt(3.0)*Tx_+Ty_) + cos(beta3)*(-Tx_+sqrt(3.0)*Ty_)) );
    //ROS_INFO("[bb_control2D] real u: [%f, %f, %f]",T1_,T2_,T3_);

    std::vector<double> T_ {T1_,T2_,T3_};

    clock_t end = std::clock();
    double elapsed_secs = double(end - begin) / (CLOCKS_PER_SEC);
    //std::cout<<elapsed_secs<<std::endl;
    return T_;
  }

  //constructor:
  Control2D::Control2D()
  {

  }
}  // end namespace ballbot
