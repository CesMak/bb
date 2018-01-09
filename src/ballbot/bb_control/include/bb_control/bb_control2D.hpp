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

#ifndef BB_CONTROL2D_H__
#define BB_CONTROL2D_H__

#include <math.h>       /* sin */
#include <Eigen/Dense>  /* for matrix multiplication */
#include <ctime>


namespace ballbot
{
class Control2D
{
public:
  Control2D();

  std::vector<double> calc2DMotorCommands(std::vector<double> imu_phi,
                           std::vector<double> imu_dphi,
                           std::vector<double> joints_position,
                           std::vector<double> joints_velocity,
                           std::vector<double> gains_2D_Kxz,
                           std::vector<double> gains_2D_Kyz,
                           std::vector<double> gains_2D_Kxy,
                           double alpha,
                           double beta1,
                           double beta2,
                           double beta3);

protected:
  // parameters:
  // 2D Gain Matrix in here:
  std::vector<std::vector<double>> K_;

  // state Matrix:
  std::vector<std::vector<double>> states_;

  //virtual motor torques:
  std::vector<std::vector<double>> T_;

 };

} // end namespace
#endif
