/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 14.02.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_core/types.h>
#include <ow_core/test_utilities/gtest.h>  
#include <ow_zmp/zmp_estimator.h>

/*!
 * \brief The Vector3TestSuite class.
 *
 * The TestFixture for Vector3.
 *
 */
class ZmpEstimatorTestSuite : 
  public ::testing::Test
{
protected:
  double yR_;
  double yL_;

  double fx_;
  double fy_;
  double fz_;

  double d_;
  double dt_;
  double coff_;

  ow::LinearState zmp_state_expected_;
  ow::HomogeneousTransformation T_L_W_;
  ow::HomogeneousTransformation T_R_W_;
  ow::Wrench WL_;
  ow::Wrench WR_;

  ZmpEstimatorTestSuite() :
    yR_(0.1),
    yL_(-0.1),
    fx_(0.0),
    fy_(-10.0),
    fz_(200.0),
    d_(0.1),
    dt_(0.005),
    coff_(10.0),
    zmp_state_expected_(ow::LinearState::Zero())
  {
    T_L_W_.translation() << 0.0, yL_, 0.0;
    T_R_W_.translation() << 0.0, yR_, 0.0;
    WL_ << fx_, fy_, fz_, 0, 0, 0;
    WR_ << fx_, fy_, fz_, 0, 0, 0;
  }
};

TEST_F(ZmpEstimatorTestSuite, updateZMPEstimator)
{
  ow::LinearState zmp_state = ow::LinearState::Zero();

  ow_zmp::ZmpEstimator zmp(coff_,dt_,d_);
  zmp_state = zmp.updateWrtWorld(WL_, T_L_W_, WR_, T_R_W_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(zmp_state_expected_.p(), "zmp_state.p()", 
    zmp_state_expected_.p(), "zmp_state_expected_.p()"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}