/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
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

#include <unistd.h>
#include <stdio.h>
#include <iostream>


#include <ow_core/types.h>
#include <ow_zmp/foot_zmp_estimator.h>
#include <ow_zmp/zmp_estimator.h>

int main(int argc, char* argv[])
{
  std::cout << "argc: " << argc;

  for (int var = 0; var < argc; ++var)
  {
    std::cout << "argv[" << var << "]: "<< argv[var];
  }

  /*ow::LinearState P;
  ow::HomogeneousTransformation T_L_W;
  ow::HomogeneousTransformation T_R_W;
  ow::Wrench WL;
  ow::Wrench WR;
  ow::Scalar d = 0.1;
  ow::Scalar dt = 0.005;
  ow::Scalar f_cutoff = 10.0;
  ow::Scalar f_thresh = 20.0;

  WL.force() << 0.0, -10.0, 200.0;
  WR.force() << 0.0, -10.0, 200.0;
  T_L_W.translation() << 0.0,  0.1, 0.0;
  T_R_W.translation() << 0.0, -0.1, 0.0;

  ow_zmp::ZmpEstimator zmp(f_cutoff, dt, d, f_thresh);

  P.setZero();
  P = zmp.update(WL, T_L_W, WR, T_R_W);

  std::cout << "P" << P.toString() << std::endl;*/

  usleep(1000*1000);
  return 0;
}
