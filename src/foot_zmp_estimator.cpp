/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
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

#include <ow_zmp/foot_zmp_estimator.h>

namespace ow_zmp
{

  FootZmpEstimator::FootZmpEstimator() : p_f_(ow::LinearState::Zero()),
                                         min_force_thres_(0.0),
                                         d_(0.0),
                                         is_support_(false)
  {
  }

  bool FootZmpEstimator::init(
      ow::Scalar freq,
      ow::Scalar d,
      ow::Scalar min_force_thres)
  {
    d_ = d;
    min_force_thres_ = min_force_thres;

    diff_.reset(new Differentiator(
        ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(freq)));

    return true;
  }

  const ow::LinearState &FootZmpEstimator::updateWrtFoot(
      const ow::Wrench &W_f)
  {
    // Compute ZMP.
    if (W_f.force().z() > min_force_thres_)
    {
      is_support_ = true;
      p_f_.x().x() = (-W_f.moment().y() - W_f.force().x() * d_) / W_f.force().z();
      p_f_.x().y() = (W_f.moment().x() - W_f.force().y() * d_) / W_f.force().z();
      p_f_.x().z() = 0.0;
    }
    else
    {
      is_support_ = false;
      p_f_.x().setZero();
    }

    // Update derivatives.
    p_f_ = diff_->update(p_f_);
    return p_f_;
  }

  const ow::LinearState &FootZmpEstimator::p_f() const
  {
    return p_f_;
  }

  const bool &FootZmpEstimator::isSupport() const
  {
    return is_support_;
  }

} // namespace ow_zmp
