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

#include <ow_zmp/zmp_estimator.h>

namespace ow_zmp
{

  ZmpEstimator::ZmpEstimator() : Base("zmp_estimator")
  {
  }

  ZmpEstimator::~ZmpEstimator()
  {
  }

  bool ZmpEstimator::init(const ow::Parameter &parameter, ros::NodeHandle &nh)
  {
    // get global ow parameter
    ow::Scalar frequency = parameter.get<ow::Scalar>("loop_rate");

    // load module parameter
    parameter_.add<ow::Scalar>("sensor_height", 0.0);
    parameter_.add<ow::Scalar>("f_threshold", 20.0);
    if (!parameter_.load(nh, "zmp"))
    {
      ROS_ERROR("%s::initialize: Config loading failed.", ModuleBase::name().c_str());
      return false;
    }

    // init members
    differentiator_.reset(new Differentiator(
        ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(frequency)));

    const ow::Scalar &d = parameter_.get<ow::Scalar>("sensor_height");
    const ow::Scalar &f_thres = parameter_.get<ow::Scalar>("f_threshold");
    left_foot_.init(frequency, d, f_thres);
    right_foot_.init(frequency, d, f_thres);

    return true;
  }

  void ZmpEstimator::update(
      ow::Flags &flags,
      const ow::Wrench &W_l,
      const ow::CartesianState &X_l_w,
      const ow::Wrench &W_r,
      const ow::CartesianState &X_r_w)
  {
    // Compute local CoP for each foot.
    const ow::LinearState &p_l = left_foot_.updateWrtFoot(W_l);
    const ow::LinearState &p_r = right_foot_.updateWrtFoot(W_r);

    // Transform foot Zmps and ankle forces to world frame.
    ow::LinearPosition p_l_w = ow::changeRefFrame(p_l.pos(), X_l_w.pos());
    ow::LinearPosition p_r_w = ow::changeRefFrame(p_r.pos(), X_r_w.pos());
    ow::Force f_l_w = ow::changeRefFrame(W_l.force(), X_l_w.pos());
    ow::Force f_r_w = ow::changeRefFrame(W_r.force(), X_r_w.pos());

    // compute zmp in world frame
    updateWrtWorld(p_l_w, f_l_w, p_r_w, f_r_w);

    // \todo: Update robot contacts move somewhere else
    flags.feetInContact()[ow::FootId::LEFT] = left_foot_.isSupport();
    flags.feetInContact()[ow::FootId::RIGHT] = right_foot_.isSupport();
    flags.isStanding() = left_foot_.isSupport() && right_foot_.isSupport();      
    flags.hasGroundContact() = left_foot_.isSupport() || right_foot_.isSupport();
  }

  void ZmpEstimator::updateWrtWorld(
      const ow::LinearPosition &p_l_w,
      const ow::Force &f_l_w,
      const ow::LinearPosition &p_r_w,
      const ow::Force &f_r_w)
  {
    // Compute global zmp in world coordinate frame.
    p_w_.x().x() = (p_r_w.x() * f_r_w.z() + p_l_w.x() * f_l_w.z()) / (f_r_w.z() + f_l_w.z());
    p_w_.x().y() = (p_r_w.y() * f_r_w.z() + p_l_w.y() * f_l_w.z()) / (f_r_w.z() + f_l_w.z());
    p_w_.x().z() = 0.0;

    // Update derivatives.
    p_w_ = differentiator_->update(p_w_);
  }

  const ow::LinearState &ZmpEstimator::ZMP_w() const
  {
    return p_w_;
  }

} // namespace ow_zmp
