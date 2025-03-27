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

#ifndef OPEN_WALKER_ZMP_ESTIMATOR_H
#define OPEN_WALKER_ZMP_ESTIMATOR_H

#include <ow_core/interfaces/i_zmp.h>

#include <ow_core/math.h>
#include <ow_core/algorithms.h>

#include <ow_zmp/foot_zmp_estimator.h>

/*!
 * \brief Open Walker ZMP module namespace. These classes implement the
 * components required to estimate the zero moment point.
 */
namespace ow_zmp
{

/*!
 * \brief The ZmpEstimator class
 *
 * This class calculates the zero moment point (ZMP) of the whole robot (both
 * feet) wrt world frame from ankle force torque sensor data [Kajita et al.
 * 2014].
 */
class ZmpEstimator : 
  public ow::IZmpEstimator
{
public:
  typedef ow::IZmpEstimator Base;
  typedef ow::StateDifferentiator<ow::LinearState> Differentiator;

protected:
  ow::Parameter parameter_;       //!< configuration of this module

  /*!
   * \brief State diferentiator.
   */
  std::unique_ptr<Differentiator> differentiator_;
  ow::LinearState p_w_;  //!< ZeroMomentPointState wrt world.
  FootZmpEstimator left_foot_;    //!< FootZmpEstimator for the left foot.
  FootZmpEstimator right_foot_;   //!< FootZmpEstimator for the right foot.

public:

  /*!
   * \brief ZmpEstimator
   *      Default constructor.
   */
  ZmpEstimator();

  virtual ~ZmpEstimator();

  /*!
   * \brief Compute foot ZMP from FT sensor data wrt world frame.
   *
   * First, the center of pressure (CoP) of each foot (\f$\mathVecB{p}{F}\f$)
   * is calculated in FootZmpEstimator objects separately.
   *
   * Then, the foot CoPs and ankle forces are transformed to world frame
   *
   * \f$ \mathVecQB{p}{F}{\mathFW}=\mathMatFB{T}{F}{\mathFW}\,\mathVecB{p}{F}\f$
   *
   * \f$ \mathVecB{f}{W}=\mathMatFB{R}{F}{\mathFW}\, \mathVecB{f}{F} \f$
   *
   * These CoP are then combined with a ponderation on the vertical contact
   * force [Kajita et al. 2014].
   *
   * \f$\mathVecQB{p}{x}{\mathFW}=\frac{\mathVecQBI{p}{R}{\mathFW}{x}\,
   * \mathVecQBI{f}{R}{\mathFW}{z}+\mathVecQBI{p}{L}{\mathFW}{x}\,
   * \mathVecQBI{f}{L}{\mathFW}{z}}{\mathVecQBI{f}{L}{\mathFW}{z}
   * +\mathVecQBI{f}{R}{\mathFW}{z}}\f$
   *
   * \f$\mathVecQB{p}{y}{\mathFW}=\frac{\mathVecQBI{p}{R}{\mathFW}{y}\,
   * \mathVecQBI{f}{R}{\mathFW}{z}+\mathVecQBI{p}{L}{\mathFW}{y}\,
   * \mathVecQBI{f}{L}{\mathFW}{z}}{\mathVecQBI{f}{L}{\mathFW}{z}
   * +\mathVecQBI{f}{R}{\mathFW}{z}} \f$
   *
   * where \f$ \mathVecQB{p}{R}{\mathFW} \f$, \f$ \mathVecQB{p}{L}{\mathFW} \f$
   * are the foot CoPs wrt world frame and \f$ \mathVecQBI{f}{R}{\mathFW}{z}\f$,
   * \f$ \mathVecQBI{f}{L}{\mathFW}{z} \f$ are the foot vertical forces wrt to
   * world frame.
   *
   * \param W_l
   *      Ankle FT sensor wrench from left foot wrt foot frame.
   *
   * \param X_l_w
   *      CartesianState of the left foot wrt. to world.
   *
   * \param W_r
   *      Ankle FT sensor wrench from right foot wrt foot frame.
   *
   * \param X_r_w
   *      CartesianState of the left foot wrt. to world.
   *
   * \param flags
   *      Open Walker Flags.
   *
   * \return
   *      ZeroMomentPointState wrt foot frame \f$ \mathVecB{P}{W} \f$ containing
   *      \f$ \mathVecB{p}{W} \f$,
   *      \f$ \mathVecB{\dot{p}}{W} \f$, and
   *      \f$ \mathVecB{\ddot{p}}{W} \f$.
   */
  void update(
      ow::Flags& flags,
      const ow::Wrench& W_l,
      const ow::CartesianState& X_l_w,
      const ow::Wrench& W_r,
      const ow::CartesianState& X_r_w);

  /*!
    * \brief Get ZeroMomentPointState wrt world frame \f$ \mathVecB{P}{W} \f$.
    *
    * \return
    *   ZeroMomentPointState wrt world frame \f$ \mathVecB{P}{W} \f$ containing
    *   \f$ \mathVecB{p}{W} \f$,
    *   \f$ \mathVecB{\dot{p}}{W} \f$, and
    *   \f$ \mathVecB{\ddot{p}}{W} \f$.
    */
  virtual const ow::LinearState& ZMP_w() const;

protected:

  /*!
   * \brief Initialization of InverseKinematics module
   */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

  /*!
   * \brief Compute foot ZMP from FT sensor data and foot Center of pressure
   * wrt world frame.
   *
   * These CoP are then combined with a ponderation on the vertical contact
   * force [Kajita et al. 2014].
   *
   * \f$\mathVecQB{p}{x}{\mathFW}=\frac{\mathVecQBI{p}{R}{\mathFW}{x}\,
   * \mathVecQBI{f}{R}{\mathFW}{z}+\mathVecQBI{p}{L}{\mathFW}{x}\,
   * \mathVecQBI{f}{L}{\mathFW}{z}}{\mathVecQBI{f}{L}{\mathFW}{z}
   * +\mathVecQBI{f}{R}{\mathFW}{z}}\f$
   *
   * \f$\mathVecQB{p}{y}{\mathFW}=\frac{\mathVecQBI{p}{R}{\mathFW}{y}\,
   * \mathVecQBI{f}{R}{\mathFW}{z}+\mathVecQBI{p}{L}{\mathFW}{y}\,
   * \mathVecQBI{f}{L}{\mathFW}{z}}{\mathVecQBI{f}{L}{\mathFW}{z}
   * +\mathVecQBI{f}{R}{\mathFW}{z}} \f$
   *
   * where \f$ \mathVecQB{p}{R}{\mathFW} \f$, \f$ \mathVecQB{p}{L}{\mathFW} \f$
   * are the foot CoPs wrt world frame and \f$ \mathVecQBI{f}{R}{\mathFW}{z}\f$,
   * \f$ \mathVecQBI{f}{L}{\mathFW}{z} \f$ are the foot vertical forces wrt to
   * world frame.
   *
   * \param p_l_w
   *      Left foot center of pressure wrt world frame.
   *
   * \param f_l_w
   *      Left foot ankle force wrt world frame.
   *
   * \param p_r_w
   *      Right foot center of pressure wrt world frame.
   *
   * \param f_r_w
   *      Right foot ankle force wrt world frame.
   *
   * \return
   *      ZeroMomentPointState wrt foot frame \f$ \mathVecB{P}{W} \f$ containing
   *      \f$ \mathVecB{p}{W} \f$,
   *      \f$ \mathVecB{\dot{p}}{W} \f$, and
   *      \f$ \mathVecB{\ddot{p}}{W} \f$.
   */
  void updateWrtWorld(
      const ow::LinearPosition& p_l_w,
      const ow::Force& f_l_w,
      const ow::LinearPosition& p_r_w,
      const ow::Force& f_r_w);
};

}

#endif // OPEN_WALKER_ZMP_MODULE_H
