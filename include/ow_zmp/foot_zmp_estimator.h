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

#ifndef OPEN_WALKER_FOOT_ZMP_ESTIMATOR_H
#define OPEN_WALKER_FOOT_ZMP_ESTIMATOR_H

#include <ow_core/types.h>
#include <ow_core/algorithms.h>

#include <ow_core/algorithms/scalar_finite_difference.h>
#include <ow_core/algorithms/state_differentiator.h>

namespace ow_zmp
{

/*!
 * \brief The FootZmpEstimator class
 *
 * This class calculates the center of pressure of one foot sole from ankle
 * FT sensor wrench wrt Sole frame.
 *
 * The standard method to compute the ZMP is based directly on the FT sensor
 * information of each foot [Kajita et al. 2014].
 */
class FootZmpEstimator
{
public:
  typedef ow::StateDifferentiator<ow::LinearState> Differentiator;

protected:
  ow::LinearState p_f_;  //!< ZeroMomentPointState wrt foot frame.
  ow::Scalar min_force_thres_;    //!< Minimal force to compute ZMP.
  ow::Scalar d_;                  //!< Distance between sole and FT sensor frame.
  bool is_support_;               //!< Flag to fast check foot contact.

  /*!
   * \brief ZMP derivative updater.
   */
  std::unique_ptr<Differentiator> diff_;

public:
  /*!
   * \brief FootZmpEstimator
   *    Default constructor. Initializes the filer and the internal variables.
   */
  FootZmpEstimator();

  /*!
   * \brief Initialization.
   *
   * \param freq
   *    Update frequency.
   *
   * \param d
   *    Vertical distance from sole to FT sensor.
   *
   * \param min_force_thres
   *    Minimal force to compute \f$ \mathVecB{p}{F} \f$. Otherwise
   *
   * \return
   *    True on success.
   */
  bool init(
    ow::Scalar freq,
    ow::Scalar d,
    ow::Scalar min_force_thres);

  /*!
   * \brief Compute ZeroMomentPointState from FT sensor data in foot frame as
   * presented in [Kajita et al. 2014]. The ZMP is calculated as
   *
   * \f$ \mathVecQB{p}{x}{F}=\frac{-\mathVecSQB{\mu}{y}{F}-\mathVecQB{f}{x}{F}\,
   * \mathScalarQ{d}{F}}{\mathVecQB{f}{z}{F}} \f$
   *
   * \f$ \mathVecQB{p}{y}{F}=\frac{-\mathVecSQB{\mu}{x}{F}-\mathVecQB{f}{y}{F}\,
   * \mathScalarQ{d}{F}}{\mathVecQB{f}{z}{F}} \f$
   *
   * \f$ \mathVecB{p}{F}=\mathSBs{\mathCommaT{\mathVecQB{p}{x}{F}}{\mathVecQB{p}
   * {y}{F}}{0}}^{\mathOpT} \f$
   *
   * where \f$\mathName{F}=\mathCBs{\mathCommaD{\mathName{L}}{\mathName{R}}}\f$
   * stands for each foot frame (Left and Right), \f$ \mathVecB{FT}{F}=
   * \mathSBs{\mathCommaD{\mathVecB{f}{F}}{\mathVecSB{\mu} {F}}}\f$ are the
   * ankle FT sensor wrenches, and \f$\mathScalarQ{d}{F}\f$ is the FT sensor
   * offset of the foot F.
   *
   * \param W_f
   *    Ankle wrench wrt foot frame.
   *
   * \return
   *    ZeroMomentPointState wrt world frame containing
   *    \f$ \mathVecB{p}{F} \f$,
   *    \f$ \mathVecB{\dot{p}}{F} \f$, and
   *    \f$ \mathVecB{\ddot{p}}{F} \f$.
   */
  const ow::LinearState& updateWrtFoot(const ow::Wrench &W_f);  

  /*!
   * \brief Get ZeroMomentPointState wrt foot frame \f$ \mathVecB{P}{F} \f$.
   *
   * \return
   *   ZeroMomentPointState wrt foot frame \f$ \mathVecB{P}{F} \f$ containing
   *   \f$ \mathVecB{p}{F} \f$,
   *   \f$ \mathVecB{\dot{p}}{F} \f$, and
   *   \f$ \mathVecB{\ddot{p}}{F} \f$.
   */
  const ow::LinearState& p_f() const;

  /*!
   * \brief Check if this foot is in contact with the ground.
   *
   * \return
   *    True if the force vertical force in the FT sensor is greater than than
   *    min_force_thres_.
   */
  const bool &isSupport() const;
};

}

#endif  //  OPEN_WALKER_FOOT_ZMP_H
