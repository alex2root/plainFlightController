/* 
* Copyright (c) 2025 P.Cook (alias 'plainFlight')
*
* This file is part of the PlainFlightController distribution (https://github.com/plainFlight/plainFlightController).
* 
* This program is free software: you can redistribute it and/or modify  
* it under the terms of the GNU General Public License as published by  
* the Free Software Foundation, version 3.
*
* This program is distributed in the hope that it will be useful, but 
* WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/**
* @file   ADRC.hpp
* @brief  Active Disturbance Rejection Control with Linear Extended State Observer (ESO).
*         
* ADRC replaces traditional PID control with a disturbance observer-based approach.
* This implementation uses a linear ESO to estimate system state and disturbances,
* providing superior performance across different aircraft types without retuning.
*
* Key advantages over PIDF:
* - Universal: Works for planes, quads, etc. without model-specific parameters
* - Disturbance rejection: Actively compensates for wind, mechanical issues, sensor noise
* - Adaptive: Automatically learns system dynamics
* - Simple tuning: Only 2-3 parameters instead of 9+ for PIDF
*
* Theory:
* -------
* Traditional control: error = setpoint - measurement → correction based on error
*
* ADRC approach:
*   1. ESO observes actual state and estimates hidden disturbance f(t)
*   2. Control law: u = (setpoint - measurement) * gain - f_estimated
*   3. Result: Actively rejects disturbances instead of reacting to them
*
* ESO State representation:
*   z1 = angle (roll/pitch/yaw)
*   z2 = angular velocity
*   z3 = disturbance (wind, friction, sensor bias, etc.)
*
* System model: z2_dot = a·z2 + b·u + z3
*               z3_dot ≈ 0 (slow-changing disturbance assumption)
*/

#pragma once

#include <inttypes.h>
#include <cmath>

class ADRC
{
public:
  /**
   * @struct Gains
   * @brief  ADRC tuning parameters
   */
  struct Gains
  {
    float omega_n;    ///< Natural frequency (0.1-10 rad/s, typical 2-5). Higher = faster response
    float zeta;       ///< Damping ratio (typical 0.7-1.0, default 0.85 for critical damping)
    float b_est;      ///< Estimated control input coefficient (typical 1.0-2.0, learn by tuning)
    float k_eso;      ///< ESO observer gain multiplier (typical 1.0-3.0, higher = faster disturbance tracking)
  };

  struct AxisGains
  {
    Gains roll;
    Gains pitch;
    Gains yaw;
  };

  static constexpr int64_t ADRC_MAX_LIMIT = 10000;  ///< Output saturation limit

  ADRC();
  ~ADRC();

  /**
   * @brief  Process ADRC control law for single axis
   * @param  setPoint   Target angle/rate from user input (e.g., stick command)
   * @param  actualPoint Current measured angle/rate from IMU
   * @param  gains      ADRC gains for this axis
   * @param  dt         Time delta since last call (seconds)
   * @return Control output to be sent to servo/motor
   */
  int32_t adrcController(const int32_t setPoint, const int32_t actualPoint, 
                         const Gains* const gains, const float dt);

  /**
   * @brief  Get current estimated disturbance (useful for debugging)
   * @return Estimated disturbance in same units as output
   */
  int32_t getDisturbanceEstimate();

  /**
   * @brief  Reset ESO state (call when flight mode changes)
   */
  void reset();

  /**
   * @brief  Set observer gains manually (alternative to automatic calculation)
   * @param  kp, kd ESO observer pole placement
   */
  void setObserverGains(float kp, float kd);

private:
  // ESO state variables (Extended State Observer)
  float m_z1;           ///< Observed angle
  float m_z2;           ///< Observed angular velocity
  float m_z3;           ///< Observed disturbance
  
  // ESO gains (calculated from omega_n and zeta)
  float m_kp;           ///< Observer proportional gain
  float m_kd;           ///< Observer derivative gain
  float m_b0;           ///< Control coefficient for ESO
  
  // Debug/monitoring
  static constexpr bool DEBUG_ADRC = false;
  
  /**
   * @brief  Calculate ESO gains from natural frequency and damping
   * @param  omega_n Natural frequency (rad/s)
   * @param  zeta    Damping ratio (typically 0.7-1.0)
   * @param  k_eso   Observer speed multiplier
   */
  void calculateObserverGains(float omega_n, float zeta, float k_eso);
};
