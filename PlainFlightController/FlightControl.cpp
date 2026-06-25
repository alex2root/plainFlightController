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
* @file   FlightControl.cpp (ADRC version)
* @brief  This class contains methods to operate the flight controller.
*         Modified to use ADRC (Active Disturbance Rejection Control) instead of PIDF.
*/

#include "FlightControl.hpp"
#include "InternalConfig.hpp"



/**
* @brief  Starts the flight control process  
*/
void
FlightControl::begin()
{
  modelConfig();

  Serial.begin(InternalConfig::USB_BAUD);
  if (Serial)
  {
    delay(3000);  //Need to wait by this magic number or some console data will be lost while PC is connecting.
    Serial.print("PlainFlightController ADRC: ");
    Serial.println(InternalConfig::SOFTWARE_VERSION);
  }

  imu.begin();
  imu.setMadgwickWeighting(IMU::MADGWICK_WARM_UP_WEIGHTING);
  imuData = imu.getImuData();

  if (imu.isFaulted())
  {
    Serial.println("Faulted");
    m_flightState = DemandProcessor::FlightState::FAULTED;
  }

  if (!config.begin())
  {
    Serial.println("Config not ok! rebooting...");
    delay(3000);
    ESP.restart();
  }

  myModel->begin();

  if constexpr(Config::ESP32S3.HAS_NEOPIXEL)
  {
    statusLedNeopixel.begin();
  }
  else
  {
    statusLed.begin();
  }

  if constexpr(Config::USE_EXTERNAL_LED)
  {
    externLed.begin();
  }

  batteryMonitor.begin(config.getBatteryScaler());
}


/**
* @brief  Instantiates the model type  
*/
void
FlightControl::modelConfig()
{
  if constexpr(Config::MODEL_TYPE == ModelType::PLANE_FULL_HOUSE)
  {
    myModel = new PlaneFullHouse();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::PLANE_FULL_HOUSE_V_TAIL)
  {
    myModel = new PlaneFullHouseVTail();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::PLANE_ADVANCED_RUDDER_ELEVATOR)
  {
    myModel = new PlaneAdvancedRudderElevator();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::PLANE_RUDDER_ELEVATOR)
  {
    myModel = new PlaneRudderElevator();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::PLANE_V_TAIL)
  {
    myModel = new PlaneVTail();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::PLANE_FLYING_WING)
  {
    myModel = new PlaneFlyingWing();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::QUAD_X_COPTER)
  {
    myModel = new QuadXCopter();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::QUAD_P_COPTER)
  {
    myModel = new QuadPlusCopter();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::BI_COPTER)
  {
    myModel = new BiCopter();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::CHINOOK_COPTER)
  {
    myModel = new ChinookCopter();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::TRI_COPTER)
  {
    myModel = new TriCopter();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::DUAL_COPTER)
  {
    myModel = new DualCopter();
  }
  else if constexpr(Config::MODEL_TYPE == ModelType::SINGLE_COPTER)
  {
    myModel = new SingleCopter();
  }
}


/**
* @brief  Runs the flight control process  
*/
void
FlightControl::operate()
{
  const float timedelta = loopRateControl();
  imu.operate(timedelta, &m_flightState);
  rc.process(&m_flightState, &m_lastFlightState, config.getRates(), config.getMaxAngles());
  checkStateChange();
  batteryMonitor.operate();

  if constexpr(Config::ESP32S3.HAS_NEOPIXEL)
  {
    statusLedNeopixel.operate(static_cast<uint32_t>(m_flightState));
  }
  else
  {
    statusLed.operate(static_cast<uint32_t>(m_flightState));
  }

  if constexpr(Config::USE_EXTERNAL_LED)
  {
    externLed.operate(static_cast<uint32_t>(m_flightState));
  }

  if (!imu.isOk())
  {
    DemandProcessor::FlightState::FAULTED;
  }

  switch (m_flightState)
  {
  case DemandProcessor::FlightState::CALIBRATE:
    doCalibrateState();
    break;

  case DemandProcessor::FlightState::WAITING_TO_DISARM:
  case DemandProcessor::FlightState::DISARMED:
    doDisarmedState();
    break;

  default://Intentional fall through
  case DemandProcessor::FlightState::PASS_THROUGH:
    doPassThroughState();
    break;

  case DemandProcessor::FlightState::RATE:
    doRateState();
    break;

  case DemandProcessor::FlightState::FAILSAFE:
    doFailSafeState();
    break;

  case DemandProcessor::FlightState::SELF_LEVELLED:
    doLevelledState();
    break;

  case DemandProcessor::FlightState::ACRO_TRAINER:
    doAcroTrainerState();
    break;

  case DemandProcessor::FlightState::AP_WIFI:
    doWifiApState();
    break;

  case DemandProcessor::FlightState::FAULTED:
    doFaultedState();
    break;

  case DemandProcessor::FlightState::PROP_HANG:
    doPropHangState();
    break;
  }

  //Compile in/out debug data
  if constexpr(InternalConfig::DEBUG_RC_DATA)
  {
    rc.printData();
  }
  else if constexpr(InternalConfig::DEBUG_LOOP_RATE)
  {
    printLoopRateData();
  }
  else if constexpr(InternalConfig::DEBUG_BATTERY_MONITOR)
  {
    batteryMonitor.debug();
  }
  else
  {
    ;//Nothing compiled in
  }
}


/**
* @brief  Allows the gyro to calibrate and Madgwick filter to gain position.
*/
void
FlightControl::doCalibrateState()
{
  if (imu.calibrateGyro())
  {
    //Complete so set madgwick to flight weighting...
    imu.setMadgwickWeighting(IMU::MADGWICK_FLIGHT_WEIGHTING);
    //Determine next state to go to...
    m_flightState = (rc.isArmed()) ? DemandProcessor::FlightState::WAITING_TO_DISARM : DemandProcessor::FlightState::DISARMED;
  }

  myModel->servoMixer(rc.getDemands(), config.getServoTrims());
  myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);  //Ensure motors do not operate
}


/**
* @brief  Defines what is done when disarmed.
*/
void
FlightControl::doDisarmedState()
{
  myModel->servoMixer(rc.getDemands(), config.getServoTrims());
  myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);  //Ensure motors do not operate
}


/**
* @brief  Defines what is done when in pass through mode.
*/
void
FlightControl::doPassThroughState()
{
  DemandProcessor::Demands demands = *rc.getDemands();
  myModel->servoMixer(&demands, config.getServoTrims());

  if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
  {
    demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), RxBase::MIN_NORMALISED);
  }

  myModel->motorMixer(&demands);
}


/**
* @brief  Defines what is done when in gyro rate mode.
*         ADRC version - replaces PIDF with ADRC control law
*/
void
FlightControl::doRateState()
{
  DemandProcessor::Demands demands = *rc.getDemands();
  processADRC(&demands);
  myModel->servoRateMixer(&demands, config.getServoTrims());

  if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
  {
    demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), RxBase::MIN_NORMALISED);
  }

  myModel->motorRateMixer(&demands);
}


/**
* @brief  Defines what is done when in self levelled mode.
*         ADRC version
*/
void
FlightControl::doFailSafeState()
{
  if constexpr (InternalConfig::MODEL_IS_MULTICOPTER)
  {
    //TODO - auto level & reduce throttle rather than just drop
    //For multicopters default demands to stop all motors from spinning.
    //Multicopters will fall out of the sky upon failsafe.
    myModel->servoMixer(&DemandProcessor::DEFAULT_DEMANDS, config.getServoTrims());
    myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);
  }
  else
  {
    doLevelledState();
  }
}


/**
* @brief  Defines what is done when in self levelled mode.
*         ADRC version - actively rejects disturbances (wind, etc.)
*/
void
FlightControl::doLevelledState()
{
  DemandProcessor::Demands demands = *rc.getDemands();

  //Angle demand is the error/difference between the stick demand and attitude of the model
  int32_t rollDemand = demands.roll - static_cast<int32_t>(((imuData->roll + config.getRollTrim()) * 100.0f));
  //Map error/difference from angle to degrees per second to produce roll rate
  rollDemand = map32(rollDemand, -config.getMaxRollAngle(), config.getMaxRollAngle(), -config.getRollRate(), config.getRollRate());
  demands.roll = constrain(rollDemand, -config.getRollRate(), config.getRollRate());
  //Repeat calculations for pitch...
  int32_t pitchDemand = demands.pitch - static_cast<int32_t>(((imuData->pitch + config.getPitchTrim()) * 100.0f));
  pitchDemand = map32(pitchDemand, -config.getMaxPitchAngle(), config.getMaxPitchAngle(), -config.getPitchRate(), config.getPitchRate());
  demands.pitch = constrain(pitchDemand, -config.getPitchRate(), config.getPitchRate());

  processADRC(&demands);
  myModel->servoRateMixer(&demands, config.getServoTrims());

  if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
  {
    demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), RxBase::MIN_NORMALISED);
  }

  myModel->motorRateMixer(&demands);
}


/**
* @brief  Defines what is done when in acro trainer mode.
* @note   When pitch and roll are centred we self level otherwise we operate in rate mode.
* @note   ADRC version provides smoother recovery without overshoot.
*/
void
FlightControl::doAcroTrainerState()
{
  DemandProcessor::Demands demands = *rc.getDemands();

  if ((0 == demands.pitch) && (0 == demands.roll))
  {
    //Sticks are centred so self level
    DemandProcessor::Demands demands = *rc.getDemands();

    //Angle demand is the error/difference between the stick demand and attitude of the model
    int32_t rollDemand = demands.roll - static_cast<int32_t>(((imuData->roll + config.getRollTrim()) * 100.0f));
    //Map error/difference from angle to degrees per second to produce roll rate
    rollDemand = map32(rollDemand, -config.getMaxRollAngle(), config.getMaxRollAngle(), -ACRO_TRAINER_RECOVERY_RATE, ACRO_TRAINER_RECOVERY_RATE);
    demands.roll = constrain(rollDemand, -ACRO_TRAINER_RECOVERY_RATE, ACRO_TRAINER_RECOVERY_RATE);

    //Repeat calculations for pitch...
    int32_t pitchDemand = demands.pitch - static_cast<int32_t>(((imuData->pitch + config.getPitchTrim()) * 100.0f));
    pitchDemand = map32(pitchDemand, -config.getMaxPitchAngle(), config.getMaxPitchAngle(), -ACRO_TRAINER_RECOVERY_RATE, ACRO_TRAINER_RECOVERY_RATE);
    demands.pitch = constrain(pitchDemand, -ACRO_TRAINER_RECOVERY_RATE, ACRO_TRAINER_RECOVERY_RATE);

    processADRC(&demands);
    myModel->servoRateMixer(&demands, config.getServoTrims());

    if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
    {
      demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), RxBase::MIN_NORMALISED);
    }

    myModel->motorRateMixer(&demands);
  }
  else
  {
    doRateState();
  }
}


/**
* @brief  Defines what is done when in prop hanging state.
*         ADRC version
*/
void
FlightControl::doPropHangState()
{
  DemandProcessor::Demands demands = *rc.getDemands();

  if constexpr(Config::PROP_HANG_TAIL_SITTER_MODE)
  {
    //Swap roll and yaw controls for prop hanging.
    int32_t rollDemand;

    if (Config::PROP_HANG_REVERSE_ROLL_DEMAND)
    {
      rollDemand = -demands.roll - static_cast<int32_t>(((imuData->yaw + config.getYawTrim()) * 100.0f));
    }
    else
    {
      rollDemand = demands.roll - static_cast<int32_t>(((imuData->yaw + config.getYawTrim()) * 100.0f));
    }

    if constexpr(Config::PROP_HANG_REVERSE_YAW_DEMAND)
    {
      demands.yaw = -demands.yaw;
    }

    demands.roll = demands.yaw;
    //Angle demand is the error/difference between the stick demand and attitude of the model
    //Map error/difference from angle to degrees per second to produce roll rate
    rollDemand = map32(rollDemand, -config.getMaxRollAngle(), config.getMaxRollAngle(), -config.getYawRate(), config.getYawRate());
    demands.yaw = constrain(rollDemand, -config.getYawRate(), config.getYawRate());
    //Repeat calculations for pitch...
    int32_t pitchDemand = demands.pitch - static_cast<int32_t>(((imuData->pitch + config.getPitchTrim()) * 100.0f));
    pitchDemand = map32(pitchDemand, -config.getMaxPitchAngle(), config.getMaxPitchAngle(), -config.getPitchRate(), config.getPitchRate());
    demands.pitch = constrain(pitchDemand, -config.getPitchRate(), config.getPitchRate());
  }
  else
  {
    //Angle demand is the error/difference between the stick demand and attitude of the model
    int32_t yawDemand = demands.yaw - static_cast<int32_t>(((imuData->yaw + config.getYawTrim()) * 100.0f));
    //Map error/difference from angle to degrees per second to produce roll rate
    yawDemand = map32(yawDemand, -config.getMaxRollAngle(), config.getMaxRollAngle(), -config.getYawRate(), config.getYawRate());
    demands.yaw = constrain(yawDemand, -config.getYawRate(), config.getYawRate());
    //Repeat calculations for pitch...
    int32_t pitchDemand = demands.pitch - static_cast<int32_t>(((imuData->pitch + config.getPitchTrim()) * 100.0f));
    pitchDemand = map32(pitchDemand, -config.getMaxPitchAngle(), config.getMaxPitchAngle(), -config.getPitchRate(), config.getPitchRate());
    demands.pitch = constrain(pitchDemand, -config.getPitchRate(), config.getPitchRate());
  }

  processADRC(&demands);
  myModel->servoRateMixer(&demands, config.getServoTrims());

  if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
  {
    demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), RxBase::MIN_NORMALISED);
  }

  myModel->motorRateMixer(&demands);
}


/**
* @brief  Defines what is done when in wifi configurator mode.
*/
void
FlightControl::doWifiApState()
{
  myModel->servoMixer(&DemandProcessor::DEFAULT_DEMANDS, config.getServoTrims());
  myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);

  batteryMonitor.setVoltageScaler(config.getBatteryScaler());
  config.updateBatteryVoltage(batteryMonitor.getVoltage());
  config.updateImuAngles((imuData->pitch + config.getPitchTrim()), (imuData->roll + config.getRollTrim()), (imuData->yaw + config.getYawTrim()));
  config.operate();
}


/**
* @brief  Defines what is done when faulted.
* @note   Only gets here from I2C read error, model will operate in pass through with no throttle.
* @note   If using multicopter then you are going to fall out of the sky !
*/
void
FlightControl::doFaultedState()
{
  //bad things happened
  const DemandProcessor::Demands demands = *rc.getDemands();
  myModel->servoMixer(&demands, config.getServoTrims());
  myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);
}


/**
* @brief  When flight state changes ESO state is reset.
*         This prevents windup and ensures clean transitions between modes.
*/
void
FlightControl::checkStateChange()
{
  if (m_flightState != m_lastFlightState)
  {
    rollADRC.reset();
    pitchADRC.reset();
    yawADRC.reset();
    m_lastFlightState = m_flightState;
  }
}


/**
* @brief  Processes pitch, roll and yaw using ADRC control law.
*         ADRC actively rejects disturbances like wind, friction, and sensor bias.
*
* Advantages over PIDF:
* - Estimates and compensates disturbances (wind gusts, mechanical play)
* - No D-gain noise amplification (observer handles filtering)
* - Universal (same gains work for different aircraft types)
* - Simpler tuning (omega_n, zeta, b_est, k_eso instead of P, I, D, F)
*
* @param demands Structure of demand upon the system
*/
void
FlightControl::processADRC(DemandProcessor::Demands * const demands)
{
  // Get time delta for ADRC observer integration
  static uint32_t lastTime = 0;
  uint32_t currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0f;  // Convert to seconds
  if (dt > 0.02f) dt = 0.02f;  // Cap at 20ms to prevent observer instability
  lastTime = currentTime;

  // Get current gyro rates from IMU (in degrees/second * 100)
  int32_t gyroX = static_cast<int32_t>(imuData->mpu6050.gyro_X * 100.0f);
  int32_t gyroY = static_cast<int32_t>(imuData->mpu6050.gyro_Y * 100.0f);
  int32_t gyroZ = static_cast<int32_t>(imuData->mpu6050.gyro_Z * 100.0f);

  // ADRC for Roll axis
  if constexpr(Config::REVERSE_ROLL_CORRECTIONS)
  {
    demands->roll = rollADRC.adrcController(demands->roll, -gyroX, config.getRollGains(), dt);
  }
  else
  {
    demands->roll = rollADRC.adrcController(demands->roll, gyroX, config.getRollGains(), dt);
  }

  // ADRC for Pitch axis
  if constexpr(Config::REVERSE_PITCH_CORRECTIONS)
  {
    demands->pitch = pitchADRC.adrcController(demands->pitch, -gyroY, config.getPitchGains(), dt);
  }
  else
  {
    demands->pitch = pitchADRC.adrcController(demands->pitch, gyroY, config.getPitchGains(), dt);
  }

  // ADRC for Yaw axis
  float gyro_Z = imuData->mpu6050.gyro_Z;

  if constexpr(Config::REVERSE_YAW_CORRECTIONS)
  {
    gyro_Z = -gyro_Z;
  }

  if constexpr(InternalConfig::MODEL_IS_MULTICOPTER)
  {
    //Multicopter yaw control
    demands->yaw = yawADRC.adrcController(demands->yaw, gyroZ, config.getYawGains(), dt);
  }
  else
  {
    //Fixed wing yaw control with heading hold option
    if constexpr(Config::USE_HEADING_HOLD)
    {
      if (rc.headingHoldActive() || rc.propHangActive())
      {
        //Apply ADRC with full disturbance rejection
        demands->yaw = yawADRC.adrcController(demands->yaw, gyroZ, config.getYawGains(), dt);
      }
      else
      {
        //Heading hold off - reset ADRC observer to prevent integrator windup
        yawADRC.reset();
        // Simple rate control without heading hold (no I term equivalent)
        // Use default gains but disable disturbance rejection
        ADRC::Gains yawGains = *config.getYawGains();
        yawGains.k_eso = 0.0f;  // Disable disturbance estimation
        demands->yaw = yawADRC.adrcController(demands->yaw, gyroZ, &yawGains, dt);
      }
    }
    else
    {
      //Never apply heading hold on yaw
      yawADRC.reset();
      ADRC::Gains yawGains = *config.getYawGains();
      yawGains.k_eso = 0.0f;
      demands->yaw = yawADRC.adrcController(demands->yaw, gyroZ, &yawGains, dt);
    }
  }
}
