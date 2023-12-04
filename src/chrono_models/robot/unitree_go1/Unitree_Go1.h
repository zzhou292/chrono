// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
//
// =============================================================================

#ifndef UNITREEGO1_H
#define UNITREEGO1_H

#include <string>
#include <array>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaft.h"

#include "chrono_parsers/ChParserURDF.h"
#include "chrono_parsers/ChRobotActuation.h"

#include "chrono_models/ChApiModels.h"
using namespace chrono::parsers;
namespace chrono {

/// Namespace with classes for the Unitree Go1 model.
namespace unitree_go1 {

/// @addtogroup robot_models_unitree_go1
/// @{

/// Cobra wheel/suspension identifiers.
enum UnitreeSideID {
    UA1_FL = 0,  ///< left front
    UA1_FR = 1,  ///< right front
    UA1_RL = 2,  ///< left back
    UA1_RR = 3   ///< right back
};

const double HIP_MOTOR_MIN = -0.802851455917f;
const double HIP_MOTOR_MAX = 0.802851455917f;

const double THIGH_MOTOR_MIN = -1.0471975512f;
const double THIGH_MOTOR_MAX = 4.18879020479f;

const double CALF_MOTOR_MIN = 0.916297857297f;
const double CALF_MOTOR_MAX = 2.69653369433f;


/// Cobra rover class.
/// This class encapsulates the location and rotation information of all Cobra parts wrt the chassis.
/// This class should be the entry point to create a complete rover.
class CH_MODELS_API Unitree_Go1 {
  public:
    Unitree_Go1(ChSystem* system);

    ~Unitree_Go1() {}

    /// Get the containing system.
    ChSystem* GetSystem() const { return m_system; }

    /// Initialize the Cobra rover at the specified position.
    void Initialize(const ChFrame<>& pos);

    void SetHipMotorPos(UnitreeSideID id, double pos, double x);
    void SetThighMotorPos(UnitreeSideID id, double pos, double x);
    void SetCalfMotorPos(UnitreeSideID id, double pos, double x);

    double GetHipMotorPos(UnitreeSideID id, double x);
    double GetThighMotorPos(UnitreeSideID id, double x);
    double GetCalfMotorPos(UnitreeSideID id, double x);

    std::shared_ptr<ChBody> GetTrunkBody() const { return m_trunk; }
    std::shared_ptr<ChBody> GetCalfBody(UnitreeSideID id) const { return m_calfs[id]; }
    std::shared_ptr<ChBody> GetThighBody(UnitreeSideID id) const { return m_thighs[id]; }
    std::shared_ptr<ChBody> GetHipBody(UnitreeSideID id) const { return m_hips[id]; }
    std::shared_ptr<ChBody> GetFootBody(UnitreeSideID id) const { return m_feet[id]; }

    std::shared_ptr<ChLinkMotor> GetHipMotor(UnitreeSideID id) const { return m_hip_motors[id]; }
    std::shared_ptr<ChLinkMotor> GetThighMotor(UnitreeSideID id) const { return m_thigh_motors[id]; }
    std::shared_ptr<ChLinkMotor> GetCalfMotor(UnitreeSideID id) const { return m_calf_motors[id]; }


  private:
    ChSystem* m_system;     ///< pointer to the Chrono system
    ChParserURDF* m_robot;  ///< URDF parser

    // Unitree Go1 Body Parts
    std::shared_ptr<ChBody> m_trunk;
    std::array<std::shared_ptr<ChBody>, 4> m_calfs;
    std::array<std::shared_ptr<ChBody>, 4> m_thighs;
    std::array<std::shared_ptr<ChBody>, 4> m_hips;
    std::array<std::shared_ptr<ChBody>, 4> m_feet;

    // Unitree Go1 Motors
    std::array<std::shared_ptr<ChLinkMotor>, 4> m_hip_motors;               ///< drive motors
    std::array<std::shared_ptr<ChFunction_Setpoint>, 4> m_hip_motor_funcs;  ///< drive motor functions

    std::array<std::shared_ptr<ChLinkMotor>, 4> m_thigh_motors;               ///< drive motors
    std::array<std::shared_ptr<ChFunction_Setpoint>, 4> m_thigh_motor_funcs;  ///< drive motor functions

    std::array<std::shared_ptr<ChLinkMotor>, 4> m_calf_motors;               ///< drive motors
    std::array<std::shared_ptr<ChFunction_Setpoint>, 4> m_calf_motor_funcs;  ///< drive motor functions
};

/// @} robot_models_unitree_go1

}  // namespace unitree_go1
}  // namespace chrono

#endif
