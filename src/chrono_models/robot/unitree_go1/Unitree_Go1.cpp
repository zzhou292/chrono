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
#include <cmath>
#include <algorithm>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono/motion_functions/ChFunction_Setpoint.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChShaftsBody.h"

#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_models/robot/unitree_go1/Unitree_Go1.h"

namespace chrono {
namespace unitree_go1 {

// =============================================================================

// Default contact material for rover parts
std::shared_ptr<ChMaterialSurface> DefaultContactMaterial(ChContactMethod contact_method) {
    float mu = 0.9f;   // coefficient of friction
    float cr = 0.0f;   // coefficient of restitution
    float Y = 1.5e6f;  // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 5e6f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 5e6f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

Unitree_Go1::Unitree_Go1(ChSystem* system) : m_system(system) {
    m_robot = new ChParserURDF(GetChronoDataFile("robot/unitree-go1/urdf/go1.urdf"));
}

void Unitree_Go1::Initialize(const ChFrame<>& pos) {
    // Set root body pose
    m_robot->SetRootInitPose(pos);

    // Enable actuation for all joints
    m_robot->SetJointActuationType("FL_hip_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("FL_thigh_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("FL_calf_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("FR_hip_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("FR_thigh_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("FR_calf_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("RL_hip_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("RL_thigh_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("RL_calf_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("RR_hip_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("RR_thigh_joint", ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("RR_calf_joint", ChParserURDF::ActuationType::POSITION);

    // Create the Chrono model
    m_robot->PopulateSystem(*m_system);

    // Fix root body
    m_robot->GetRootChBody()->SetBodyFixed(false);

    m_trunk = m_robot->GetChBody("trunk");

    m_calfs[UA1_FL] = m_robot->GetChBody("FL_calf");
    m_calfs[UA1_FR] = m_robot->GetChBody("FR_calf");
    m_calfs[UA1_RL] = m_robot->GetChBody("RL_calf");
    m_calfs[UA1_RR] = m_robot->GetChBody("RR_calf");

    m_feet[UA1_FL] = m_robot->GetChBody("FL_foot");
    m_feet[UA1_FR] = m_robot->GetChBody("FR_foot");
    m_feet[UA1_RL] = m_robot->GetChBody("RL_foot");
    m_feet[UA1_RR] = m_robot->GetChBody("RR_foot");

    m_hips[UA1_FL] = m_robot->GetChBody("FL_hip");
    m_hips[UA1_FR] = m_robot->GetChBody("FR_hip");
    m_hips[UA1_RL] = m_robot->GetChBody("RL_hip");
    m_hips[UA1_RR] = m_robot->GetChBody("RR_hip");

    m_thighs[UA1_FL] = m_robot->GetChBody("FL_thigh");
    m_thighs[UA1_FR] = m_robot->GetChBody("FR_thigh");
    m_thighs[UA1_RL] = m_robot->GetChBody("RL_thigh");
    m_thighs[UA1_RR] = m_robot->GetChBody("RR_thigh");

    // Enable collsion and set contact material for selected bodies of the robot
    m_trunk->SetCollide(true);

    m_calfs[UA1_FL]->SetCollide(true);
    m_calfs[UA1_FR]->SetCollide(true);
    m_calfs[UA1_RL]->SetCollide(true);
    m_calfs[UA1_RR]->SetCollide(true);

    m_feet[UA1_FL]->SetCollide(true);
    m_feet[UA1_FR]->SetCollide(true);
    m_feet[UA1_RL]->SetCollide(true);
    m_feet[UA1_RR]->SetCollide(true);

    m_hips[UA1_FL]->SetCollide(true);
    m_hips[UA1_FR]->SetCollide(true);
    m_hips[UA1_RL]->SetCollide(true);
    m_hips[UA1_RR]->SetCollide(true);

    m_thighs[UA1_FL]->SetCollide(true);
    m_thighs[UA1_FR]->SetCollide(true);
    m_thighs[UA1_RL]->SetCollide(true);
    m_thighs[UA1_RR]->SetCollide(true);

    // ChContactMaterialData mat;
    // mat.mu = 0.8f;
    // mat.cr = 0.0f;
    // mat.Y = 1e7f;
    // auto cmat = mat.CreateMaterial(m_system->GetContactMethod());
    auto cmat = DefaultContactMaterial(m_system->GetContactMethod());

    m_trunk->GetCollisionModel()->SetAllShapesMaterial(cmat);

    m_calfs[UA1_FL]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_calfs[UA1_FR]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_calfs[UA1_RL]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_calfs[UA1_RR]->GetCollisionModel()->SetAllShapesMaterial(cmat);

    m_hips[UA1_FL]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_hips[UA1_FR]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_hips[UA1_RL]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_hips[UA1_RR]->GetCollisionModel()->SetAllShapesMaterial(cmat);

    m_thighs[UA1_FL]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_thighs[UA1_FR]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_thighs[UA1_RL]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_thighs[UA1_RR]->GetCollisionModel()->SetAllShapesMaterial(cmat);

    m_feet[UA1_FL]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_feet[UA1_FR]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_feet[UA1_RL]->GetCollisionModel()->SetAllShapesMaterial(cmat);
    m_feet[UA1_RR]->GetCollisionModel()->SetAllShapesMaterial(cmat);

    // Obtain motors
    std::vector<std::string> hip_motors_names = {"FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint"};
    std::vector<std::string> thigh_motors_names = {"FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint",
                                                   "RR_thigh_joint"};
    std::vector<std::string> calf_motors_names = {"FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"};

    for (int i = 0; i < 4; i++) {
        m_hip_motors[i] = m_robot->GetChMotor(hip_motors_names[i]);
        m_thigh_motors[i] = m_robot->GetChMotor(thigh_motors_names[i]);
        m_calf_motors[i] = m_robot->GetChMotor(calf_motors_names[i]);

        m_hip_motor_funcs[i] = chrono_types::make_shared<ChFunction_Setpoint>();
        m_thigh_motor_funcs[i] = chrono_types::make_shared<ChFunction_Setpoint>();
        m_calf_motor_funcs[i] = chrono_types::make_shared<ChFunction_Setpoint>();

        m_hip_motors[i]->SetMotorFunction(m_hip_motor_funcs[i]);
        m_thigh_motors[i]->SetMotorFunction(m_thigh_motor_funcs[i]);
        m_calf_motors[i]->SetMotorFunction(m_calf_motor_funcs[i]);
    }


}


void Unitree_Go1::SetHipMotorPos(UnitreeSideID id, double pos, double x){
    //pos = std::max(HIP_MOTOR_MIN, std::min(pos, HIP_MOTOR_MAX));
    pos = std::clamp(pos, HIP_MOTOR_MIN, HIP_MOTOR_MAX);
    m_hip_motor_funcs[id]->SetSetpoint(pos, x);
}

void Unitree_Go1::SetThighMotorPos(UnitreeSideID id, double pos, double x){
    pos = std::clamp(pos, THIGH_MOTOR_MIN, THIGH_MOTOR_MAX);
    m_thigh_motor_funcs[id]->SetSetpoint(pos, x);
}

void Unitree_Go1::SetCalfMotorPos(UnitreeSideID id, double pos, double x){
    pos = std::clamp(pos, CALF_MOTOR_MIN, CALF_MOTOR_MAX);
    m_calf_motor_funcs[id]->SetSetpoint(pos, x);
}

double Unitree_Go1::GetHipMotorPos(UnitreeSideID id, double x){
    return m_hip_motor_funcs[id]->Get_y(x);
}

double Unitree_Go1::GetThighMotorPos(UnitreeSideID id, double x){
    return m_thigh_motor_funcs[id]->Get_y(x);
}

double Unitree_Go1::GetCalfMotorPos(UnitreeSideID id, double x){
    return m_calf_motor_funcs[id]->Get_y(x);
}
// =============================================================================

}  // namespace unitree_go1
}  // namespace chrono
