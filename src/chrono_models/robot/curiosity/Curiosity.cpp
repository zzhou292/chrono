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
// Authors: Jason Zhou, Radu Serban
// =============================================================================
//
// NASA Curiosity Mars Rover Model Class.
// This class contains model for NASA's Curiosity Mars rover
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtils.h"

#include "chrono/functions/ChFunctionSetpoint.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_models/robot/curiosity/Curiosity.h"

namespace chrono {
namespace curiosity {

// =============================================================================

// Maximum steering angle
// hard limit: +- 95 deg, soft limit: +- 85 deg
static const double max_steer_angle = 85 * CH_DEG_TO_RAD;

// rover wheels positions
static const ChVector3d wheel_rel_pos_lf = ChVector3d(1.095, 1.063, 0.249);
static const ChVector3d wheel_rel_pos_rf = ChVector3d(1.095, -1.063, 0.249);
static const ChVector3d wheel_rel_pos_lm = ChVector3d(-0.089, 1.194, 0.249);
static const ChVector3d wheel_rel_pos_rm = ChVector3d(-0.089, -1.194, 0.249);
static const ChVector3d wheel_rel_pos_lb = ChVector3d(-1.163, 1.063, 0.249);
static const ChVector3d wheel_rel_pos_rb = ChVector3d(-1.163, -1.063, 0.249);

static const ChVector3d wheel_pos[] = {
    wheel_rel_pos_lf, wheel_rel_pos_rf,  // front (left/right)
    wheel_rel_pos_lm, wheel_rel_pos_rm,  // middle (left/right)
    wheel_rel_pos_lb, wheel_rel_pos_rb   // back (left/right)
};

// upright positions
static const ChVector3d sr_rel_pos_lf = ChVector3d(1.095, 1.063, 0.64);
static const ChVector3d sr_rel_pos_rf = ChVector3d(1.095, -1.063, 0.64);
static const ChVector3d sr_rel_pos_lb = ChVector3d(-1.163, 1.063, 0.64);
static const ChVector3d sr_rel_pos_rb = ChVector3d(-1.163, -1.063, 0.64);

// rocker/bogie positions
static const ChVector3d cr_rel_pos_lf = ChVector3d(0.214, 0.604, 0.8754);
static const ChVector3d cr_rel_pos_rf = ChVector3d(0.214, -0.604, 0.8754);
static const ChVector3d cr_rel_pos_lb = ChVector3d(-0.54, 0.845, 0.6433);
static const ChVector3d cr_rel_pos_rb = ChVector3d(-0.54, -0.845, 0.6433);

// differential components positions
static const ChVector3d tr_rel_pos_l = ChVector3d(0.214, 0.672, 1.144);
static const ChVector3d tr_rel_pos_r = ChVector3d(0.214, -0.672, 1.144);
static const ChVector3d tr_rel_pos_t = ChVector3d(-0.142, 0.0, 1.172);

// =============================================================================

// Default contact material for rover parts
std::shared_ptr<ChContactMaterial> DefaultContactMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.0f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
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
            return std::shared_ptr<ChContactMaterial>();
    }
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
std::shared_ptr<ChLinkLockRevolute> AddRevoluteJoint(std::shared_ptr<ChBodyAuxRef> body_1,
                                                     std::shared_ptr<ChBodyAuxRef> body_2,
                                                     std::shared_ptr<CuriosityChassis> chassis,
                                                     const ChVector3d& rel_joint_pos,
                                                     const ChQuaternion<>& rel_joint_rot) {
    const ChFrame<>& X_GP = chassis->GetBody()->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);                    // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                                    // global -> child

    auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
    joint->Initialize(body_1, body_2, ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    chassis->GetBody()->GetSystem()->AddLink(joint);

    return joint;
}

// Add a rotational speed motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotorSpeed(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<CuriosityChassis> chassis,
                                                        const ChVector3d& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a rotational angle motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationAngle> AddMotorAngle(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<CuriosityChassis> chassis,
                                                        const ChVector3d& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a rotational torque motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationTorque> AddMotorTorque(std::shared_ptr<ChBody> body1,
                                                          std::shared_ptr<ChBody> body2,
                                                          std::shared_ptr<CuriosityChassis> chassis,
                                                          const ChVector3d& rel_pos,
                                                          const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// ===============================================================================
//
// Base class for all Curiosity Part
CuriosityPart::CuriosityPart(const std::string& name,
                             const ChFrame<>& rel_pos,
                             std::shared_ptr<ChContactMaterial> mat,
                             bool collide)
    : m_name(name), m_pos(rel_pos), m_mat(mat), m_collide(collide), m_visualize(true) {}

void CuriosityPart::Construct(ChSystem* system) {
    m_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_body->SetName(m_name + "_body");
    m_body->SetMass(m_mass);
    m_body->SetInertiaXX(m_inertia);
    m_body->SetFrameCOMToRef(m_cog);

    // Add visualization shape
    if (m_visualize) {
        auto vis_mesh_file = GetChronoDataFile("robot/curiosity/obj/" + m_mesh_name + ".obj");
        auto trimesh_vis = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
        trimesh_vis->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
        trimesh_vis->RepairDuplicateVertexes(1e-9);                               // if meshes are not watertight

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh_vis);
        trimesh_shape->SetName(m_mesh_name);
        trimesh_shape->SetMutable(false);

        m_body->AddVisualShape(trimesh_shape);
    }

    // Add collision shape
    if (m_collide) {
        auto col_mesh_file = GetChronoDataFile("robot/curiosity/col/" + m_mesh_name + ".obj");
        auto trimesh_col = ChTriangleMeshConnected::CreateFromWavefrontFile(col_mesh_file, false, false);
        trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
        trimesh_col->RepairDuplicateVertexes(1e-9);                               // if meshes are not watertight

        auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_mat, trimesh_col, false, false, 0.005);
        m_body->AddCollisionShape(shape);

        m_body->EnableCollision(m_collide);
    }

    system->AddBody(m_body);
}

void CuriosityPart::CalcMassProperties(double density) {
    auto mesh_filename = GetChronoDataFile("robot/curiosity/col/" + m_mesh_name + ".obj");
    auto trimesh_col = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_filename, false, false);
    trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
    trimesh_col->RepairDuplicateVertexes(1e-9);                               // if meshes are not watertight

    double vol;
    ChVector3d cog_pos;
    ChMatrix33<> cog_rot;
    ChMatrix33<> inertia;
    trimesh_col->ComputeMassProperties(true, vol, cog_pos, inertia);
    ChInertiaUtils::PrincipalInertia(inertia, m_inertia, cog_rot);
    m_mass = density * vol;
    m_inertia *= density;
    m_cog = ChFrame<>(cog_pos, cog_rot);
}

void CuriosityPart::Initialize(std::shared_ptr<ChBodyAuxRef> chassis) {
    Construct(chassis->GetSystem());

    // Set absolute position
    ChFrame<> X_GC = chassis->GetFrameRefToAbs() * m_pos;
    m_body->SetFrameRefToAbs(X_GC);
}

// =============================================================================
//
// Rover Chassis
CuriosityChassis::CuriosityChassis(const std::string& name,
                                   CuriosityChassisType chassis_type,
                                   std::shared_ptr<ChContactMaterial> mat)
    : CuriosityPart(name, ChFrame<>(VNULL, QUNIT), mat, false), m_chassis_type(chassis_type) {
    switch (m_chassis_type) {
        case CuriosityChassisType::FullRover:
            m_mass = 750;
            m_inertia = ChVector3d(0.85, 0.85, 1.0) * m_mass;
            m_cog = ChFrame<>();
            m_mesh_name = "curiosity_chassis";
            break;

        case CuriosityChassisType::Scarecrow:
            m_mass = 200;
            m_inertia = ChVector3d(0.4, 0.4, 0.5) * m_mass;
            m_cog = ChFrame<>();
            m_mesh_name = "scarecrow_chassis";
            break;
    }
    m_color = ChColor(1.0f, 1.0f, 1.0f);
}

void CuriosityChassis::Initialize(ChSystem* system, const ChFrame<>& pos) {
    Construct(system);

    m_body->SetFrameRefToAbs(pos);
}

// Curiosity Wheel
CuriosityWheel::CuriosityWheel(const std::string& name,
                               const ChFrame<>& rel_pos,
                               std::shared_ptr<ChContactMaterial> mat,
                               CuriosityWheelType wheel_type)
    : CuriosityPart(name, rel_pos, mat, true) {
    switch (wheel_type) {
        default:
        case CuriosityWheelType::RealWheel:
            m_mesh_name = "curiosity_wheel";
            break;
        case CuriosityWheelType::SimpleWheel:
            m_mesh_name = "curiosity_simplewheel";
            break;
        case CuriosityWheelType::CylWheel:
            m_mesh_name = "curiosity_cylwheel";
            break;
    }
    m_color = ChColor(1.0f, 1.0f, 1.0f);

    double radius = 0.25;       // wheel radius
    double width = 0.40;        // wheel width
    double thickness = 0.8e-3;  // average thickness
    double density = 2700;      // aluminum
    double radius1 = radius - thickness;
    double vol = (CH_2PI * radius * width + CH_PI * radius * radius) * thickness;
    double gyration1 = (3 * (radius * radius + radius1 * radius1) + width * width) / 12.0;
    double gyration2 = (radius * radius + radius1 * radius1) / 2.0;
    m_mass = density * vol;
    m_inertia = ChVector3d(gyration1, gyration2, gyration1) * m_mass;
    m_cog = ChFrame<>();
}

// Curiosity suspension rocker
CuriosityRocker::CuriosityRocker(const std::string& name,
                                 const ChFrame<>& rel_pos,
                                 std::shared_ptr<ChContactMaterial> mat,
                                 int side)
    : CuriosityPart(name, rel_pos, mat, false) {
    m_mesh_name = (side == 0) ? "curiosity_F_L_arm" : "curiosity_F_R_arm";
    m_color = ChColor(1.0f, 0.4f, 0.0f);

    // Titanium tubing (use an average density)
    CalcMassProperties(2000);
}

// Curiosity suspension bogie
CuriosityBogie::CuriosityBogie(const std::string& name,
                               const ChFrame<>& rel_pos,
                               std::shared_ptr<ChContactMaterial> mat,
                               int side)
    : CuriosityPart(name, rel_pos, mat, false) {
    m_mesh_name = (side == 0) ? "curiosity_B_L_arm" : "curiosity_B_R_arm";
    m_color = ChColor(0.4f, 1.0f, 0.0f);

    // Titanium tubing (use an average density)
    CalcMassProperties(2000);
}

// Curiosity steering rod
CuriosityUpright::CuriosityUpright(const std::string& name,
                                   const ChFrame<>& rel_pos,
                                   std::shared_ptr<ChContactMaterial> mat)
    : CuriosityPart(name, rel_pos, mat, false) {
    m_mesh_name = "curiosity_steer";
    m_color = ChColor(0.4f, 0.4f, 0.7f);

    m_mass = 6;
    m_inertia = ChVector3d(0.007, 0.007, 0.004) * m_mass;
    m_cog = ChFrame<>();
}

// Curiosity differential bar
CuriosityDifferentialBar::CuriosityDifferentialBar(const std::string& name,
                                                   const ChFrame<>& rel_pos,
                                                   std::shared_ptr<ChContactMaterial> mat)
    : CuriosityPart(name, rel_pos, mat, false) {
    m_mesh_name = "curiosity_balancer";
    m_color = ChColor(0.4f, 0.4f, 0.7f);

    CalcMassProperties(1000);
}

// Curiosity differential links
CuriosityDifferentialLink::CuriosityDifferentialLink(const std::string& name,
                                                     const ChFrame<>& rel_pos,
                                                     std::shared_ptr<ChContactMaterial> mat,
                                                     int side)
    : CuriosityPart(name, rel_pos, mat, false) {
    m_mesh_name = (side == 0) ? "curiosity_bar_L" : "curiosity_bar_R";
    m_color = ChColor(0.4f, 0.4f, 0.7f);

    CalcMassProperties(1000);
}

// ==========================================================

Curiosity::Curiosity(ChSystem* system, CuriosityChassisType chassis_type, CuriosityWheelType wheel_type)
    : m_system(system), m_initialized(false) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = m_system->GetContactMethod();
    if (contact_method == ChContactMethod::NSC) {
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    // Create the contact materials
    m_default_material = DefaultContactMaterial(contact_method);
    m_wheel_material = DefaultContactMaterial(contact_method);

    Create(chassis_type, wheel_type);
}

void Curiosity::Create(CuriosityChassisType chassis_type, CuriosityWheelType wheel_type) {
    // create rover chassis
    ChQuaternion<> body_rot;
    m_chassis = chrono_types::make_shared<CuriosityChassis>("chassis", chassis_type, m_default_material);

    // Create 6 Curiosity Rover wheels
    for (int i = 0; i < 6; i++) {
        m_wheels[i] = chrono_types::make_shared<CuriosityWheel>("wheel", ChFrame<>(wheel_pos[i], QUNIT),
                                                                m_wheel_material, wheel_type);
    }
    m_wheels[C_RF]->m_mesh_xform = ChFrame<>(VNULL, QuatFromAngleZ(CH_PI));
    m_wheels[C_RM]->m_mesh_xform = ChFrame<>(VNULL, QuatFromAngleZ(CH_PI));
    m_wheels[C_RB]->m_mesh_xform = ChFrame<>(VNULL, QuatFromAngleZ(CH_PI));

    // Create Curiosity suspension rockers and bogies
    m_rockers[0] = chrono_types::make_shared<CuriosityRocker>(  //
        "rocker_L", ChFrame<>(cr_rel_pos_lf, QUNIT), m_default_material, 0);
    m_rockers[1] = chrono_types::make_shared<CuriosityRocker>(  //
        "rocker_R", ChFrame<>(cr_rel_pos_rf, QUNIT), m_default_material, 1);

    m_bogies[0] = chrono_types::make_shared<CuriosityBogie>(  //
        "bogie_L", ChFrame<>(cr_rel_pos_lb, QUNIT), m_default_material, 0);
    m_bogies[1] = chrono_types::make_shared<CuriosityBogie>(  //
        "bogie_R", ChFrame<>(cr_rel_pos_rb, QUNIT), m_default_material, 1);

    // Create the steering uprights
    ChQuaternion<> steer_rot = QuatFromAngleZ(CH_PI);

    m_rocker_uprights[0] = chrono_types::make_shared<CuriosityUpright>(  //
        "upright_FL", ChFrame<>(sr_rel_pos_lf, QUNIT), m_default_material);
    m_rocker_uprights[1] = chrono_types::make_shared<CuriosityUpright>(  //
        "upright_FR", ChFrame<>(sr_rel_pos_rf, steer_rot), m_default_material);

    m_bogie_uprights[0] = chrono_types::make_shared<CuriosityUpright>(  //
        "upright_RL", ChFrame<>(sr_rel_pos_lb, QUNIT), m_default_material);
    m_bogie_uprights[1] = chrono_types::make_shared<CuriosityUpright>(  //
        "upright_RR", ChFrame<>(sr_rel_pos_rb, steer_rot), m_default_material);

    // Create the differential components
    m_diff_bar = chrono_types::make_shared<CuriosityDifferentialBar>(  //
        "diff bar", ChFrame<>(tr_rel_pos_t, QUNIT), m_default_material);
    m_diff_links[0] = chrono_types::make_shared<CuriosityDifferentialLink>(  //
        "diff link L", ChFrame<>(tr_rel_pos_l, QUNIT), m_default_material, 0);
    m_diff_links[1] = chrono_types::make_shared<CuriosityDifferentialLink>(  //
        "diff link R", ChFrame<>(tr_rel_pos_r, QUNIT), m_default_material, 1);

    // Create drive shafts
    for (int i = 0; i < 6; i++) {
        m_drive_shafts[i] = chrono_types::make_shared<ChShaft>();
    }
}

void Curiosity::Initialize(const ChFrame<>& pos) {
    assert(m_driver);

    // Initialize rover parts, fixing bodies to ground as requested
    m_chassis->Initialize(m_system, pos);
    m_diff_bar->Initialize(m_chassis->GetBody());

    for (int i = 0; i < 2; i++) {
        m_rockers[i]->Initialize(m_chassis->GetBody());
        m_bogies[i]->Initialize(m_chassis->GetBody());

        m_diff_links[i]->Initialize(m_chassis->GetBody());

        m_rocker_uprights[i]->Initialize(m_chassis->GetBody());
        m_bogie_uprights[i]->Initialize(m_chassis->GetBody());
    }
    for (int i = 0; i < 6; i++) {
        m_wheels[i]->Initialize(m_chassis->GetBody());
    }

    // Add drive motors on all six wheels
    ChQuaternion<> z2y = QuatFromAngleX(CH_PI_2);  // align Z with (negative) Y

    // Front (wheels attached to rocker uprights)
    for (int i = 0; i < 2; i++) {
        switch (m_driver->GetDriveMotorType()) {
            case CuriosityDriver::DriveMotorType::SPEED:
                m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunctionSetpoint>();
                m_drive_motors[i] = AddMotorSpeed(m_rocker_uprights[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis,
                                                  wheel_pos[i], z2y);
                m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
                break;
            case CuriosityDriver::DriveMotorType::TORQUE:
                AddRevoluteJoint(m_rocker_uprights[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis, wheel_pos[i], z2y);
                break;
        }
    }

    // Middle (wheeled attached to bogies)
    for (int i = 2; i < 4; i++) {
        switch (m_driver->GetDriveMotorType()) {
            case CuriosityDriver::DriveMotorType::SPEED:
                m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunctionSetpoint>();
                m_drive_motors[i] =
                    AddMotorSpeed(m_bogies[i - 2]->GetBody(), m_wheels[i]->GetBody(), m_chassis, wheel_pos[i], z2y);
                m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
                break;
            case CuriosityDriver::DriveMotorType::TORQUE:
                AddRevoluteJoint(m_bogies[i - 2]->GetBody(), m_wheels[i]->GetBody(), m_chassis, wheel_pos[i], z2y);
                break;
        }
    }

    // Rear (wheels attached to bogie uprights)
    for (int i = 4; i < 6; i++) {
        switch (m_driver->GetDriveMotorType()) {
            case CuriosityDriver::DriveMotorType::SPEED:
                m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunctionSetpoint>();
                m_drive_motors[i] = AddMotorSpeed(m_bogie_uprights[i - 4]->GetBody(), m_wheels[i]->GetBody(), m_chassis,
                                                  wheel_pos[i], z2y);
                m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
                break;
            case CuriosityDriver::DriveMotorType::TORQUE:
                AddRevoluteJoint(m_bogie_uprights[i - 4]->GetBody(), m_wheels[i]->GetBody(), m_chassis, wheel_pos[i],
                                 z2y);
                break;
        }
    }

    double J = 0.1;  // shaft rotational inertia
    for (int i = 0; i < 6; i++) {
        m_drive_shafts[i]->SetInertia(J);
        m_system->Add(m_drive_shafts[i]);

        // Connect shaft aligned with the wheel's axis of rotation (local wheel Y).
        // Set connection such that a positive torque applied to the shaft results in forward rover motion.
        auto shaftbody_connection = chrono_types::make_shared<ChShaftBodyRotation>();
        shaftbody_connection->Initialize(m_drive_shafts[i], m_wheels[i]->GetBody(), ChVector3d(0, -1, 0));
        m_system->Add(shaftbody_connection);
    }

    // Add steering motors
    ChVector3d rocker_motor_loc[] = {sr_rel_pos_lf, sr_rel_pos_rf};
    for (int i = 0; i < 2; i++) {
        m_rocker_motor_funcs[i] = chrono_types::make_shared<ChFunctionConst>(0.0);
        m_rocker_motors[i] = AddMotorAngle(m_rocker_uprights[i]->GetBody(), m_rockers[i]->GetBody(), m_chassis,
                                           rocker_motor_loc[i], QuatFromAngleX(CH_PI));
        m_rocker_motors[i]->SetMotorFunction(m_rocker_motor_funcs[i]);
    }

    ChVector3d bogie_motor_loc[] = {sr_rel_pos_lb, sr_rel_pos_rb};
    for (int i = 0; i < 2; i++) {
        m_bogie_motor_funcs[i] = chrono_types::make_shared<ChFunctionConst>(0.0);
        m_bogie_motors[i] =
            AddMotorAngle(m_bogie_uprights[i]->GetBody(), m_bogies[i]->GetBody(), m_chassis, bogie_motor_loc[i], QUNIT);
        m_bogie_motors[i]->SetMotorFunction(m_bogie_motor_funcs[i]);
    }

    // Connect the differential bar to chassis
    m_diff_joint = AddRevoluteJoint(m_diff_bar->GetBody(), m_chassis->GetBody(), m_chassis, tr_rel_pos_t, QUNIT);

    // Connect differential links to differential bar and to the rockers
    AddRevoluteJoint(m_rockers[0]->GetBody(), m_diff_links[0]->GetBody(), m_chassis, tr_rel_pos_l, z2y);
    AddRevoluteJoint(m_rockers[1]->GetBody(), m_diff_links[1]->GetBody(), m_chassis, tr_rel_pos_r, z2y);
    AddRevoluteJoint(m_diff_links[0]->GetBody(), m_diff_bar->GetBody(), m_chassis,
                     tr_rel_pos_l + ChVector3d(tr_rel_pos_t.x() - tr_rel_pos_l.x(), 0, 0), QUNIT);
    AddRevoluteJoint(m_diff_links[1]->GetBody(), m_diff_bar->GetBody(), m_chassis,
                     tr_rel_pos_r + ChVector3d(tr_rel_pos_t.x() - tr_rel_pos_r.x(), 0, 0), QUNIT);

    // Add revolute joint for suspension rockers and bogies
    m_rocker_joints[0] = AddRevoluteJoint(m_rockers[0]->GetBody(), m_chassis->GetBody(), m_chassis, cr_rel_pos_lf, z2y);
    m_rocker_joints[1] = AddRevoluteJoint(m_rockers[1]->GetBody(), m_chassis->GetBody(), m_chassis, cr_rel_pos_rf, z2y);
    m_bogie_joints[0] =
        AddRevoluteJoint(m_bogies[0]->GetBody(), m_rockers[0]->GetBody(), m_chassis, cr_rel_pos_lb, z2y);
    m_bogie_joints[1] =
        AddRevoluteJoint(m_bogies[1]->GetBody(), m_rockers[1]->GetBody(), m_chassis, cr_rel_pos_rb, z2y);

    // Set initialization flag
    m_initialized = true;
}

void Curiosity::SetWheelContactMaterial(std::shared_ptr<ChContactMaterial> mat) {
    for (auto& wheel : m_wheels)
        wheel->m_mat = mat;
}

void Curiosity::SetDriver(std::shared_ptr<CuriosityDriver> driver) {
    m_driver = driver;
    m_driver->curiosity = this;
}

void Curiosity::SetChassisVisualization(bool state) {
    m_chassis->SetVisualize(state);
}

void Curiosity::SetWheelVisualization(bool state) {
    for (auto& wheel : m_wheels)
        wheel->SetVisualize(state);
}

void Curiosity::SetSuspensionVisualization(bool state) {
    m_diff_bar->SetVisualize(state);
    for (int i = 0; i < 2; i++) {
        m_rockers[i]->SetVisualize(state);
        m_bogies[i]->SetVisualize(state);
        m_diff_links[i]->SetVisualize(state);
        m_rocker_uprights[i]->SetVisualize(state);
        m_bogie_uprights[i]->SetVisualize(state);
    }
}

void Curiosity::FixChassis(bool fixed) {
    if (!m_initialized)
        return;
    m_chassis->GetBody()->SetFixed(fixed);
}

void Curiosity::FixSuspension(bool fixed) {
    if (!m_initialized)
        return;

    m_rocker_joints[0]->Lock(fixed);
    m_rocker_joints[1]->Lock(fixed);

    m_bogie_joints[0]->Lock(fixed);
    m_bogie_joints[1]->Lock(fixed);
}

ChVector3d Curiosity::GetWheelContactForce(CuriosityWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactForce();
}

ChVector3d Curiosity::GetWheelContactTorque(CuriosityWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactTorque();
}

ChVector3d Curiosity::GetWheelAppliedForce(CuriosityWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedForce();
}

ChVector3d Curiosity::GetWheelAppliedTorque(CuriosityWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedTorque();
}

double Curiosity::GetWheelTracTorque(CuriosityWheelID id) const {
    if (m_driver->GetDriveMotorType() == CuriosityDriver::DriveMotorType::TORQUE)
        return 0;

    return m_drive_motors[id]->GetMotorTorque();
}

double Curiosity::GetRoverMass() const {
    double tot_mass = m_chassis->GetBody()->GetMass() + m_diff_bar->GetBody()->GetMass();
    for (int i = 0; i < 2; i++) {
        tot_mass = tot_mass + m_rockers[i]->GetBody()->GetMass();
        tot_mass = tot_mass + m_bogies[i]->GetBody()->GetMass();
        tot_mass = tot_mass + m_diff_links[i]->GetBody()->GetMass();
        tot_mass = tot_mass + m_rocker_uprights[i]->GetBody()->GetMass();
        tot_mass = tot_mass + m_bogie_uprights[i]->GetBody()->GetMass();
    }
    for (int i = 0; i < 6; i++) {
        tot_mass = tot_mass + m_wheels[i]->GetBody()->GetMass();
    }
    return tot_mass;
}

double Curiosity::GetWheelMass() const {
    return m_wheels[0]->GetBody()->GetMass();
}

void Curiosity::Update() {
    double time = m_system->GetChTime();
    m_driver->Update(time);

    if (m_driver->GetDriveMotorType() == CuriosityDriver::DriveMotorType::SPEED) {
        for (int i = 0; i < 6; i++) {
            double driving = m_driver->drive_speeds[i];
            m_drive_motor_funcs[i]->SetSetpoint(driving, time);
        }
    }

    for (int i = 0; i < 2; i++) {
        double steering = m_driver->steer_angles[i];
        // Enforce maximum steering angle
        ChClampValue(steering, -max_steer_angle, +max_steer_angle);
        m_rocker_motor_funcs[i]->SetConstant(steering);
    }

    for (int i = 0; i < 2; i++) {
        double steering = m_driver->steer_angles[i + 2];
        // Enforce maximum steering angle
        ChClampValue(steering, -max_steer_angle, +max_steer_angle);
        m_bogie_motor_funcs[i]->SetConstant(steering);
    }
}

std::vector<std::pair<std::shared_ptr<chrono::ChBody>, std::string>> Curiosity::GetCollidableBodiesWithPaths() const {
    std::vector<std::pair<std::shared_ptr<chrono::ChBody>, std::string>> return_bodies_list;

    // Chassis
    if (m_chassis->GetCollision()) {
        return_bodies_list.push_back(std::make_pair(
            m_chassis->GetBody(), GetChronoDataFile("robot/curiosity/col/" + m_chassis->GetMeshName() + ".obj")));
    } else {
        return_bodies_list.push_back(std::make_pair(m_chassis->GetBody(), std::string("")));
    }

    // Wheels
    for (auto& wheel : m_wheels) {
        if (wheel->GetCollision()) {
            return_bodies_list.push_back(std::make_pair(
                wheel->GetBody(), GetChronoDataFile("robot/curiosity/col/" + wheel->GetMeshName() + ".obj")));
        } else {
            return_bodies_list.push_back(std::make_pair(wheel->GetBody(), std::string("")));
        }
    }

    // Rocker
    for (auto& rocker : m_rockers) {
        if (rocker->GetCollision()) {
            return_bodies_list.push_back(std::make_pair(
                rocker->GetBody(), GetChronoDataFile("robot/curiosity/col/" + rocker->GetMeshName() + ".obj")));
        } else {
            return_bodies_list.push_back(std::make_pair(rocker->GetBody(), std::string("")));
        }
    }

    // Bogie
    for (auto& bogie : m_bogies) {
        if (bogie->GetCollision()) {
            return_bodies_list.push_back(std::make_pair(
                bogie->GetBody(), GetChronoDataFile("robot/curiosity/col/" + bogie->GetMeshName() + ".obj")));
        } else {
            return_bodies_list.push_back(std::make_pair(bogie->GetBody(), std::string("")));
        }
    }

    // rocker_uprights
    for (auto& rocker_upright : m_rocker_uprights) {
        if (rocker_upright->GetCollision()) {
            return_bodies_list.push_back(
                std::make_pair(rocker_upright->GetBody(),
                               GetChronoDataFile("robot/curiosity/col/" + rocker_upright->GetMeshName() + ".obj")));
        } else {
            return_bodies_list.push_back(std::make_pair(rocker_upright->GetBody(), std::string("")));
        }
    }

    // bogie_uprights
    for (auto& bogie_upright : m_bogie_uprights) {
        if (bogie_upright->GetCollision()) {
            return_bodies_list.push_back(
                std::make_pair(bogie_upright->GetBody(),
                               GetChronoDataFile("robot/curiosity/col/" + bogie_upright->GetMeshName() + ".obj")));
        } else {
            return_bodies_list.push_back(std::make_pair(bogie_upright->GetBody(), std::string("")));
        }
    }

    return return_bodies_list;
}

std::vector<std::pair<std::shared_ptr<chrono::ChBody>, std::string>> Curiosity::GetVisualBodiesWithPaths() const {
    std::vector<std::pair<std::shared_ptr<chrono::ChBody>, std::string>> bodies;

    // Chassis
    if (m_chassis->GetVisualize()) {
        bodies.push_back(std::make_pair(m_chassis->GetBody(),
                                        GetChronoDataFile("robot/curiosity/obj/" + m_chassis->GetMeshName() + ".obj")));
    } else {
        bodies.push_back(std::make_pair(m_chassis->GetBody(), std::string("")));
    }

    // Wheels
    for (auto& wheel : m_wheels) {
        if (wheel->GetVisualize()) {
            bodies.push_back(std::make_pair(wheel->GetBody(),
                                            GetChronoDataFile("robot/curiosity/obj/" + wheel->GetMeshName() + ".obj")));
        } else {
            bodies.push_back(std::make_pair(wheel->GetBody(), std::string("")));
        }
    }

    // Rocker
    for (auto& rocker : m_rockers) {
        if (rocker->GetVisualize()) {
            bodies.push_back(std::make_pair(
                rocker->GetBody(), GetChronoDataFile("robot/curiosity/obj/" + rocker->GetMeshName() + ".obj")));
        } else {
            bodies.push_back(std::make_pair(rocker->GetBody(), std::string("")));
        }
    }

    // Bogie
    for (auto& bogie : m_bogies) {
        if (bogie->GetVisualize()) {
            bodies.push_back(std::make_pair(bogie->GetBody(),
                                            GetChronoDataFile("robot/curiosity/obj/" + bogie->GetMeshName() + ".obj")));
        } else {
            bodies.push_back(std::make_pair(bogie->GetBody(), std::string("")));
        }
    }

    // rocker_uprights
    for (auto& rocker_upright : m_rocker_uprights) {
        if (rocker_upright->GetVisualize()) {
            bodies.push_back(
                std::make_pair(rocker_upright->GetBody(),
                               GetChronoDataFile("robot/curiosity/obj/" + rocker_upright->GetMeshName() + ".obj")));
        } else {
            bodies.push_back(std::make_pair(rocker_upright->GetBody(), std::string("")));
        }
    }

    // bogie_uprights
    for (auto& bogie_upright : m_bogie_uprights) {
        if (bogie_upright->GetVisualize()) {
            bodies.push_back(
                std::make_pair(bogie_upright->GetBody(),
                               GetChronoDataFile("robot/curiosity/obj/" + bogie_upright->GetMeshName() + ".obj")));
        } else {
            bodies.push_back(std::make_pair(bogie_upright->GetBody(), std::string("")));
        }
    }

    return bodies;
}

std::vector<std::pair<std::shared_ptr<chrono::ChBody>, ChFrame<>>> Curiosity::GetMeshTransforms() const {
    std::vector<std::pair<std::shared_ptr<chrono::ChBody>, ChFrame<>>> transforms;

    // chassis
    transforms.push_back(std::make_pair(m_chassis->GetBody(), ChFrame<>(ChVector3d(0, 0, 0), QUNIT)));

    // wheels
    for (int i = 0; i < 6; i++) {
        if (i == C_RF || i == C_RM || i == C_RB)
            transforms.push_back(
                std::make_pair(m_wheels[i]->GetBody(), ChFrame<>(ChVector3d(0, 0, 0), QuatFromAngleZ(CH_PI))));
        else
            transforms.push_back(std::make_pair(m_wheels[i]->GetBody(), ChFrame<>(ChVector3d(0, 0, 0), QUNIT)));
    }
    for (auto& wheel : m_wheels) {
        transforms.push_back(std::make_pair(wheel->GetBody(), ChFrame<>(ChVector3d(0, 0, 0), QUNIT)));
    }

    // Rocker
    for (auto& rocker : m_rockers) {
        transforms.push_back(std::make_pair(rocker->GetBody(), ChFrame<>(ChVector3d(0, 0, 0), QUNIT)));
    }

    // Bogie
    for (auto& bogie : m_bogies) {
        transforms.push_back(std::make_pair(bogie->GetBody(), ChFrame<>(ChVector3d(0, 0, 0), QUNIT)));
    }

    // rocker_uprights
    for (auto& rocker_upright : m_rocker_uprights) {
        transforms.push_back(std::make_pair(rocker_upright->GetBody(), ChFrame<>(ChVector3d(0, 0, 0), QUNIT)));
    }

    // bogie_uprights
    for (auto& bogie_upright : m_bogie_uprights) {
        transforms.push_back(std::make_pair(bogie_upright->GetBody(), ChFrame<>(ChVector3d(0, 0, 0), QUNIT)));
    }

    return transforms;
}

std::vector<std::pair<std::shared_ptr<chrono::ChBody>, unsigned int>> Curiosity::GetBodyIndices() const {
    std::vector<std::pair<std::shared_ptr<chrono::ChBody>, unsigned int>> body_indices;

    assert(m_initialized);

    // chassis
    body_indices.push_back(std::make_pair(m_chassis->GetBody(), m_chassis->GetBody()->GetIndex()));

    // wheels
    for (int i = 0; i < 6; i++) {
        body_indices.push_back(std::make_pair(m_wheels[i]->GetBody(), m_wheels[i]->GetBody()->GetIndex()));
    }

    // Rocker
    for (auto& rocker : m_rockers) {
        body_indices.push_back(std::make_pair(rocker->GetBody(), rocker->GetBody()->GetIndex()));
    }

    // Bogie
    for (auto& bogie : m_bogies) {
        body_indices.push_back(std::make_pair(bogie->GetBody(), bogie->GetBody()->GetIndex()));
    }

    // rocker_uprights
    for (auto& rocker_upright : m_rocker_uprights) {
        body_indices.push_back(std::make_pair(rocker_upright->GetBody(), rocker_upright->GetBody()->GetIndex()));
    }

    // bogie_uprights
    for (auto& bogie_upright : m_bogie_uprights) {
        body_indices.push_back(std::make_pair(bogie_upright->GetBody(), bogie_upright->GetBody()->GetIndex()));
    }

    return body_indices;
}

// =============================================================================

CuriosityDriver::CuriosityDriver() : drive_speeds({0, 0, 0, 0, 0, 0}), steer_angles({0, 0, 0, 0}), curiosity(nullptr) {}

void CuriosityDriver::SetSteering(double angle) {
    steer_angles = {angle, angle, angle, angle};
}

void CuriosityDriver::SetSteering(double angle, CuriosityWheelID id) {
    if (id == CuriosityWheelID::C_LM || id == CuriosityWheelID::C_RM)
        return;
    steer_angles[id] = angle;
}

// -----------------------------------------------------------------------------

CuriosityDCMotorControl::CuriosityDCMotorControl()
    : m_stall_torque({300, 300, 300, 300, 300, 300}), m_no_load_speed({CH_PI, CH_PI, CH_PI, CH_PI, CH_PI, CH_PI}) {}

void CuriosityDCMotorControl::Update(double time) {
    double speed_reading;
    double target_torque;
    for (int i = 0; i < 6; i++) {
        speed_reading = -curiosity->m_drive_shafts[i]->GetPosDt();

        if (speed_reading > m_no_load_speed[i]) {
            target_torque = 0;
        } else if (speed_reading < 0) {
            target_torque = m_stall_torque[i];
        } else {
            target_torque = m_stall_torque[i] * ((m_no_load_speed[i] - speed_reading) / m_no_load_speed[i]);
        }
        curiosity->m_drive_shafts[i]->SetAppliedLoad(-target_torque);
    }
}

CuriositySpeedDriver::CuriositySpeedDriver(double time_ramp, double speed) : m_ramp(time_ramp), m_speed(speed) {}

void CuriositySpeedDriver::Update(double time) {
    double speed = m_speed;
    if (time < m_ramp)
        speed = m_speed * (time / m_ramp);
    drive_speeds = {speed, speed, speed, speed, speed, speed};
}

}  // namespace curiosity
}  // namespace chrono
