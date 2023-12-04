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
// SBEL Cobra Experimental Rover Model Class.
// This class contains model for an experimental rover Cobra used in SBEL
//
// =============================================================================
#include <cmath>

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

#include "chrono_models/robot/cobra/Cobra.h"

namespace chrono {
namespace cobra {

// =============================================================================

const double Cobra::m_max_steer_angle = CH_C_PI / 6;

// =============================================================================

// =============================================================================

// Default contact material for rover parts
std::shared_ptr<ChMaterialSurface> DefaultContactMaterial(ChContactMethod contact_method) {
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

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void AddLockJoint(std::shared_ptr<ChBodyAuxRef> body_1,
                  std::shared_ptr<ChBodyAuxRef> body_2,
                  std::shared_ptr<CobraChassis> chassis,
                  const ChVector<>& rel_joint_pos,
                  const ChQuaternion<>& rel_joint_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_joint_pos, rel_joint_rot);

    // auto revo = chrono_types::make_shared<ChLinkLockRevolute>();
    auto revo = chrono_types::make_shared<ChLinkLockLock>();
    revo->Initialize(body_1, body_2, ChCoordsys<>(X_GC.GetCoord().pos, X_GC.GetCoord().rot));
    chassis->GetBody()->GetSystem()->AddLink(revo);
}

// Add a revolute joint between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
void AddRevoluteJoint(std::shared_ptr<ChBody> body1,
                      std::shared_ptr<ChBody> body2,
                      std::shared_ptr<CobraChassis> chassis,
                      const ChVector<>& rel_pos,
                      const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

    // Create joint (DOF about Z axis of X_GC frame)
    auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
    joint->Initialize(body1, body2, ChCoordsys<>(X_GC.GetPos(), X_GC.GetRot()));
    chassis->GetBody()->GetSystem()->AddLink(joint);
}

// Add a universal joint between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
void AddUniversalJoint(std::shared_ptr<ChBody> body1,
                       std::shared_ptr<ChBody> body2,
                       std::shared_ptr<CobraChassis> chassis,
                       const ChVector<>& rel_pos,
                       const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

    // Create joint (DOFs about X and Y axes of X_GC frame)
    auto joint = chrono_types::make_shared<ChLinkUniversal>();
    joint->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(joint);
}

// Add a rotational speed motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotorSpeed(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<CobraChassis> chassis,
                                                        const ChVector<>& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

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
                                                        std::shared_ptr<CobraChassis> chassis,
                                                        const ChVector<>& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

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
                                                          std::shared_ptr<CobraChassis> chassis,
                                                          const ChVector<>& rel_pos,
                                                          const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a spring between two bodies connected at the specified points
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkTSDA> AddSuspensionSpring(std::shared_ptr<ChBodyAuxRef> body1,
                                                std::shared_ptr<ChBodyAuxRef> body2,
                                                std::shared_ptr<CobraChassis> chassis,
                                                const ChVector<>& pos1,
                                                const ChVector<>& pos2) {
    const ChFrame<>& X_GP = chassis->GetBody()->GetFrame_REF_to_abs();
    auto p1 = X_GP.TransformPointLocalToParent(pos1);
    auto p2 = X_GP.TransformPointLocalToParent(pos2);

    std::shared_ptr<ChLinkTSDA> spring;
    spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body1, body2, false, p1, p2);
    spring->SetSpringCoefficient(800000.0);
    spring->SetDampingCoefficient(10000.0);
    chassis->GetBody()->GetSystem()->AddLink(spring);
    return spring;
}

// =============================================================================

// Base class for all Cobra Part
CobraPart::CobraPart(const std::string& name,
                     const ChFrame<>& rel_pos,
                     std::shared_ptr<ChMaterialSurface> mat,
                     bool collide)
    : m_name(name), m_pos(rel_pos), m_mat(mat), m_collide(collide), m_visualize(true) {}

void CobraPart::Construct(ChSystem* system) {
    m_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_body->SetNameString(m_name + "_body");
    m_body->SetMass(m_mass);
    m_body->SetInertiaXX(m_inertia);
    m_body->SetFrame_COG_to_REF(m_cog);

    // Add visualization shape
    if (m_visualize && m_mesh_name != "cobra_wheel_cyl") {
        auto vis_mesh_file = GetChronoDataFile("robot/cobra/obj/" + m_mesh_name + ".obj");
        auto trimesh_vis = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
        trimesh_vis->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
        trimesh_vis->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh_vis);
        trimesh_shape->SetName(m_mesh_name);
        trimesh_shape->SetMutable(false);
        trimesh_shape->SetColor(m_color);
        m_body->AddVisualShape(trimesh_shape);
    } else if (m_visualize && m_mesh_name == "cobra_wheel_cyl") {
        auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.1125, 2 * 0.025);
        m_body->AddVisualShape(cyl, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));
    }

    // Add collision shape
    auto col_mesh_file = GetChronoDataFile("robot/cobra/obj/" + m_mesh_name + ".obj");
    auto trimesh_col = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(col_mesh_file, false, false);
    trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
    trimesh_col->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

    auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_mat, trimesh_col, false, false, 0.005);
    m_body->AddCollisionShape(shape);
    m_body->SetCollide(m_collide);

    system->AddBody(m_body);
}

void CobraPart::CalcMassProperties(double density) {
    auto mesh_filename = GetChronoDataFile("robot/cobra/obj/" + m_mesh_name + ".obj");
    if (m_mesh_name == "cobra_wheel_cyl") {
        mesh_filename = GetChronoDataFile("robot/cobra/obj/cobra_wheel.obj");
    }
    auto trimesh_col = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_filename, true, false);
    trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
    trimesh_col->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

    double vol;
    ChVector<> cog_pos;
    ChMatrix33<> cog_rot;
    ChMatrix33<> inertia;
    trimesh_col->ComputeMassProperties(true, vol, cog_pos, inertia);
    ChInertiaUtils::PrincipalInertia(inertia, m_inertia, cog_rot);
    m_mass = density * vol;
    m_inertia *= density;
    m_cog = ChFrame<>(cog_pos, cog_rot);
}

void CobraPart::Initialize(std::shared_ptr<ChBodyAuxRef> chassis) {
    Construct(chassis->GetSystem());

    // Set absolute position
    ChFrame<> X_GC = chassis->GetFrame_REF_to_abs() * m_pos;
    m_body->SetFrame_REF_to_abs(X_GC);
}

// =============================================================================

// Rover Chassis
CobraChassis::CobraChassis(const std::string& name, std::shared_ptr<ChMaterialSurface> mat)
    : CobraPart(name, ChFrame<>(VNULL, QUNIT), mat, false) {
    m_mesh_name = "cobra_chassis";
    m_color = ChColor(0.7f, 0.4f, 0.4f);
    CalcMassProperties(1650);
}

void CobraChassis::Initialize(ChSystem* system, const ChFrame<>& pos) {
    Construct(system);

    m_body->SetFrame_REF_to_abs(pos);
}

// =============================================================================

// Rover Steering Hub
CobraSteerHub::CobraSteerHub(const std::string& name, const ChFrame<>& rel_pos, std::shared_ptr<ChMaterialSurface> mat)
    : CobraPart(name, rel_pos, mat, false) {
    m_mesh_name = "steering_hub";
    m_color = ChColor(0.7f, 0.7f, 0.9f);
    CalcMassProperties(300);
}

// =============================================================================

// Rover Steering Struct
CobraSteerStruct::CobraSteerStruct(const std::string& name,
                                   const ChFrame<>& rel_pos,
                                   std::shared_ptr<ChMaterialSurface> mat)
    : CobraPart(name, rel_pos, mat, false) {
    m_mesh_name = "steering_struct";
    m_color = ChColor(0.7f, 0.4f, 0.4f);
    CalcMassProperties(300);
}

// =============================================================================

// Cobra Wheel
CobraWheel::CobraWheel(const std::string& name,
                       const ChFrame<>& rel_pos,
                       std::shared_ptr<ChMaterialSurface> mat,
                       CobraWheelType wheel_type)
    : CobraPart(name, rel_pos, mat, true) {
    switch (wheel_type) {
        case CobraWheelType::RealWheel:
            m_mesh_name = "cobra_wheel";
            m_wheel_type = CobraWheelType::RealWheel;
            break;
        case CobraWheelType::SimpleWheel:
            m_mesh_name = "cobra_simplewheel";
            m_wheel_type = CobraWheelType::SimpleWheel;
            break;
    }

    m_color = ChColor(0.4f, 0.7f, 0.4f);
    CalcMassProperties(1200);
}

// =============================================================================

// Rover model
Cobra::Cobra(ChSystem* system, CobraWheelType wheel_type) : m_system(system), m_chassis_fixed(false) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = m_system->GetContactMethod();

    // Create the contact materials
    m_default_material = DefaultContactMaterial(contact_method);
    m_wheel_material = DefaultContactMaterial(contact_method);

    Create(wheel_type);
}

void Cobra::Create(CobraWheelType wheel_type) {
    // create rover chassis
    m_chassis = chrono_types::make_shared<CobraChassis>("chassis", m_default_material);

    // initilize rover steer hubs
    double shx = 0.1968;
    double shy = 0.12644;
    double shz = 0.1196;

    m_steerhubs[CO_LF] = chrono_types::make_shared<CobraSteerHub>(
        "SteerHub_LF", ChFrame<>(ChVector<>(+shx, +shy, shz), QUNIT), m_default_material);
    m_steerhubs[CO_RF] = chrono_types::make_shared<CobraSteerHub>(
        "SteerHub_RF", ChFrame<>(ChVector<>(+shx, -shy, shz), QUNIT), m_default_material);
    m_steerhubs[CO_LB] = chrono_types::make_shared<CobraSteerHub>(
        "SteerHub_LB", ChFrame<>(ChVector<>(-shx, +shy, shz), QUNIT), m_default_material);
    m_steerhubs[CO_RB] = chrono_types::make_shared<CobraSteerHub>(
        "SteerHub_RB", ChFrame<>(ChVector<>(-shx, -shy, shz), QUNIT), m_default_material);

    m_steerhubs[CO_RF]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));
    m_steerhubs[CO_RB]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));

    // initialize rover steering struct
    double ssx = 0.1968;
    double ssy = 0.26341;
    double ssz = 0.0355;

    m_steerstructs[CO_LF] = chrono_types::make_shared<CobraSteerStruct>(
        "SteerStruct_LF", ChFrame<>(ChVector<>(+ssx, +ssy, ssz), QUNIT), m_default_material);
    m_steerstructs[CO_RF] = chrono_types::make_shared<CobraSteerStruct>(
        "SteerStruct_RF", ChFrame<>(ChVector<>(+ssx, -ssy, ssz), QUNIT), m_default_material);
    m_steerstructs[CO_LB] = chrono_types::make_shared<CobraSteerStruct>(
        "SteerStruct_LB", ChFrame<>(ChVector<>(-ssx, +ssy, ssz), QUNIT), m_default_material);
    m_steerstructs[CO_RB] = chrono_types::make_shared<CobraSteerStruct>(
        "SteerStruct_RB", ChFrame<>(ChVector<>(-ssx, -ssy, ssz), QUNIT), m_default_material);

    m_steerstructs[CO_RF]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));
    m_steerstructs[CO_RB]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));

    // initialize rover wheels
    double wx = 0.1968;
    double wy = 0.26341 + 0.031;
    double wz = 0.0355;

    m_wheels[CO_LF] = chrono_types::make_shared<CobraWheel>("Wheel_LF", ChFrame<>(ChVector<>(+wx, +wy, wz), QUNIT),
                                                            m_wheel_material, wheel_type);
    m_wheels[CO_RF] = chrono_types::make_shared<CobraWheel>("Wheel_RF", ChFrame<>(ChVector<>(+wx, -wy, wz), QUNIT),
                                                            m_wheel_material, wheel_type);
    m_wheels[CO_LB] = chrono_types::make_shared<CobraWheel>("Wheel_LB", ChFrame<>(ChVector<>(-wx, +wy, wz), QUNIT),
                                                            m_wheel_material, wheel_type);
    m_wheels[CO_RB] = chrono_types::make_shared<CobraWheel>("Wheel_RB", ChFrame<>(ChVector<>(-wx, -wy, wz), QUNIT),
                                                            m_wheel_material, wheel_type);

    m_wheels[CO_RF]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));
    m_wheels[CO_RB]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));
}

void Cobra::Initialize(const ChFrame<>& pos) {
    assert(m_driver);

    m_chassis->Initialize(m_system, pos);
    m_chassis->GetBody()->SetBodyFixed(m_chassis_fixed);

    for (int i = 0; i < 4; i++) {
        m_steerhubs[i]->Initialize(m_chassis->GetBody());
        m_steerstructs[i]->Initialize(m_chassis->GetBody());
        m_wheels[i]->Initialize(m_chassis->GetBody());
    }

    // rover steer hubs
    double shx = 0.1968;
    double shy = 0.12644;
    double shz = 0.1196;

    // rover steer structs
    double ssx = 0.1968;
    double ssy = 0.26341;
    double ssz = 0.0355;

    // rover steer motors
    double smx = 0.1968;
    double smy = 0.26341;
    double smz = 0.174;

    // rover wheels
    double wx = 0.1968;
    double wy = 0.26341 + 0.0125;
    double wz = 0.0355;

    ChVector<> steer_hub_rel_pos[] = {
        ChVector<>(+shx, +shy, shz),  // LF
        ChVector<>(+shx, -shy, shz),  // RF
        ChVector<>(-shx, +shy, shz),  // LB
        ChVector<>(-shx, -shy, shz)   // RB
    };

    ChVector<> steer_motor_rel_pos[] = {
        ChVector<>(+smx, +smy, smz),  // LF
        ChVector<>(+smx, -smy, smz),  // RF
        ChVector<>(-smx, +smy, smz),  // LB
        ChVector<>(-smx, -smy, smz)   // RB
    };

    ChVector<> drive_motor_rel_pos[] = {
        ChVector<>(+wx, +wy, wz),  // LF
        ChVector<>(+wx, -wy, wz),  // RF
        ChVector<>(-wx, +wy, wz),  // LB
        ChVector<>(-wx, -wy, wz)   // RB
    };

    for (int i = 0; i < 4; i++) {
        AddLockJoint(m_chassis->GetBody(), m_steerhubs[i]->GetBody(), m_chassis, steer_hub_rel_pos[i], QUNIT);
    }

    // initialize steering motors
    for (int i = 0; i < 4; i++) {
        m_steer_motor_funcs[i] = chrono_types::make_shared<ChFunction_Const>(0.0);
        m_steer_motors[i] = AddMotorAngle(m_steerhubs[i]->GetBody(), m_steerstructs[i]->GetBody(), m_chassis,
                                          steer_motor_rel_pos[i], QUNIT);
        m_steer_motors[i]->SetMotorFunction(m_steer_motor_funcs[i]);
    }

    // initialize drive motors
    ChQuaternion<> z2y;
    z2y.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(1, 0, 0));
    for (int i = 0; i < 4; i++) {
        m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunction_Const>(0.5);
        m_drive_motors[i] =
            AddMotorSpeed(m_steerstructs[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis, drive_motor_rel_pos[i], z2y);
        m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
    }
}

void Cobra::SetDriver(std::shared_ptr<CobraDriver> driver) {
    m_driver = driver;
    m_driver->cobra = this;
}

void Cobra::SetWheelContactMaterial(std::shared_ptr<ChMaterialSurface> mat) {
    for (auto& wheel : m_wheels)
        wheel->m_mat = mat;
}

void Cobra::SetChassisFixed(bool fixed) {
    m_chassis_fixed = fixed;
}

ChVector<> Cobra::GetWheelContactForce(CobraWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactForce();
}

ChVector<> Cobra::GetWheelContactTorque(CobraWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactTorque();
}

ChVector<> Cobra::GetWheelAppliedForce(CobraWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedForce();
}

ChVector<> Cobra::GetWheelAppliedTorque(CobraWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedTorque();
}

double Cobra::GetWheelTracTorque(CobraWheelID id) const {
    if (m_driver->GetDriveMotorType() == CobraDriver::DriveMotorType::TORQUE)
        return 0;

    return m_drive_motors[id]->GetMotorTorque();
}

double Cobra::GetRoverMass() const {
    double tot_mass = m_chassis->GetBody()->GetMass();
    for (int i = 0; i < 4; i++) {
        tot_mass += m_wheels[i]->GetBody()->GetMass();
    }
    return tot_mass;
}

double Cobra::GetWheelMass() const {
    return m_wheels[0]->GetBody()->GetMass();
}

void Cobra::Update() {
    double time = m_system->GetChTime();
    m_driver->Update(time);

    for (int i = 0; i < 4; i++) {
        // Extract driver inputs
        double driving = m_driver->drive_speeds[i];
        double steering = m_driver->steer_angles[i];

        // Enforce maximum steering angle
        ChClampValue(steering, -m_max_steer_angle, +m_max_steer_angle);

        // Set motor functions
        m_steer_motor_funcs[i]->Set_yconst(steering);
        if (m_driver->GetDriveMotorType() == CobraDriver::DriveMotorType::SPEED)
            m_drive_motor_funcs[i]->Set_yconst(driving);
    }
}

// =============================================================================

CobraDriver::CobraDriver() : drive_speeds({0, 0, 0, 0}), steer_angles({0, 0, 0, 0}), cobra(nullptr) {}

void CobraDriver::SetSteering(double angle) {
    for (int i = 0; i < 4; i++) {
        if (i == 0 || i == 1) {
            steer_angles[i] = -angle;
        } else {
            steer_angles[i] = angle;
        }
    }
}

void CobraDriver::SetSteering(double angle, CobraWheelID id) {
    steer_angles[id] = angle;
}

CobraSpeedDriver::CobraSpeedDriver(double time_ramp, double speed) : m_ramp(time_ramp), m_speed(speed) {}

/// Set current drive motor speed input.
void CobraSpeedDriver::SetMotorSpeed(double speed) {
    m_speed = speed;
}

void CobraSpeedDriver::Update(double time) {
    double speed = m_speed;
    if (time < m_ramp) {
        speed = m_speed * (time / m_ramp);
    }
    drive_speeds = {speed, speed, speed, speed};
}

}  // namespace cobra
}  // namespace chrono
