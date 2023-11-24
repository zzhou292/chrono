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

#ifndef COBRA_H
#define COBRA_H

#include <string>
#include <array>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaft.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {

/// Namespace with classes for the Cobra model.
namespace cobra {

/// @addtogroup robot_models_cobra
/// @{

/// Cobra wheel/suspension identifiers.
enum CobraWheelID {
    CO_LF = 0,  ///< left front
    CO_RF = 1,  ///< right front
    CO_LB = 2,  ///< left back
    CO_RB = 3   ///< right back
};

/// Cobra wheel type.
enum class CobraWheelType {
    RealWheel,   ///< actual geometry of the Cobra wheel
    SimpleWheel  ///< simplified geometry of the Cobra wheel
};

// -----------------------------------------------------------------------------

/// Base class definition for all Cobra parts.
/// Cobra Rover Parts include Chassis, Upper Suspension Arm, Bottom Suspension Arm and Wheel.
class CH_MODELS_API CobraPart {
  public:
    CobraPart(const std::string& name,                 ///< part name
              const ChFrame<>& rel_pos,                ///< position relative to chassis frame
              std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
              bool collide                             ///< enable collision?
    );
    virtual ~CobraPart() {}

    /// Return the name of the part.
    const std::string& GetName() const { return m_name; }

    /// Set the name of the part.
    void SetName(const std::string& name) { m_name = name; }

    /// Enable/disable visualization.
    void SetVisualize(bool state) { m_visualize = state; }

    /// Enable/disable collision.
    void SetCollide(bool state) { m_collide = state; }

    /// Initialize the rover part by attaching it to the specified chassis body.
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis);

    /// Return the ChBody of the corresponding Cobra part.
    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }

    /// Return the position of the Cobra part.
    /// This is the absolute location of the part reference frame.
    const ChVector<>& GetPos() const { return m_body->GetFrame_REF_to_abs().GetPos(); }

    /// Return the rotation of the Cobra part.
    /// This is the orientation wrt the global frame of the part reference frame.
    const ChQuaternion<>& GetRot() const { return m_body->GetFrame_REF_to_abs().GetRot(); }

    /// Return the linear velocity of the Cobra part.
    /// This is the absolute linear velocity of the part reference frame.
    const ChVector<>& GetLinVel() const { return m_body->GetFrame_REF_to_abs().GetPos_dt(); }

    /// Return the angular velocity of the Cobra part.
    /// This is the absolute angular velocity of the part reference frame.
    const ChVector<> GetAngVel() const { return m_body->GetFrame_REF_to_abs().GetWvel_par(); }

    /// Return the linear acceleration of the Cobra part.
    /// This is the absolute linear acceleration of the part reference frame.
    const ChVector<>& GetLinAcc() const { return m_body->GetFrame_REF_to_abs().GetPos_dtdt(); }

    /// Return the angular acceleratino of the Cobra part.
    /// This is the absolute angular acceleratin of the part reference frame.
    const ChVector<> GetAngAcc() const { return m_body->GetFrame_REF_to_abs().GetWacc_par(); }

  protected:
    /// Utility function for calculating mass properties using the part's collision mesh.
    void CalcMassProperties(double density);

    /// Construct the part body.
    void Construct(ChSystem* system);

    std::string m_name;                        ///< part name
    std::shared_ptr<ChBodyAuxRef> m_body;      ///< part rigid body
    std::shared_ptr<ChMaterialSurface> m_mat;  ///< contact material

    std::string m_mesh_name;  ///< visualization mesh name
    ChFrame<> m_mesh_xform;   ///< mesh transform (translate, rotate, scale)
    ChColor m_color;          ///< visualization asset color

    ChFrame<> m_pos;       ///< relative position wrt the chassis
    double m_mass;         ///< mass
    ChVector<> m_inertia;  ///< principal moments of inertia
    ChFrame<> m_cog;       ///< COG frame (relative to body frame)

    bool m_visualize;  ///< part visualization flag
    bool m_collide;    ///< part collision flag
};

/// Cobra rover Chassis.
class CH_MODELS_API CobraChassis : public CobraPart {
  public:
    CobraChassis(const std::string& name,                ///< part name
                 std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~CobraChassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize(ChSystem* system, const ChFrame<>& pos);
};

/// Cobra rover Steering Hub.
class CH_MODELS_API CobraSteerHub : public CobraPart {
  public:
    CobraSteerHub(const std::string& name,                ///< part name
                  const ChFrame<>& rel_pos,               ///< position relative to chassis frame
                  std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~CobraSteerHub() {}

    friend class Cobra;
};

/// Cobra rover Steering struct.
class CH_MODELS_API CobraSteerStruct : public CobraPart {
  public:
    CobraSteerStruct(const std::string& name,                ///< part name
                     const ChFrame<>& rel_pos,               ///< position relative to chassis frame
                     std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~CobraSteerStruct() {}

    friend class Cobra;
};

/// Cobra rover Wheel.
class CH_MODELS_API CobraWheel : public CobraPart {
  public:
    CobraWheel(const std::string& name,                 ///< part name
               const ChFrame<>& rel_pos,                ///< position relative to chassis frame
               std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
               CobraWheelType wheel_type                ///< wheel type
    );
    ~CobraWheel() {}

    friend class Cobra;

  private:
    CobraWheelType m_wheel_type;  ///< wheel type
};

class CobraDriver;

/// Cobra rover class.
/// This class encapsulates the location and rotation information of all Cobra parts wrt the chassis.
/// This class should be the entry point to create a complete rover.
class CH_MODELS_API Cobra {
  public:
    Cobra(ChSystem* system, CobraWheelType wheel_type = CobraWheelType::RealWheel);

    ~Cobra() {}

    /// Get the containing system.
    ChSystem* GetSystem() const { return m_system; }

    /// Attach a driver system.
    void SetDriver(std::shared_ptr<CobraDriver> driver);

    /// Set wheel contact material.
    void SetWheelContactMaterial(std::shared_ptr<ChMaterialSurface> mat);

    /// Fix the chassis to ground.
    void SetChassisFixed(bool fixed);

    /// Initialize the Cobra rover at the specified position.
    void Initialize(const ChFrame<>& pos);

    /// Get the rover chassis.
    std::shared_ptr<CobraChassis> GetChassis() const { return m_chassis; }

    /// Get all rover wheels.
    std::array<std::shared_ptr<CobraWheel>, 4> GetWheels() const { return m_wheels; }

    /// Get the specified rover wheel.
    std::shared_ptr<CobraWheel> GetWheel(CobraWheelID id) const { return m_wheels[id]; }

    /// Get the steering hub of the rover
    std::shared_ptr<CobraSteerHub> GetSteerHub(CobraWheelID id) const { return m_steerhubs[id]; }

    /// Get chassis position.
    ChVector<> GetChassisPos() const { return m_chassis->GetPos(); }

    /// Get chassis orientation.
    ChQuaternion<> GetChassisRot() const { return m_chassis->GetRot(); }

    /// Get chassis linear velocity.
    ChVector<> GetChassisVel() const { return m_chassis->GetLinVel(); }

    /// Get chassis linear acceleration.
    ChVector<> GetChassisAcc() const { return m_chassis->GetLinAcc(); }

    /// Get wheel speed.
    ChVector<> GetWheelLinVel(CobraWheelID id) const { return m_wheels[id]->GetLinVel(); }

    /// Get wheel angular velocity.
    ChVector<> GetWheelAngVel(CobraWheelID id) const { return m_wheels[id]->GetAngVel(); }

    /// Get wheel contact force.
    ChVector<> GetWheelContactForce(CobraWheelID id) const;

    /// Get wheel contact torque.
    ChVector<> GetWheelContactTorque(CobraWheelID id) const;

    /// Get wheel total applied force.
    ChVector<> GetWheelAppliedForce(CobraWheelID id) const;

    /// Get wheel tractive torque - if DC control set to off
    double GetWheelTracTorque(CobraWheelID id) const;

    /// Get wheel total applied torque.
    ChVector<> GetWheelAppliedTorque(CobraWheelID id) const;

    /// Get total rover mass.
    double GetRoverMass() const;

    /// Get total wheel mass.
    double GetWheelMass() const;

    /// Get drive motor function.
    /// This will return an empty pointer if the associated driver uses torque control.
    std::shared_ptr<ChFunction_Const> GetDriveMotorFunc(CobraWheelID id) const { return m_drive_motor_funcs[id]; }

    /// Get drive motor.
    /// This will return an empty pointer if the associated driver uses torque control.
    std::shared_ptr<ChLinkMotorRotation> GetDriveMotor(CobraWheelID id) const { return m_drive_motors[id]; }

    /// Cobra update function.
    /// This function must be called before each integration step.
    void Update();

  private:
    /// Create the rover parts.
    void Create(CobraWheelType wheel_type);

    ChSystem* m_system;  ///< pointer to the Chrono system

    bool m_chassis_fixed;  ///< fix chassis to ground

    std::shared_ptr<CobraChassis> m_chassis;                          ///< rover chassis
    std::array<std::shared_ptr<CobraWheel>, 4> m_wheels;              ///< rover wheels (LF, RF, LR, RB)
    std::array<std::shared_ptr<CobraSteerHub>, 4> m_steerhubs;        ///< rover steering hubs (LF, RF, LR, RB)
    std::array<std::shared_ptr<CobraSteerStruct>, 4> m_steerstructs;  ///< rover steering structs (LF, RF, LR, RB)

    std::array<std::shared_ptr<ChLinkMotorRotation>, 4> m_steer_motors;    ///< drive motors
    std::array<std::shared_ptr<ChFunction_Const>, 4> m_steer_motor_funcs;  ///< drive motor functions

    std::array<std::shared_ptr<ChLinkMotorRotation>, 4> m_drive_motors;    ///< drive motors
    std::array<std::shared_ptr<ChFunction_Const>, 4> m_drive_motor_funcs;  ///< drive motor functions

    std::shared_ptr<CobraDriver> m_driver;  ///< rover driver system

    std::shared_ptr<ChMaterialSurface> m_default_material;  ///< common contact material for all non-wheel parts
    std::shared_ptr<ChMaterialSurface> m_wheel_material;    ///< wheel contact material (shared across limbs)

    static const double m_max_steer_angle;  ///< maximum steering angle
};

// -----------------------------------------------------------------------------

/// Base class definition for a Cobra driver.
/// A derived class must implement the Update function to set the various motor controls at the current time.
/// Alternatively, a derived class may directly access the associate Cobra rover and control it through different means
/// (such as applying torques to the wheel driveshafts).
class CH_MODELS_API CobraDriver {
  public:
    /// Type of drive motor control.
    enum class DriveMotorType {
        SPEED,  ///< angular speed
        TORQUE  ///< torque
    };
    virtual ~CobraDriver() {}

    /// Indicate the control type for the drive motors.
    virtual DriveMotorType GetDriveMotorType() const = 0;

    /// Set current steering input (angle: negative for left, positive for right).
    void SetSteering(double angle);

    /// Set current steering input (angle: negative for left turn, positive for right turn).
    /// This function sets the steering angle for the specified wheel.
    void SetSteering(double angle, CobraWheelID id);

  protected:
    CobraDriver();

    virtual void Update(double time) = 0;

    Cobra* cobra;  ///< associated Cobra rover

    std::array<double, 4> drive_speeds;  ///< angular speeds for drive motors
    std::array<double, 4> steer_angles;  ///< angles for steer motors

    friend class Cobra;
};

/// Concrete Cobra speed driver.
/// This driver applies the same angular speed (ramped from 0 to a prescribed value) to all wheels.
class CH_MODELS_API CobraSpeedDriver : public CobraDriver {
  public:
    CobraSpeedDriver(double time_ramp, double speed);
    ~CobraSpeedDriver() {}

    /// Set current drive motor speed input.
    void SetMotorSpeed(double speed);

  private:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }
    virtual void Update(double time) override;

    double m_ramp;
    double m_speed;
};

/// @} robot_models_cobra

}  // namespace cobra
}  // namespace chrono

#endif
