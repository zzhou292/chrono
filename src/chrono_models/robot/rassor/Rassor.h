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
// Authors: Json Zhou
// =============================================================================
//
// NASA RASSOR Mining Experimental Rover Model Class.
// This class contains model for an experimental rover Rassor
// Reference page: https://technology.nasa.gov/patent/KSC-TOPS-7
//
// =============================================================================

#ifndef RASSOR_H
#define RASSOR_H

#include <string>
#include <array>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaft.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {

/// Namespace with classes for the Rassor model.
namespace rassor {

/// @addtogroup robot_models_rassor
/// @{

/// Rassor wheel/suspension identifiers.
enum RassorWheelID {
    RA_LF = 0,  ///< left front
    RA_RF = 1,  ///< right front
    RA_LB = 2,  ///< left back
    RA_RB = 3   ///< right back
};

/// Rassor razor/arm identifiers.
enum RassorDirID {
    RA_F = 0,  ///< front
    RA_B = 1,  ///< back
};

/// Rassor wheel type.
enum class RassorWheelType {
    RealWheel  ///< actual geometry of the Rassor wheel
};

// -----------------------------------------------------------------------------

/// Base class definition for all Rassor parts.
/// Rassor Rover Parts include Chassis, Wheel, Arm, and Razor
class CH_MODELS_API RassorPart {
  public:
    RassorPart(const std::string& name,                 ///< part name
               const ChFrame<>& rel_pos,                ///< position relative to chassis frame
               std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
               bool collide                             ///< enable collision?
    );
    virtual ~RassorPart() {}

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

    /// Return the ChBody of the corresponding Rassor part.
    std::shared_ptr<ChBodyAuxRef> GetBody() const { return m_body; }

    /// Return the position of the Rassor part.
    /// This is the absolute location of the part reference frame.
    const ChVector<>& GetPos() const { return m_body->GetFrame_REF_to_abs().GetPos(); }

    /// Return the rotation of the Rassor part.
    /// This is the orientation wrt the global frame of the part reference frame.
    const ChQuaternion<>& GetRot() const { return m_body->GetFrame_REF_to_abs().GetRot(); }

    /// Return the linear velocity of the Rassor part.
    /// This is the absolute linear velocity of the part reference frame.
    const ChVector<>& GetLinVel() const { return m_body->GetFrame_REF_to_abs().GetPos_dt(); }

    /// Return the angular velocity of the Rassor part.
    /// This is the absolute angular velocity of the part reference frame.
    const ChVector<> GetAngVel() const { return m_body->GetFrame_REF_to_abs().GetWvel_par(); }

    /// Return the linear acceleration of the Rassor part.
    /// This is the absolute linear acceleration of the part reference frame.
    const ChVector<>& GetLinAcc() const { return m_body->GetFrame_REF_to_abs().GetPos_dtdt(); }

    /// Return the angular acceleratino of the Rassor part.
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

/// Rassor rover Chassis.
class CH_MODELS_API RassorChassis : public RassorPart {
  public:
    RassorChassis(const std::string& name,                ///< part name
                  std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~RassorChassis() {}

    /// Initialize the chassis at the specified (absolute) position.
    void Initialize(ChSystem* system, const ChFrame<>& pos);
};

/// Rassor rover Wheel.
class CH_MODELS_API RassorWheel : public RassorPart {
  public:
    RassorWheel(const std::string& name,                 ///< part name
                const ChFrame<>& rel_pos,                ///< position relative to chassis frame
                std::shared_ptr<ChMaterialSurface> mat,  ///< contact material
                RassorWheelType wheel_type               ///< wheel type
    );
    ~RassorWheel() {}

    friend class Rassor;

  private:
    RassorWheelType m_wheel_type;  ///< wheel type
};

/// Rassor rover Razo.
class CH_MODELS_API RassorRazor : public RassorPart {
  public:
    RassorRazor(const std::string& name,                ///< part name
                const ChFrame<>& rel_pos,               ///< position relative to chassis frame
                std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~RassorRazor() {}

    friend class Rassor;
};

/// Rassor rover Arm.
class CH_MODELS_API RassorArm : public RassorPart {
  public:
    RassorArm(const std::string& name,                ///< part name
              const ChFrame<>& rel_pos,               ///< position relative to chassis frame
              std::shared_ptr<ChMaterialSurface> mat  ///< contact material
    );
    ~RassorArm() {}

    friend class Rassor;
};

class RassorDriver;

/// Rassor rover class.
/// This class encapsulates the location and rotation information of all Rassor parts wrt the chassis.
/// This class should be the entry point to create a complete rover.
class CH_MODELS_API Rassor {
  public:
    Rassor(ChSystem* system, RassorWheelType wheel_type = RassorWheelType::RealWheel);

    ~Rassor() {}

    /// Get the containing system.
    ChSystem* GetSystem() const { return m_system; }

    /// Attach a driver system.
    void SetDriver(std::shared_ptr<RassorDriver> driver);

    /// Set wheel contact material.
    void SetWheelContactMaterial(std::shared_ptr<ChMaterialSurface> mat);

    /// Fix the chassis to ground.
    void SetChassisFixed(bool fixed);

    /// Initialize the Rassor rover at the specified position.
    void Initialize(const ChFrame<>& pos);

    /// Get the rover chassis.
    std::shared_ptr<RassorChassis> GetChassis() const { return m_chassis; }

    /// Get all rover wheels.
    std::array<std::shared_ptr<RassorWheel>, 4> GetWheels() const { return m_wheels; }

    /// Get the specified rover wheel.
    std::shared_ptr<RassorWheel> GetWheel(RassorWheelID id) const { return m_wheels[id]; }

    /// Get the specified rover wheel.
    std::shared_ptr<RassorArm> GetArm(RassorDirID id) const { return m_arms[id]; }

    /// Get the specified rover wheel.
    std::shared_ptr<RassorRazor> GetRazor(RassorDirID id) const { return m_razors[id]; }

    /// Get chassis position.
    ChVector<> GetChassisPos() const { return m_chassis->GetPos(); }

    /// Get chassis orientation.
    ChQuaternion<> GetChassisRot() const { return m_chassis->GetRot(); }

    /// Get chassis linear velocity.
    ChVector<> GetChassisVel() const { return m_chassis->GetLinVel(); }

    /// Get chassis linear acceleration.
    ChVector<> GetChassisAcc() const { return m_chassis->GetLinAcc(); }

    /// Get wheel speed.
    ChVector<> GetWheelLinVel(RassorWheelID id) const { return m_wheels[id]->GetLinVel(); }

    /// Get wheel angular velocity.
    ChVector<> GetWheelAngVel(RassorWheelID id) const { return m_wheels[id]->GetAngVel(); }

    /// Get wheel contact force.
    ChVector<> GetWheelContactForce(RassorWheelID id) const;

    /// Get wheel contact torque.
    ChVector<> GetWheelContactTorque(RassorWheelID id) const;

    /// Get wheel total applied force.
    ChVector<> GetWheelAppliedForce(RassorWheelID id) const;

    /// Get wheel tractive torque - if DC control set to off
    double GetWheelTracTorque(RassorWheelID id) const;

    /// Get wheel total applied torque.
    ChVector<> GetWheelAppliedTorque(RassorWheelID id) const;

    double GetDriveMotorRot(RassorWheelID id){return m_drive_motors[id]->GetMotorRot();}

    double GetDriveMotorRot_dt(RassorWheelID id){return m_drive_motors[id]->GetMotorRot_dt();}

    double GetArmMotorRot(RassorWheelID id){return m_arm_1_motors[id]->GetMotorRot();}

    double GetArmMotorRot_dt(RassorWheelID id){return m_arm_1_motors[id]->GetMotorRot_dt();}

    double GetRazorMotorRot(RassorWheelID id){return m_arm_2_motors[id]->GetMotorRot();}

    double GetRazorMotorRot_dt(RassorWheelID id){return m_arm_2_motors[id]->GetMotorRot_dt();}

    /// Get total rover mass.
    double GetRoverMass() const;

    /// Get total wheel mass.
    double GetWheelMass() const;

    /// Get drive motor function.
    /// This will return an empty pointer if the associated driver uses torque control.
    std::shared_ptr<ChFunction_Const> GetDriveMotorFunc(RassorWheelID id) const { return m_drive_motor_funcs[id]; }

    /// Get drive motor.
    /// This will return an empty pointer if the associated driver uses torque control.
    std::shared_ptr<ChLinkMotorRotation> GetDriveMotor(RassorWheelID id) const { return m_drive_motors[id]; }

    /// Rassor update function.
    /// This function must be called before each integration step.
    void Update();

  private:
    /// Create the rover parts.
    void Create(RassorWheelType wheel_type);

    ChSystem* m_system;  ///< pointer to the Chrono system

    bool m_chassis_fixed;  ///< fix chassis to ground

    std::shared_ptr<RassorChassis> m_chassis;              ///< rover chassis
    std::array<std::shared_ptr<RassorWheel>, 4> m_wheels;  ///< rover wheels (LF, RF, LR, RB)
    std::array<std::shared_ptr<RassorRazor>, 2> m_razors;  ///< rover razors (F,B)
    std::array<std::shared_ptr<RassorArm>, 2> m_arms;      ///< rover arms (F,B)

    std::array<std::shared_ptr<ChLinkMotorRotation>, 4> m_drive_motors;    ///< drive motors
    std::array<std::shared_ptr<ChFunction_Const>, 4> m_drive_motor_funcs;  ///< drive motor functions
    std::array<std::shared_ptr<ChLinkMotorRotation>, 2> m_arm_1_motors;    ///< drive motors
    std::array<std::shared_ptr<ChFunction_Const>, 2> m_arm_1_motor_funcs;  ///< drive motor functions
    std::array<std::shared_ptr<ChLinkMotorRotation>, 2> m_arm_2_motors;    ///< drive motors
    std::array<std::shared_ptr<ChFunction_Const>, 2> m_arm_2_motor_funcs;  ///< drive motor functions

    std::shared_ptr<RassorDriver> m_driver;  ///< rover driver system

    std::shared_ptr<ChMaterialSurface> m_default_material;  ///< common contact material for all non-wheel parts
    std::shared_ptr<ChMaterialSurface> m_wheel_material;    ///< wheel contact material (shared across limbs)
};

// -----------------------------------------------------------------------------

/// Base class definition for a Rassor driver.
/// A derived class must implement the Update function to set the various motor controls at the current time.
/// Alternatively, a derived class may directly access the associate Rassor rover and control it through different means
/// (such as applying torques to the wheel driveshafts).
class CH_MODELS_API RassorDriver {
  public:
    /// Type of drive motor control.
    enum class DriveMotorType {
        SPEED,  ///< angular speed
        TORQUE  ///< torque
    };
    virtual ~RassorDriver() {}

    /// Indicate the control type for the drive motors.
    virtual DriveMotorType GetDriveMotorType() const = 0;

  protected:
    RassorDriver();

    virtual void Update(double time) = 0;

    Rassor* rassor;  ///< associated Rassor rover

    std::array<double, 4> drive_speeds;  ///< angular speeds for drive motors
    std::array<double, 2> arm_speeds;    ///< angular speeds for arm motors
    std::array<double, 2> razor_speeds;  ///< angular speeds for razor motors

    friend class Rassor;
};

/// Concrete Rassor speed driver.
/// This driver applies the same angular speed (ramped from 0 to a prescribed value) to all wheels.
/// Note that this speed drive controls three types of motors -> drive, arm, and razor
/// Note that this speed driver control signal is persistent, i.e. it is not reset at each time step.
class CH_MODELS_API RassorSpeedDriver : public RassorDriver {
  public:
    RassorSpeedDriver(double time_ramp);
    ~RassorSpeedDriver() {}

    /// Set current drive motor speed input.
    void SetDriveMotorSpeed(RassorWheelID wheel_id, double drive_speed);

    /// Set current arm motor speed input.
    void SetArmMotorSpeed(RassorDirID dir_id, double arm_speed);

    /// Set current razor motor speed input.
    void SetRazorMotorSpeed(RassorDirID dir_id, double razor_speed);

  private:
    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }
    virtual void Update(double time) override;

    double m_ramp;
    double m_speed;
    double m_arm_speed;
    double m_razor_speed;
};

/// @} robot_models_rassor

}  // namespace rassor
}  // namespace chrono

#endif
