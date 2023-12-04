// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Demo for Fetch Robot
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono_parsers/ChParserURDF.h"
#include "chrono_parsers/ChRobotActuation.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::parsers;

// -----------------------------------------------------------------------------

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create a Chrono system
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.8));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create a "floor" body
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);
    auto floor_box = chrono_types::make_shared<ChVisualShapeBox>(3, 2, 0.1);
    floor_box->SetTexture(GetChronoDataFile("textures/checker2.png"));
    floor->AddVisualShape(floor_box);

    float friction = 0.8f;
    float Y = 1e7f;
    float cr = 0.0f;

    auto ground_mat = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());
    ground_mat->SetFriction(friction);
    ground_mat->SetRestitution(cr);

    if (sys.GetContactMethod() == ChContactMethod::SMC) {
        std::static_pointer_cast<ChMaterialSurfaceSMC>(ground_mat)->SetYoungModulus(Y);
    }

    floor->SetCollide(true);
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeBox>(ground_mat, 100, 100, 0.2);
    floor->AddCollisionShape(ct_shape);
    sys.AddBody(floor);

    // Create parser instance
    ChParserURDF robot(GetChronoDataFile("robot/fetch/robots/fetch.urdf"));

    // Set root body pose
    robot.SetRootInitPose(ChFrame<>(ChVector<>(0, 0, 1.5), QUNIT));

    // Report parsed elements
    robot.PrintModelBodies();

    std::vector<std::string> jt_names = {"head_pan_joint",      "head_tilt_joint",  "shoulder_pan_joint",
                                         "shoulder_lift_joint", "elbow_flex_joint", "wrist_flex_joint"};

    std::vector<std::shared_ptr<ChLinkMotor>> motors(jt_names.size());
    std::vector<std::shared_ptr<ChFunction_Setpoint>> motor_functions(jt_names.size());

    for (int i = 0; i < jt_names.size(); i++) {
        robot.SetJointActuationType(jt_names[i], ChParserURDF::ActuationType::SPEED);
        motors[i] = robot.GetChMotor(jt_names[i]);
        // motor_functions[i] = chrono_types::make_shared<ChFunction_Setpoint>();
        // motors[i]->SetMotorFunction(motor_functions[i]);
    }

    // Create the Chrono model
    robot.PopulateSystem(sys);

    // Fix root body
    robot.GetRootChBody()->SetBodyFixed(true);
    robot.EnableCollisionVisualization();

    // Get selected bodies of the robot
    auto head_pan_link = robot.GetChBody("head_pan_link");
    auto head_tilt_link = robot.GetChBody("head_tilt_link");
    auto shoulder_lift_link = robot.GetChBody("shoulder_lift_link");
    auto shoulder_pan_link = robot.GetChBody("shoulder_pan_link");
    auto torso_lift_link = robot.GetChBody("torso_lift_link");
    auto torso_fixed_link = robot.GetChBody("torso_fixed_link");
    auto wrist_flex_link = robot.GetChBody("wrist_flex_link");

    // Enable collsion and set contact material for selected bodies of the robot
    head_pan_link->SetCollide(false);
    head_tilt_link->SetCollide(false);
    shoulder_lift_link->SetCollide(false);
    shoulder_pan_link->SetCollide(false);
    torso_lift_link->SetCollide(false);
    torso_fixed_link->SetCollide(false);
    wrist_flex_link->SetCollide(false);

    robot.PrintModelJoints();

    // Create the visualization window
    std::shared_ptr<ChVisualSystem> vis;
    auto camera_lookat = robot.GetRootChBody()->GetPos();
    auto camera_loc = camera_lookat + ChVector<>(3, 3, 0);
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(1200, 800);
            vis_irr->SetWindowTitle("RoboSimian URDF demo");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(camera_loc, camera_lookat);
            vis_irr->AddTypicalLights();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetWindowTitle("RoboSimian URDF demo");
            vis_vsg->AddCamera(camera_loc, camera_lookat);
            vis_vsg->SetWindowSize(ChVector2<int>(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2<int>(400, 100));
            vis_vsg->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
            vis_vsg->SetUseSkyBox(false);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(0.5f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->SetWireFrameMode(false);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Solver settings
    sys.SetSolverMaxIterations(200);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Simulation loop
    double step_size = 1e-4;
    ChRealtimeStepTimer real_timer;
    bool terrain_created = false;

    while (vis->Run()) {
        double time = sys.GetChTime();

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Advance system dynamics
        sys.DoStepDynamics(step_size);
        real_timer.Spin(step_size);
    }

    return 0;
}
