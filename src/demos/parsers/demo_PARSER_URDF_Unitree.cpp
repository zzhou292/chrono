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
// Demo for the Unitree Quadruped
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
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

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
    ChParserURDF robot(GetChronoDataFile("robot/unitree-go1/urdf/go1.urdf"));

    // Set root body pose
    robot.SetRootInitPose(ChFrame<>(ChVector<>(0, 0, 0.8), QUNIT));

    // Report parsed elements
    robot.PrintModelBodies();
    robot.PrintModelJoints();

    // Create the Chrono model
    robot.PopulateSystem(sys);

    // Fix root body
    robot.GetRootChBody()->SetBodyFixed(false);

    auto trunk = robot.GetChBody("trunk");
    auto FR_foot = robot.GetChBody("FR_foot");
    auto FL_foot = robot.GetChBody("FL_foot");
    auto RR_foot = robot.GetChBody("RR_foot");
    auto RL_foot = robot.GetChBody("RL_foot");
    auto FR_thigh = robot.GetChBody("FR_thigh");
    auto FL_thigh = robot.GetChBody("FL_thigh");
    auto RR_thigh = robot.GetChBody("RR_thigh");
    auto RL_thigh = robot.GetChBody("RL_thigh");
    auto FR_hip = robot.GetChBody("FR_hip");
    auto FL_hip = robot.GetChBody("FL_hip");
    auto RR_hip = robot.GetChBody("RR_hip");
    auto RL_hip = robot.GetChBody("RL_hip");

    // Enable collsion and set contact material for selected bodies of the robot
    trunk->SetCollide(true);
    FR_foot->SetCollide(true);
    FL_foot->SetCollide(true);
    RR_foot->SetCollide(true);
    RL_foot->SetCollide(true);
    FR_thigh->SetCollide(true);
    FL_thigh->SetCollide(true);
    RR_thigh->SetCollide(true);
    RL_thigh->SetCollide(true);
    FR_hip->SetCollide(true);
    FL_hip->SetCollide(true);
    RR_hip->SetCollide(true);
    RL_hip->SetCollide(true);

    ChContactMaterialData mat;
    mat.mu = 0.8f;
    mat.cr = 0.0f;
    mat.Y = 1e7f;
    auto cmat = mat.CreateMaterial(sys.GetContactMethod());
    trunk->GetCollisionModel()->SetAllShapesMaterial(cmat);
    FR_foot->GetCollisionModel()->SetAllShapesMaterial(cmat);
    FL_foot->GetCollisionModel()->SetAllShapesMaterial(cmat);
    RR_foot->GetCollisionModel()->SetAllShapesMaterial(cmat);
    RL_foot->GetCollisionModel()->SetAllShapesMaterial(cmat);
    FR_thigh->GetCollisionModel()->SetAllShapesMaterial(cmat);
    FL_thigh->GetCollisionModel()->SetAllShapesMaterial(cmat);
    RR_thigh->GetCollisionModel()->SetAllShapesMaterial(cmat);
    RL_thigh->GetCollisionModel()->SetAllShapesMaterial(cmat);
    FR_hip->GetCollisionModel()->SetAllShapesMaterial(cmat);
    FL_hip->GetCollisionModel()->SetAllShapesMaterial(cmat);
    RR_hip->GetCollisionModel()->SetAllShapesMaterial(cmat);
    RL_hip->GetCollisionModel()->SetAllShapesMaterial(cmat);

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
    double step_size = 2e-4;
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
