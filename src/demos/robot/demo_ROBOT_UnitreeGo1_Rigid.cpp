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
#include "chrono_models/robot/unitree_go1/Unitree_Go1.h"
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
using namespace chrono::unitree_go1;

// -----------------------------------------------------------------------------

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create a Chrono system
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create a "floor" body
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);
    auto floor_box = chrono_types::make_shared<ChVisualShapeBox>(3, 2, 0.1);
    floor_box->SetTexture(GetChronoDataFile("textures/checker2.png"));
    floor->AddVisualShape(floor_box);
    float friction = 0.9f;
    float Y = 1e7f;

    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ground_mat->SetFriction(friction);

    if (sys.GetContactMethod() == ChContactMethod::SMC) {
        std::static_pointer_cast<ChMaterialSurfaceSMC>(ground_mat)->SetYoungModulus(Y);
    }

    floor->SetCollide(true);
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeBox>(ground_mat, 100, 100, 0.1);
    floor->AddCollisionShape(ct_shape);
    sys.AddBody(floor);

    // Create the robot
    Unitree_Go1 robot(&sys);
    robot.Initialize(ChFrame<>(ChVector<>(0, 0, 0.51), QUNIT));

    // Create the visualization window
    std::shared_ptr<ChVisualSystem> vis;
    auto camera_lookat = ChVector<>(0, 0, 0);
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
    double step_size = 5e-4;
    ChRealtimeStepTimer real_timer;

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
