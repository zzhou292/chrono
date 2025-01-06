// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// Demo to show Viper Rover operated on Rigid Terrain
// This Demo includes operation to spawn a Viper rover, control wheel speed, and
// control rover steering
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

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
using namespace chrono::viper;

// -----------------------------------------------------------------------------
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Simulation step sizes
double step_size = 2e-3;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create the Chrono system with gravity in the negative Z direction
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create the ground.
    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
    ground->SetPos(ChVector3d(0, 0, -0.5));
    ground->SetFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
    sys.Add(ground);

    // Construct a Viper rover and the asociated driver
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    auto driver2 = chrono_types::make_shared<ViperDCMotorControl>();
    auto driver3 = chrono_types::make_shared<ViperDCMotorControl>();
    Viper viper(&sys, wheel_type);
    viper.SetDriver(driver);

    Viper viper2(&sys, wheel_type);
    viper2.SetDriver(driver2);

    Viper viper3(&sys, wheel_type);
    viper3.SetDriver(driver3);

    ChVector3d pos(0.0, 0.0, 0.5);
    ChQuaternion<> rot = QUNIT;
    ChVector3d pos2(0.0, 4.0, 0.5);
    ChQuaternion<> rot2 = QUNIT;
    ChVector3d pos3(0.0, 8.0, 0.5);
    ChQuaternion<> rot3 = QUNIT;

    viper.Initialize(ChFrame<>(pos, rot));
    viper2.Initialize(ChFrame<>(pos2, rot2));
    viper3.Initialize(ChFrame<>(pos3, rot3));
    std::cout << "Viper total mass: " << viper.GetRoverMass() << std::endl;
    std::cout << "  chassis:        " << viper.GetChassis()->GetBody()->GetMass() << std::endl;
    std::cout << "  upper arm:      " << viper.GetUpperArm(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  lower arm:      " << viper.GetLowerArm(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  upright:        " << viper.GetUpright(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  wheel:          " << viper.GetWheel(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << std::endl;

    std::shared_ptr<ChVisualSystem> vis;
    // Create the run-time visualization interface
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
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Viper Rover on Rigid Terrain");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(3, 3, 1));
            vis_irr->AddTypicalLights();
            vis_irr->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->AddCamera(ChVector3d(3, 3, 1));
            vis_vsg->SetWindowTitle("Viper Rover on Rigid Terrain");
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    size_t step_count = 0;

    // Simulation loop
    while (vis->Run()) {
        // #if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        //         vis->BeginScene();
        //         vis->Render();
        //         vis->EndScene();
        // #endif

        // Set current steering angle
        double time = viper.GetSystem()->GetChTime();
        double max_steering = CH_PI / 6;
        double steering = 0;
        if (time > 2 && time < 7)
            steering = max_steering * (time - 2) / 5;
        else if (time > 7 && time < 12)
            steering = max_steering * (12 - time) / 5;
        driver->SetSteering(steering);
        driver2->SetSteering(steering);
        // Update Viper controls
        viper.Update();
        viper2.Update();
        viper3.Update();
        sys.DoStepDynamics(step_size);

        step_count++;

        if (step_count % 1000 == 0 && step_count > 2000) {
            auto end_time = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> elapsed = end_time - start_time;
            std::cout << "RTF: " << elapsed.count() / (time - 4.0) << std::endl;
        }

        if (step_count == 2000) {
            start_time = std::chrono::high_resolution_clock::now();
        }
    }

    return 0;
}
