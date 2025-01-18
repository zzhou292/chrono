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
// Demo to show Curiosity Rover operated on Rigid Terrain
// This Demo includes operation to spawn a Curiosity rover, control wheel speed, and
// control rover steering
//
// =============================================================================

#include "chrono_models/robot/curiosity/Curiosity.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynRobotAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/collision/ChCollisionSystemSynchrono.h"
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
using namespace chrono::curiosity;
using namespace chrono::synchrono;

// -----------------------------------------------------------------------------
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Define Curiosity rover chassis type
CuriosityChassisType chassis_type = CuriosityChassisType::FullRover;

// Define Curiosity rover wheel type
CuriosityWheelType wheel_type = CuriosityWheelType::SimpleWheel;

// Simulation step sizes
double step_size = 1e-3;

// How often SynChrono state messages are interchanged
double heartbeat = 1e-3;  // 100[Hz]

// -----------------------------------------------------------------------------

void AddCommandLineOptions(ChCLI& cli);

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, false, false))
        return 0;

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    heartbeat = cli.GetAsType<double>("heartbeat");

    const int node_id = cli.GetAsType<int>("node_id");
    const int num_nodes = cli.GetAsType<int>("num_nodes");

    // Print help, if necessary
    if (cli.CheckHelp() && node_id == 1) {
        cli.Help();
        return 0;
    }
    // -----------------------
    // Create SynChronoManager
    // -----------------------
    auto communicator = chrono_types::make_shared<SynDDSCommunicator>(node_id, TransportType::SHARED_MEMORY);
    SynChronoManager syn_manager(node_id, num_nodes, communicator);

    // Change SynChronoManager settings
    syn_manager.SetHeartbeat(heartbeat);

    // Create the Chrono system with gravity in the negative Z direction
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Create the ground.
    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
    ground->SetPos(ChVector3d(0, 0, -0.5));
    ground->SetFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
    sys.Add(ground);

    // Construct a Curiosity rover and the asociated driver
    auto driver = chrono_types::make_shared<CuriosityDCMotorControl>();

    Curiosity rover(&sys, chassis_type, wheel_type);
    rover.SetDriver(driver);

    ChVector3d pos;
    ChQuaternion<> rot = QUNIT;

    if (node_id == 0) {
        pos = ChVector3d(0.0, 0.0, 0.5);
        rot = QUNIT;
    } else if (node_id == 1) {
        pos = ChVector3d(0.0, 4.0, 0.5);
        rot = QUNIT;
    } else if (node_id == 2) {
        pos = ChVector3d(0.0, 8.0, 0.5);
        rot = QUNIT;
    } else if (node_id == 3) {
        pos = ChVector3d(0.0, 12.0, 0.5);
        rot = QUNIT;
    }

    rover.Initialize(ChFrame<>(pos, rot));

    auto agent = chrono_types::make_shared<SynRobotAgent>(&rover);
    syn_manager.AddAgent(agent);

    // Create and initialize the custom SynChrono collision system
    auto collision_system = chrono_types::make_shared<ChCollisionSystemSynchrono>(agent, node_id);
    sys.SetCollisionSystem(collision_system);

    // sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    syn_manager.Initialize(rover.GetSystem());

    std::cout << "Curiosity total mass: " << rover.GetRoverMass() << std::endl;

    std::cout << std::endl;

    std::shared_ptr<ChVisualSystem> vis;
    if (node_id == 0) {
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
    }

    // Simulation loop
    while (syn_manager.IsOk()) {
        if (node_id == 0) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
#endif
        }

        if (node_id == 0) {
            vis->Run();
        }

        // Set current steering angle
        double time = rover.GetSystem()->GetChTime();
        double max_steering = CH_PI / 6;
        double steering = 0;
        if (time > 2 && time < 7)
            steering = max_steering * (time - 2) / 5;
        else if (time > 7 && time < 12)
            steering = max_steering * (12 - time) / 5;
        driver->SetSteering(steering);

        // Update Curiosity controls
        rover.Update();

        sys.DoStepDynamics(step_size);

        syn_manager.Synchronize(time);  // Synchronize between nodes

        // std::cout << "number of contacts: " << sys.GetContactContainer()->GetNumContacts() << std::endl;
    }

    syn_manager.QuitSimulation();
    return 0;
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat", std::to_string(heartbeat));

    // DDS Specific
    cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
    cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "2");
}
