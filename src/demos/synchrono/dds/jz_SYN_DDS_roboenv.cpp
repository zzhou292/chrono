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
// Demo to show RoboEnv
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynRoboEnvironmentAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono/physics/ChInertiaUtils.h"

#include "demos/SetChronoSolver.h"

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
using namespace chrono::synchrono;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Simulation step sizes
double step_size = 2e-3;

// How often SynChrono state messages are interchanged
double heartbeat = step_size;  // 1000[Hz]

// -----------------------------------------------------------------------------

void AddCommandLineOptions(ChCLI& cli);

int main(int argc, char* argv[]) {
    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, false, false))
        return 0;

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    heartbeat = cli.GetAsType<double>("heartbeat");

    const int node_id = cli.GetAsType<int>("node_id");
    const int num_nodes = cli.GetAsType<int>("num_nodes");

    std::cout << "node_id: " << node_id << std::endl;
    std::cout << "num_nodes: " << num_nodes << std::endl;

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

    std::vector<ChVector3d> rock_positions = {ChVector3d(3.0, 0, 0.6), ChVector3d(4.0, 4.0, 0.6)};
    std::vector<std::shared_ptr<ChBody>> rock_bodies;

    for (int i = 0; i < rock_positions.size(); i++) {
        // Load the rock mesh
        std::string rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        auto rock_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_obj_path, false, true);
        rock_mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1.0));  // No scaling

        // Compute mass properties
        double mass;
        ChVector3d cog;
        ChMatrix33<> inertia;
        double density = 500;  // Example density
        rock_mesh->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

        // Create the rock body
        auto rock_body = chrono_types::make_shared<ChBodyAuxRef>();
        rock_body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
        rock_body->SetMass(mass * density);
        rock_body->SetInertiaXX(density * principal_I);
        rock_body->SetFrameRefToAbs(ChFrame<>(rock_positions[i], QUNIT));
        sys.Add(rock_body);
        rock_bodies.push_back(rock_body);
        // Add collision shape
        auto rock_mat = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());
        auto rock_ct_shape =
            chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rock_mat, rock_mesh, false, false, 0.005);
        rock_body->AddCollisionShape(rock_ct_shape);
        rock_body->EnableCollision(true);

        // Add visual shape
        auto rock_visual_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock_visual_shape->SetMesh(rock_mesh);
        rock_visual_shape->SetBackfaceCull(true);
        rock_body->AddVisualShape(rock_visual_shape);
    }

    std::vector<std::pair<std::shared_ptr<ChBody>, std::pair<std::string, SynTransform>>> added_bodies = {
        {rock_bodies[0], {GetChronoDataFile("robot/curiosity/rocks/rock1.obj"), ChFrame<>(ChVector3d(0, 0, 0), QUNIT)}},
        {rock_bodies[1],
         {GetChronoDataFile("robot/curiosity/rocks/rock1.obj"), ChFrame<>(ChVector3d(0, 0, 0), QUNIT)}}};

    auto agent = chrono_types::make_shared<SynRoboEnvironmentAgent>(added_bodies, &sys);
    syn_manager.AddAgent(agent);
    syn_manager.Initialize(&sys);

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
            vis_irr->SetWindowTitle("Robo env on Rigid Terrain");
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
            vis_vsg->SetWindowTitle("Robo env on Rigid Terrain");
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    SetChronoSolver(sys, ChSolver::Type::PSOR, ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    size_t step_count = 0;
    // Simulation loop
    while (syn_manager.IsOk()) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
#endif

        // vis->Run();

        // Set current steering angle
        double time = sys.GetChTime();

        // Update RoboEnv controls
        syn_manager.Synchronize(time);  // Synchronize between nodes

        sys.DoStepDynamics(step_size);
        // syn_manager.PrintStepStatistics(std::cout);

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

    syn_manager.QuitSimulation();
    return 0;
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat", std::to_string(heartbeat));

    // DDS Specific
    cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
    cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "3");
}
