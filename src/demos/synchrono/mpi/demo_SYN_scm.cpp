// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jay Taves
// =============================================================================
//
// Demo code illustrating synchronization of the SCM semi-empirical model for
// deformable soil
//
// See also in chrono_vehicle:
// - demo_VEH_DeformableSoil
// - demo_VEH_DeformableSoilAndTire
// - demo_VEH_HMMWV_DefSoil
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynSCMTerrainAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#endif

#ifdef CHRONO_SENSOR
    #include "chrono_sensor/ChSensorManager.h"
    #include "chrono_sensor/sensors/ChCameraSensor.h"
    #include "chrono_sensor/filters/ChFilterAccess.h"
    #include "chrono_sensor/filters/ChFilterSave.h"
    #include "chrono_sensor/filters/ChFilterVisualize.h"
using namespace chrono::sensor;
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::synchrono;
using namespace chrono::vehicle::hmmwv;

// Forward declaration
void AddCommandLineOptions(ChCLI& cli);

// =============================================================================

int main(int argc, char* argv[]) {
    // -----------------------
    // Create SynChronoManager
    // -----------------------
    auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
    int node_id = communicator->GetRank();
    int num_nodes = communicator->GetNumRanks();
    SynChronoManager syn_manager(node_id, num_nodes, communicator);

    // Copyright
    if (node_id == 0) {
        SynLog() << "Copyright (c) 2020 projectchrono.org\n";
        SynLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";
    }

    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------

    ChCLI cli(argv[0]);
    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, node_id == 0))
        return 0;

    auto step_size = cli.GetAsType<double>("step_size");
    auto end_time = cli.GetAsType<double>("end_time");
    auto heartbeat = cli.GetAsType<double>("heartbeat");
    auto contact_method =
        (cli.Matches<std::string>("contact_method", "SMC") ? ChContactMethod::SMC : ChContactMethod::NSC);
    auto size_x = cli.GetAsType<double>("sizeX");
    auto size_y = cli.GetAsType<double>("sizeY");
    auto dpu = cli.GetAsType<double>("dpu");
#ifdef CHRONO_SENSOR
    auto cam_x = cli.GetAsType<std::vector<double>>("c_pos")[0];
    auto cam_y = cli.GetAsType<std::vector<double>>("c_pos")[1];
    auto cam_res_width = cli.GetAsType<std::vector<int>>("res")[0];
    auto cam_res_height = cli.GetAsType<std::vector<int>>("res")[1];
#endif
    auto flat_patch = cli.Matches<std::string>("terrain_type", "Flat");
    auto bulldozing = cli.GetAsType<bool>("bulldozing");

    // Change SynChronoManager settings
    syn_manager.SetHeartbeat(heartbeat);

    // ----------------------
    // Vehicle Initialization
    // ----------------------
    // Calculate initial position and paths for each vehicle
    ChVector3d offset(-size_x / 2 + 5, 0, 0);

    ChVector3d initLoc;
    ChQuaternion<> initRot;
    std::vector<ChVector3d> curve_pts;
    if (node_id % 2 == 0) {
        // Start even vehicles in a row on the south side, driving north
        initLoc = offset + ChVector3d(0, 2.0 * (node_id + 1), 0.5);
        initRot = QuatFromAngleZ(0);
        curve_pts = {initLoc, initLoc + ChVector3d(100, 0, 0)};
    } else {
        // Start odd vehicles staggered going up the west edge, driving east
        initLoc = offset + ChVector3d(2.0 * (node_id - 1), -5.0 - 2.0 * (node_id - 1), 0.5);
        initRot = QuatFromAngleZ(CH_PI / 2);
        curve_pts = {initLoc, initLoc + ChVector3d(0, 100, 0)};
    }

    // Create the HMMWV
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(contact_method);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    hmmwv.SetEngineType(EngineModelType::SHAFTS);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetTireType(TireModelType::RIGID);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::MESH);
    hmmwv.SetSteeringVisualizationType(VisualizationType::NONE);
    hmmwv.SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Solver settings.
    hmmwv.GetSystem()->SetNumThreads(std::min(8, ChOMP::GetNumProcs()));
    hmmwv.GetSystem()->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    hmmwv.GetSystem()->GetSolver()->AsIterative()->SetMaxIterations(50);

    // Add vehicle as an agent
    auto vehicle_agent = chrono_types::make_shared<SynWheeledVehicleAgent>(&hmmwv.GetVehicle());
    vehicle_agent->SetZombieVisualizationFiles("hmmwv/hmmwv_chassis.obj", "hmmwv/hmmwv_rim.obj",
                                               "hmmwv/hmmwv_tire_left.obj");
    vehicle_agent->SetNumWheels(4);
    syn_manager.AddAgent(vehicle_agent);

    // ----------------------
    // Terrain specific setup
    // ----------------------
    auto terrain = chrono_types::make_shared<SCMTerrain>(hmmwv.GetSystem());

    // Configure the SCM terrain
    if (bulldozing) {
        terrain->EnableBulldozing(bulldozing);
        terrain->SetBulldozingParameters(
            55,   // angle of friction for erosion of displaced material at the border of the rut
            1,    // displaced material vs downward pressed material.
            5,    // number of erosion refinements per timestep
            10);  // number of concentric vertex selections subject to erosion
    }

    // Only relevant for Irrlicht visualization, gives some nice colors
    terrain->SetPlotType(SCMTerrain::PLOT_SINKAGE, 0, 0.1);
    terrain->SetMeshWireframe(true);

    // The physics do not change when you add a moving patch, you just make it much easier for the SCM
    // implementation to do its job by restricting where it has to look for contacts
    terrain->AddMovingPatch(hmmwv.GetVehicle().GetChassisBody(), ChVector3d(0, 0, 0), ChVector3d(5, 3, 1));

    if (flat_patch) {
        terrain->Initialize(size_x, size_y, 1 / dpu);
    } else {
        terrain->Initialize(vehicle::GetDataFile("terrain/height_maps/slope.bmp"), size_x, size_y, 0.0, 5.0, 1 / dpu);
    }

    // Create an SCMTerrainAgent and add it to the SynChrono manager
    auto terrain_agent = chrono_types::make_shared<SynSCMTerrainAgent>(terrain);
    syn_manager.AddAgent(terrain_agent);

    // Choice of soft parameters is arbitrary
    SCMParameters params;
    params.InitializeParametersAsSoft();
    terrain_agent->SetSoilParametersFromStruct(&params);

    // Add texture for the terrain
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetSpecularColor({.1f, .1f, .1f});
    vis_mat->SetKdTexture(GetChronoDataFile("sensor/textures/grass_texture.jpg"));
    terrain->GetMesh()->AddMaterial(vis_mat);

    // Create the driver for the vehicle

    // What we defined earlier, a straight line
    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "Box path", 10);

    // Reasonable defaults for the underlying PID
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.GetSteeringController().SetGains(0.4, 0.1, 0);
    driver.GetSteeringController().SetLookAheadDistance(5);

    // Initialzie the SynChrono manager
    syn_manager.Initialize(hmmwv.GetSystem());

    // -------------
    // Visualization
    // -------------
#ifdef CHRONO_IRRLICHT
    // Create the vehicle Irrlicht interface
    std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> app;
    if (cli.HasValueInVector<int>("irr", node_id)) {
        app = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        app->SetWindowTitle("SynChrono SCM Demo");
        app->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 16.0, 0.5);
        app->Initialize();
        app->AddTypicalLights();
        app->AttachVehicle(&hmmwv.GetVehicle());
    }

    // Time interval between two render frames (1/FPS)
    double render_step_size = 1.0 / 100;
    // Number of simulation steps between two render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);
#endif

#ifdef CHRONO_SENSOR
    ChSensorManager sensor_manager(hmmwv.GetSystem());
    if (cli.HasValueInVector<int>("sens", node_id)) {
        sensor_manager.scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 6000);
        sensor_manager.scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 6000);

        // Give the camera a fixed place to live
        auto origin = chrono_types::make_shared<ChBody>();
        origin->SetFixed(true);
        hmmwv.GetSystem()->AddBody(origin);

        // Happens to be a reasonable-looking height
        ChVector3d camera_loc(cam_x, cam_y, 25);

        // Rotations to get a nice angle
        ChQuaternion<> rotation = QUNIT;
        const bool USE_ISO_VIEW = true;
        if (USE_ISO_VIEW) {
            ChQuaternion<> qA = QuatFromAngleAxis(35 * CH_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = QuatFromAngleAxis(135 * CH_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
        } else {
            // Top down view
            ChQuaternion<> qA = QuatFromAngleAxis(90 * CH_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = QuatFromAngleAxis(180 * CH_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
        }

        auto overhead_camera = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
            origin,                                         // body camera is attached to
            30.0f,                                          // update rate in Hz
            chrono::ChFrame<double>(camera_loc, rotation),  // offset pose
            cam_res_width,                                  // image width
            cam_res_height,                                 // image height
            (float)CH_PI_3                                // FOV
        );

        overhead_camera->SetName("Overhead Cam");
        overhead_camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

        // Do we draw a window on the screen?
        if (cli.GetAsType<bool>("sens_vis"))
            overhead_camera->PushFilter(chrono_types::make_shared<ChFilterVisualize>(cam_res_width, cam_res_height));

        // Do we save images to disc?
        std::string file_path = std::string("SENSOR_OUTPUT/scm") + std::to_string(node_id) + std::string("/");
        if (cli.GetAsType<bool>("sens_save"))
            overhead_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(file_path));

        sensor_manager.AddSensor(overhead_camera);
    }
#endif

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize simulation frame counters
    int step_number = 0;

    ChRealtimeStepTimer realtime_timer;
    ChTimer timer;
    timer.start();

    while (true) {
        double time = hmmwv.GetSystem()->GetChTime();

        // End simulation
        if (time >= end_time         // ran out of time
            || !syn_manager.IsOk())  // SynChronoManager has shutdown
            break;

#ifdef CHRONO_IRRLICHT
        if (app && !app->GetDevice()->run())  //  Irrlicht visualization has stopped
            break;

        // Render scene
        if (app && step_number % render_steps == 0) {
            app->BeginScene();
            app->Render();
            app->EndScene();
        }
#endif

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        syn_manager.Synchronize(time);  // Synchronize between nodes
        driver.Synchronize(time);
        terrain->Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, *terrain);
#ifdef CHRONO_IRRLICHT
        if (app)
            app->Synchronize(time, driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        hmmwv.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        if (app)
            app->Advance(step_size);
#endif

#ifdef CHRONO_SENSOR
        sensor_manager.Update();
#endif

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        // realtime_timer.Spin(step_size);

        if (step_number % 100 == 0 && node_id == 1) {
            SynLog() << timer.GetTimeSeconds() / time << "\n";
        }
    }
    syn_manager.QuitSimulation();

    return 0;
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", "1e-3");
    cli.AddOption<double>("Simulation", "e,end_time", "End time", "1000");
    cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat", "1e-2");
    cli.AddOption<std::string>("Simulation", "c,contact_method", "Contact Method", "SMC", "NSC/SMC");

    // SCM specific options
    cli.AddOption<double>("Demo", "d,dpu", "Divisions per unit", "20");
    cli.AddOption<std::string>("Demo", "t,terrain_type", "Terrain Type", "Flat", "Flat,Hmap");

    // Many more nodes are impacted with bulldozing and performance is tied to number of deformed nodes, so enabling
    // this can cause a significant slowdown
    cli.AddOption<bool>("Demo", "bulldozing", "Toggle bulldozing effects ON", "false");

    // Visualization is the only reason you should be shy about terrain size. The implementation can easily handle a
    // practically infinite terrain (provided you don't need to visualize it)
    cli.AddOption<double>("Demo", "x,sizeX", "Size in the X", "100");
    cli.AddOption<double>("Demo", "y,sizeY", "Size in the Y", "50");

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "i,irr", "Ranks for irrlicht usage", "-1");

    // Sensor options
    cli.AddOption<std::vector<int>>("Sensor", "sens", "Ranks for sensor usage", "-1");
    cli.AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    cli.AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");

    cli.AddOption<std::vector<int>>("Demo", "r,res", "Camera resolution", "1280,720", "width,height");
    cli.AddOption<std::vector<double>>("Demo", "c_pos", "Camera Position", "-15,-25", "X,Y");
}
