// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Basic demonstration of multiple wheeled vehicles in a single simulation using
// the SynChrono wrapper
//
// =============================================================================

#include <chrono>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"
#include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
#include "chrono_synchrono/utils/SynLog.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::synchrono;
using namespace chrono::vehicle;

// =============================================================================

// Initial vehicle location and orientation
ChVector3d initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType track_shoe_vis_type = VisualizationType::PRIMITIVES;
VisualizationType sprocket_vis_type = VisualizationType::PRIMITIVES;
VisualizationType idler_vis_type = VisualizationType::PRIMITIVES;
VisualizationType road_wheel_vis_type = VisualizationType::PRIMITIVES;
VisualizationType road_wheel_assembly_vis_type = VisualizationType::PRIMITIVES;

// Type of vehicle
enum VehicleType { M113 };

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;

// Simulation step sizes
double step_size = 5e-4;

// Simulation end time
double end_time = 1000;

// How often SynChrono state messages are interchanged
double heartbeat = 1e-2;  // 100[Hz]

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// =============================================================================

// Forward declares for straight forward helper functions
void LogCopyright(bool show);
void AddCommandLineOptions(ChCLI& cli);
void GetVehicleModelFiles(VehicleType type,
                          std::string& vehicle,
                          std::string& engine,
                          std::string& transmission,
                          std::string& zombie,
                          std::string& chassis_col,
                          double& cam_distance);

class IrrAppWrapper {
  public:
    IrrAppWrapper(std::shared_ptr<ChTrackedVehicleVisualSystemIrrlicht> app = nullptr) : m_app(app) {}

    void Synchronize(double time, const DriverInputs& driver_inputs) {
        if (m_app)
            m_app->Synchronize(time, driver_inputs);
    }

    void Advance(double step) {
        if (m_app)
            m_app->Advance(step);
    }

    void Render() {
        if (m_app) {
            m_app->BeginScene();
            m_app->Render();
            m_app->EndScene();
        }
    }

    void Set(std::shared_ptr<ChTrackedVehicleVisualSystemIrrlicht> app) { m_app = app; }
    bool IsOk() { return m_app ? m_app->GetDevice()->run() : true; }

    std::shared_ptr<ChTrackedVehicleVisualSystemIrrlicht> m_app;
};

class DriverWrapper : public ChDriver {
  public:
    DriverWrapper(ChVehicle& vehicle) : ChDriver(vehicle) {}

    /// Update the state of this driver system at the specified time.
    virtual void Synchronize(double time) override {
        if (irr_driver) {
            irr_driver->Synchronize(time);
            m_throttle = irr_driver->GetThrottle();
            m_steering = irr_driver->GetSteering();
            m_braking = irr_driver->GetBraking();
        }
    }

    /// Advance the state of this driver system by the specified time step.
    virtual void Advance(double step) override {
        if (irr_driver)
            irr_driver->Advance(step);
    }

    void Set(std::shared_ptr<ChInteractiveDriverIRR> irr_driver) { this->irr_driver = irr_driver; }

    std::shared_ptr<ChInteractiveDriverIRR> irr_driver;
};

// =============================================================================

int main(int argc, char* argv[]) {
    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------

    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, false, false))
        return 0;

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
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
    auto communicator = chrono_types::make_shared<SynDDSCommunicator>(node_id);
    SynChronoManager syn_manager(node_id, num_nodes, communicator);

    // Change SynChronoManager settings
    syn_manager.SetHeartbeat(heartbeat);

    // Copyright
    LogCopyright(node_id == 1);

    // --------------
    // Create systems
    // --------------

    // Adjust position of each vehicle so they aren't on top of each other
    initLoc.y() = node_id * 3.0;

    // Get the vehicle JSON filenames
    double cam_distance;
    std::string vehicle_file, engine_file, transmission_file, zombie_file, chassis_col_filename;
    GetVehicleModelFiles((VehicleType)cli.GetAsType<int>("vehicle"), vehicle_file, engine_file, transmission_file,
                         zombie_file, chassis_col_filename, cam_distance);

    // Create the vehicle, set parameters, and initialize
    TrackedVehicle vehicle(vehicle_file, contact_method);
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetTrackShoeVisualizationType(track_shoe_vis_type);
    vehicle.SetSprocketVisualizationType(sprocket_vis_type);
    vehicle.SetIdlerVisualizationType(idler_vis_type);
    vehicle.SetIdlerWheelVisualizationType(idler_vis_type);
    vehicle.SetRoadWheelVisualizationType(road_wheel_vis_type);
    vehicle.SetSuspensionVisualizationType(road_wheel_assembly_vis_type);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(engine_file);
    auto transmission = ReadTransmissionJSON(transmission_file);
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Set associated collision detection system
    vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Add a collision shape to the chassis
    auto chassis_mat = ChContactMaterial::DefaultMaterial(vehicle.GetSystem()->GetContactMethod());
    auto collision_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(chassis_col_filename, true, true);
    auto chassis_mesh_shape =
        chrono_types::make_shared<ChCollisionShapeTriangleMesh>(chassis_mat, collision_mesh, false, true, 0.001);
    vehicle.GetChassis()->GetBody()->AddCollisionShape(chassis_mesh_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    vehicle.GetChassis()->GetBody()->EnableCollision(true);
    vehicle.GetChassis()->GetBody()->GetCollisionModel()->SetFamily(WheeledCollisionFamily::CHASSIS);

    vehicle.SetChassisCollide(true);

    // Add vehicle as an agent and initialize SynChronoManager
    auto agent = chrono_types::make_shared<SynTrackedVehicleAgent>(&vehicle, zombie_file);
    syn_manager.AddAgent(agent);
    syn_manager.Initialize(vehicle.GetSystem());

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile("terrain/RigidPlane.json"));

    // Create the vehicle Irrlicht interface
    IrrAppWrapper app;
    DriverWrapper driver(vehicle);
    if (cli.HasValueInVector<int>("irr", node_id)) {
        auto temp_app = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
        temp_app->SetWindowTitle("SynChrono Wheeled Vehicle Demo");
        temp_app->SetChaseCamera(trackPoint, cam_distance, 0.5);
        temp_app->Initialize();
        temp_app->AddTypicalLights();
        temp_app->AttachVehicle(&vehicle);

        // Create the interactive driver system
        auto irr_driver = chrono_types::make_shared<ChInteractiveDriverIRR>(*temp_app);

        // Set the time response for steering and throttle keyboard inputs.
        double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
        double throttle_time = 1.0;  // time to go from 0 to +1
        double braking_time = 0.3;   // time to go from 0 to +1
        irr_driver->SetSteeringDelta(render_step_size / steering_time);
        irr_driver->SetThrottleDelta(render_step_size / throttle_time);
        irr_driver->SetBrakingDelta(render_step_size / braking_time);

        irr_driver->Initialize();

        app.Set(temp_app);
        driver.Set(irr_driver);
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    while (app.IsOk() && syn_manager.IsOk()) {
        double time = vehicle.GetSystem()->GetChTime();

        // End simulation
        if (time >= end_time)
            break;

        // Render scene
        if (step_number % render_steps == 0)
            app.Render();

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        syn_manager.Synchronize(time);  // Synchronize between nodes
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        app.Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        // Log clock time
        if (step_number % 100 == 0 && node_id == 1) {
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            SynLog() << (time_span.count() / 1e3) / time << "\n";

            std::cout << "Time: " << vehicle.GetSystem()->GetChTime()
                      << " NumContacts: " << vehicle.GetSystem()->GetNumContacts() << std::endl;
        }
    }
    syn_manager.QuitSimulation();

    return 0;
}

void LogCopyright(bool show) {
    if (!show)
        return;

    SynLog() << "Copyright (c) 2020 projectchrono.org\n";
    SynLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "e,end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat", std::to_string(heartbeat));

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "i,irr", "Nodes for irrlicht usage", "-1");

    // Other options
    cli.AddOption<int>("Demo", "v,vehicle", "Vehicle Options [0]: M113", "0");

    // DDS Specific
    cli.AddOption<int>("DDS", "d,node_id", "ID for this Node", "1");
    cli.AddOption<int>("DDS", "n,num_nodes", "Number of Nodes", "2");
}

void GetVehicleModelFiles(VehicleType type,
                          std::string& vehicle,
                          std::string& engine,
                          std::string& transmission,
                          std::string& zombie,
                          std::string& chassis_col_filename,
                          double& cam_distance) {
    switch (type) {
        case VehicleType::M113:
            vehicle = vehicle::GetDataFile("M113/vehicle/M113_Vehicle_SinglePin.json");
            engine = vehicle::GetDataFile("M113/powertrain/M113_EngineSimple.json");
            transmission = vehicle::GetDataFile("M113/powertrain/M113_AutomaticTransmissionSimpleMap.json");
            zombie = synchrono::GetDataFile("vehicle/col/M113.json");
            chassis_col_filename = vehicle::GetDataFile("M113/meshes/Chassis_Hulls.obj");
            cam_distance = 8.0;
            break;
    }
}