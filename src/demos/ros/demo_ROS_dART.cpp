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
// Authors: Aaron Young
// =============================================================================
//
// Demo to show the use of Chrono::Vehicle with ROS
//
// =============================================================================

#include "chrono/core/ChTypes.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_models/vehicle/artcar/ARTcar.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"

#include <chrono>

using namespace chrono;
// using namespace chrono::ros;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::vehicle::artcar;
// =============================================================================
// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);
// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;
// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;
// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;
// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

//sensor params
unsigned int image_width = 1920;
unsigned int image_height = 1080;
float fov = (float)CH_C_PI / 2.;
int alias_factor = 1;
float lag = 0.0f;
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// Simulation step size
double step_size = 2e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ARTcar vehicle;
    vehicle.SetContactMethod(contact_method);
    vehicle.SetChassisCollisionType(chassis_collision_type);
    vehicle.SetChassisFixed(false);
    vehicle.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    vehicle.SetTireType(tire_model);
    vehicle.SetTireStepSize(step_size);
    vehicle.Initialize();

    VisualizationType tire_vis_type = VisualizationType::MESH;

    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle.SetSteeringVisualizationType(steering_vis_type);
    vehicle.SetWheelVisualizationType(wheel_vis_type);
    vehicle.SetTireVisualizationType(tire_vis_type);
    // Containing system
    auto system = vehicle.GetSystem();

    // Add box in front of the car
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat->SetDiffuseColor({0.0, 1.0, 0.0});
    vis_mat->SetSpecularColor({1.f, 1.f, 1.f});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(.5f);
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(50000);

    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(1.0, 1.0, 1.0, 1000, true, false);
    box_body->SetPos({3, 0, 0});
    box_body->SetBodyFixed(true);
    system->Add(box_body);
    {
        auto shape = box_body->GetVisualModel()->GetShapes()[0].first;
        if(shape->GetNumMaterials() == 0){
            shape->AddMaterial(vis_mat);
        }
        else{
            shape->GetMaterials()[0] = vis_mat;
        }
    }


    // Create the terrain
    RigidTerrain terrain(system, vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // Create the basic driver
    auto driver = std::make_shared<ChDriver>(vehicle.GetVehicle());

    // Create a sensor manager
    auto manager = chrono_types::make_shared<ChSensorManager>(system);
    manager->scene->AddPointLight({100, 100, 100}, {0.4f, 0.4f, 0.4f}, 500);
    // Set the background to an environment map
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);
    manager->SetVerbose(false);

    // Create a lidar and add it to the sensor manager
    auto offset_pose = chrono::ChFrame<double>({-0.1, 0, 0.4}, Q_from_AngAxis(0, {0, 0, 1}));
    auto lidar = chrono_types::make_shared<ChLidarSensor>(vehicle.GetChassis()->GetBody(),  // body lidar is attached to
                                                          10,                             // scanning rate in Hz
                                                          offset_pose,                    // offset pose
                                                          480,                   // number of horizontal samples
                                                          1,                   // number of vertical channels
                                                          (float)(2 * CH_C_PI),  // horizontal field of view
                                                          (float)CH_C_PI / 12, (float)-CH_C_PI / 3,
                                                          140.0f  // vertical field of view
    );
    lidar->SetName("Lidar Sensor 1");
    lidar->SetLag(0.f);
    lidar->SetCollectionWindow(0.02f);
    // Create a filter graph for post-processing the data from the lidar
    // Provides the host access to the Depth,Intensity data
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    // Renders the raw lidar data
    //lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(960, 480, "Raw Lidar Depth Data"));
    // Convert Depth,Intensity data to XYZI point
    // cloud data
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    // Render the point cloud
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(960, 600, 0.25, "Lidar Point Cloud"));
    // Access the lidar data as an XYZI buffer
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    // add lidar and radar to the sensor manager
    manager->AddSensor(lidar);

    // Add camera
    auto cam_pose = chrono::ChFrame<double>({-2.304, 0, 1.0}, Q_from_AngAxis(0.1, {0, 1.25, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(vehicle.GetChassis()->GetBody(),         // body camera is attached to
                                                            20,   // update rate in Hz
                                                            cam_pose,  // offset pose
                                                            image_width,   // image width
                                                            image_height,  // image height
                                                            fov,           // camera's horizontal field of view
                                                            alias_factor,  // supersample factor for antialiasing
                                                            lens_model,    // FOV
                                                            false);        // use global illumination or not
    cam->SetName(" Camera ");
    cam->SetLag(lag);
    cam->SetCollectionWindow(0.0f);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Camera"));
    manager->AddSensor(cam);
    manager->Update();
    // ------------

    // // Create ROS manager
    // auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // // Create a publisher for the simulation clock
    // // The clock automatically publishes on every tick and on topic /clock
    // auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    // ros_manager->RegisterHandler(clock_handler);

    // // Create the publisher for the lidar
    // auto lidar_topic_name = "~/output/lidar/data/pointcloud";
    // auto lidar_handler = chrono_types::make_shared<ChROSLidarHandler>(lidar, lidar_topic_name);
    // ros_manager->RegisterHandler(lidar_handler);

    // // Create a subscriber to the driver inputs
    // auto driver_inputs_rate = 25;
    // auto driver_inputs_topic_name = "~/input/driver_inputs";
    // auto driver_inputs_handler =
    //     chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver, driver_inputs_topic_name);
    // ros_manager->RegisterHandler(driver_inputs_handler);

    // // Create a publisher for the vehicle state
    // auto vehicle_state_rate = 25;
    // auto vehicle_state_topic_name = "~/output/vehicle/state";
    // auto vehicle_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
    //     vehicle_state_rate, vehicle.GetChassisBody(), vehicle_state_topic_name);
    // ros_manager->RegisterHandler(vehicle_state_handler);

    // // Finally, initialize the ros manager
    // ros_manager->Initialize();

    double t_end = 300;
    double time = 0;


    while (time < t_end) {
        // Get driver inputs
        driver->SetThrottle(0.025f);
        DriverInputs driver_inputs = driver->GetInputs();

        // update sensor manager
        manager->Update();
        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);

        // if (!ros_manager->Update(time, step_size))
        //     break;
    }

    return 0;
}
