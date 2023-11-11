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
// Authors: Radu Serban, Asher Elmquist, Json Zhou
// =============================================================================
//
// Main driver function for the ARTcar model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/artcar/ARTcar.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"

using namespace chrono;
using namespace chrono::sensor;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::ros;
using namespace chrono::geometry;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(-2.5, 0.5, 0.3);
ChQuaternion<> initRot(1, 0, 0, 0);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.2);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "ARTcar";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

// Cone data store
float cone_offset_x = 0;
float cone_offset_y = 0;
float cone_spread_x = 1.0;
float cone_spread_y = 1.0;

std::string sensor_data_dir = "sensor_output/";
int num_cones = 100;
bool cones_from_file = true;
std::string cone_path_file = "data/paths/cone_paths_0.csv";

std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> red_cone_assets;
std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> green_cone_assets;
// =============================================================================
// Forward declaration
void AddRandomCones(int count, std::string filename, int class_id, RigidTerrain& terrain, artcar::ARTcar& car);
void AddConesFromFile(RigidTerrain& terrain, artcar::ARTcar& car);
void LabelConeAssets();

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the Sedan vehicle, set parameters, and initialize
    artcar::ARTcar car;
    car.SetContactMethod(contact_method);
    car.SetChassisCollisionType(chassis_collision_type);
    car.SetChassisFixed(false);
    initRot.Q_from_AngZ(1.57);
    car.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    car.SetTireType(tire_model);
    car.SetTireStepSize(tire_step_size);
    car.Initialize();

    VisualizationType tire_vis_type = VisualizationType::MESH;

    car.SetChassisVisualizationType(chassis_vis_type);
    car.SetSuspensionVisualizationType(suspension_vis_type);
    car.SetSteeringVisualizationType(steering_vis_type);
    car.SetWheelVisualizationType(wheel_vis_type);
    car.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(car.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;

    patch = terrain.AddPatch(patch_mat, CSYSNORM, GetChronoDataFile("autonomy-toolkit/me3038/rm3038_pt_cloud.obj"));
    // patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"));

    terrain.Initialize();

    if (cones_from_file) {
        AddConesFromFile(terrain, car);
    } else {
        AddRandomCones(int(num_cones / 2), GetChronoDataFile("sensor/cones/green_cone.obj"), 2, terrain, car);
        AddRandomCones(int(num_cones / 2), GetChronoDataFile("sensor/cones/red_cone.obj"), 1, terrain, car);
    }

    // Camera
    auto manager = chrono_types::make_shared<ChSensorManager>(car.GetSystem());
    manager->scene->AddPointLight(ChVector<>(100, 100, 100), {1, 1, 1}, 5000);

    Background b;
    b.mode = BackgroundMode::GRADIENT;
    manager->scene->SetBackground(b);

    chrono::ChFrame<double> camera_pos({0.204, 0, 0.10018}, Q_from_AngAxis(.2, {0, 1, 0}));

    int width = 1280;
    int height = 720;
    int frame_rate = 30;
    float fov = 1.396;

    // Add FOV camera
    auto cam1 =
        chrono_types::make_shared<ChCameraSensor>(car.GetChassisBody(), frame_rate, camera_pos, width, height, fov, 1);
    cam1->SetName("Camera Sensor");
    cam1->SetCollectionWindow(0);
    cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360));
    // cam1->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_data_dir + "cam1/"));
    cam1->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam1);

    // Add Birdeye Camera
    chrono::ChFrame<double> birdeye_pos({initLoc.x() + 2, initLoc.y(), 10}, Q_from_AngAxis(CH_C_PI_2, {0, 1, 0}));
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(patch->GetGroundBody(), frame_rate, birdeye_pos, width,
                                                          height, fov, 2);
    cam2->SetName("Birdeye Sensor");
    cam2->SetCollectionWindow(0);
    cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360));
    // cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_data_dir + "cam2/"));
    cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam2);

    // Add Segmentation Camera
    auto cam_seg = chrono_types::make_shared<ChSegmentationCamera>(car.GetChassisBody(), frame_rate, camera_pos, width,
                                                                   height, fov);
    cam_seg->SetName("Segmentation Filter");

    cam_seg->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Semantic Segmentation"));
    cam_seg->PushFilter(chrono_types::make_shared<ChFilterSave>(sensor_data_dir + "cam_seg/"));
    cam_seg->PushFilter(chrono_types::make_shared<ChFilterSemanticAccess>());
    manager->AddSensor(cam_seg);

    manager->ReconstructScenes();
    LabelConeAssets();
    manager->ReconstructScenes();

    manager->Update();

    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Create the publisher for the camera
    auto camera_rate = cam1->GetUpdateRate();
    auto camera_topic_name = "/sensing/front_facing_camera/raw";
    auto camera_handler = chrono_types::make_shared<ChROSCameraHandler>(camera_rate, cam1, camera_topic_name);
    ros_manager->RegisterHandler(camera_handler);

    ros_manager->Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("ARTcar Demo");
    vis->SetChaseCamera(trackPoint, 1.5, 0.05);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&car.GetVehicle());

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChInteractiveDriverIRR driver(*vis);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    std::string joystick_config = "/home/jason/Desktop/STUDY/chrono-dep/NSFDemoDataDir/joystick/controller_G27.json";
    driver.SetJoystickConfigFile(joystick_config);

    // If in playback mode, attach the data file to the driver system and
    // force it to playback the driver inputs.
    if (driver_mode == PLAYBACK) {
        driver.SetInputDataFile(driver_file);
        driver.SetInputMode(ChInteractiveDriverIRR::InputMode::DATAFILE);
    }

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        car.LogHardpointLocations();
    }

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << car.GetVehicle().GetMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    if (contact_vis) {
        vis->SetSymbolScale(1e-4);
        vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
    }

    car.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        double time = car.GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(car.GetSystem(), filename);
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            car.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Driver output
        if (driver_mode == RECORD) {
            driver_csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking
                       << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        car.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        car.Advance(step_size);
        vis->Advance(step_size);

        manager->Update();

        if (!ros_manager->Update(time, step_size))
            break;

        // Increment frame number
        step_number++;
    }

    if (driver_mode == RECORD) {
        driver_csv.write_to_file(driver_file);
    }

    return 0;
}

void AddRandomCones(int count, std::string filename, int class_id, RigidTerrain& terrain, artcar::ARTcar& car) {
    auto mmesh = chrono_types::make_shared<chrono::geometry::ChTriangleMeshConnected>();
    mmesh->LoadWavefrontMesh(filename, false, true);
    mmesh->Transform(chrono::ChVector<>(0, 0, 0), chrono::ChMatrix33<>(1));

    for (int i = 0; i < count; ++i) {
        // Generate a random position
        double x = (chrono::ChRandom() - 0.5) * cone_spread_x + cone_offset_x;
        double y = (chrono::ChRandom() - 0.5) * cone_spread_y + cone_offset_y;
        double z = terrain.GetHeight(chrono::ChVector<>(x, y, 1000));  // get the terrain z
        chrono::ChVector<> pos(x, y, z);

        auto trimesh_shape = std::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(mmesh);
        trimesh_shape->SetName(filename);
        trimesh_shape->SetMutable(false);
        trimesh_shape->SetScale(chrono::ChVector<>(1, 1, 1));

        auto mesh_body = std::make_shared<chrono::ChBody>();
        mesh_body->SetPos(pos);
        mesh_body->SetRot(chrono::QUNIT);
        mesh_body->AddVisualShape(trimesh_shape);
        mesh_body->SetBodyFixed(true);
        car.GetSystem()->Add(mesh_body);

        if (class_id == 1) {
            red_cone_assets.push_back(trimesh_shape);
        } else {
            green_cone_assets.push_back(trimesh_shape);
        }
    }
}

void AddConesFromFile(RigidTerrain& terrain, artcar::ARTcar& car) {
    auto green_cone_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    green_cone_mesh->LoadWavefrontMesh(chrono::GetChronoDataPath() + "sensor/cones/green_cone.obj", false, true);
    green_cone_mesh->Transform(chrono::ChVector<>(0, 0, 0), chrono::ChMatrix33<>(1));

    auto red_cone_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    red_cone_mesh->LoadWavefrontMesh(chrono::GetChronoDataPath() + "sensor/cones/red_cone.obj", false, true);
    red_cone_mesh->Transform(chrono::ChVector<>(0, 0, 0), chrono::ChMatrix33<>(1));

    std::ifstream cone_file(chrono::GetChronoDataPath() + "autonomy-toolkit/paths/cone_paths_0.csv");
    std::string line;
    while (std::getline(cone_file, line)) {
        std::istringstream iss(line);
        double x_inner, y_inner, x_outer, y_outer;
        if (!(iss >> x_inner >> y_inner >> x_outer >> y_outer)) {
            break;
        }  // error

        double pos_green_x = x_inner + cone_offset_x;
        double pos_green_y = y_inner + cone_offset_y;
        double pos_green_z = terrain.GetHeight(chrono::ChVector<>(pos_green_x, pos_green_y, 1000));

        double pos_red_x = x_outer + cone_offset_x;
        double pos_red_y = y_outer + cone_offset_y;
        double pos_red_z = terrain.GetHeight(chrono::ChVector<>(pos_red_x, pos_red_y, 1000));

        auto pos_green = chrono::ChVector<>(pos_green_x, pos_green_y, pos_green_z);
        auto pos_red = chrono::ChVector<>(pos_red_x, pos_red_y, pos_red_z);
        auto rot = chrono::ChQuaternion<>(1, 0, 0, 0);

        auto green_cone_shape = std::make_shared<ChVisualShapeTriangleMesh>();
        green_cone_shape->SetMesh(green_cone_mesh);
        green_cone_shape->SetName("green_cone_shape");
        green_cone_shape->SetMutable(false);
        green_cone_shape->SetVisible(true);

        auto green_body = std::make_shared<chrono::ChBody>();
        green_body->SetPos(pos_green);
        green_body->SetRot(rot);
        green_body->AddVisualShape(green_cone_shape);
        green_body->SetBodyFixed(true);
        car.GetSystem()->Add(green_body);

        auto red_cone_shape = std::make_shared<ChVisualShapeTriangleMesh>();
        red_cone_shape->SetMesh(red_cone_mesh);
        red_cone_shape->SetName("red_cone_shape");
        red_cone_shape->SetMutable(false);
        red_cone_shape->SetVisible(true);

        auto red_body = std::make_shared<chrono::ChBody>();
        red_body->SetPos(pos_red);
        red_body->SetRot(rot);
        red_body->AddVisualShape(red_cone_shape);
        red_body->SetBodyFixed(true);
        car.GetSystem()->Add(red_body);

        red_cone_assets.push_back(red_cone_shape);
        green_cone_assets.push_back(green_cone_shape);
    }
}

void LabelConeAssets() {
    int cone_id = 0;
    std::cout << "red_cone_assets size: " << red_cone_assets.size() << std::endl;
    std::cout << "green_cone_assets size: " << green_cone_assets.size() << std::endl;
    for (auto cone : red_cone_assets) {
        cone_id += 1;
        for (auto mat : cone->GetMaterials()) {
            mat->SetClassID(20000);
            mat->SetInstanceID(cone_id);
        }
    }

    cone_id = 0;
    for (auto cone : green_cone_assets) {
        cone_id += 1;
        for (auto mat : cone->GetMaterials()) {
            mat->SetClassID(30000);
            mat->SetInstanceID(cone_id);
        }
    }
}