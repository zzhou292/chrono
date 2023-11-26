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
// Authors: Sriram Ashokkumar
// =============================================================================
//
// Demo to show Cobra Rover operated on Rigid Terrain
// This Demo includes operation to spawn a Cobra rover, control wheel speed, and
// control rover steering
//
// =============================================================================

#include "chrono_models/robot/cobra/Cobra.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"

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

#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/robot/cobra/ChROSCobraDCMotorControlHandler.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"

#include "chrono/physics/ChInertiaUtils.h"

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
using namespace chrono::cobra;
using namespace chrono::sensor;
using namespace chrono::geometry;
using namespace chrono::ros;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Use custom material for the Cobra Rover Wheel
bool use_custom_mat = false;

// Define Cobra rover wheel type
CobraWheelType wheel_type = CobraWheelType::SimpleWheel;

// Simulation time step
double time_step = 5e-4;

// -----------------------------------------------------------------------------

void addCones(ChSystem& sys, std::vector<std::string>& cone_files, std::vector<ChVector<>>& cone_pos);

std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the Chrono system with gravity in the negative Z direction
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // auto object_mat = ChMaterialSurface::DefaultMaterial(contact_method);
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    auto room_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    room_mmesh->LoadWavefrontMesh(chrono::GetChronoDataFile("robot/environment/hallway_1/hallway.obj"), false, true);
    room_mmesh->Transform(chrono::ChVector<>(0, 0, 0), chrono::ChMatrix33<>(1));

    auto room_trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    room_trimesh_shape->SetMesh(room_mmesh);
    room_trimesh_shape->SetName("Hallway Mesh");
    room_trimesh_shape->SetMutable(false);

    auto room_mesh_body = chrono_types::make_shared<ChBody>();
    room_mesh_body->SetPos(chrono::ChVector<>(-2, -2, -0.1));
    room_mesh_body->AddVisualShape(room_trimesh_shape);
    room_mesh_body->SetBodyFixed(true);
    auto cshape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(ground_mat, room_mmesh, true, true);
    room_mesh_body->AddCollisionShape(cshape);
    room_mesh_body->SetCollide(true);

    sys.AddBody(room_mesh_body);

    // Construct a Cobra rover and the asociated driver
    auto driver = chrono_types::make_shared<CobraSpeedDriver>(1.0, 3.0);

    Cobra cobra(&sys, wheel_type);
    cobra.SetDriver(driver);
    if (use_custom_mat)
        cobra.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::SMC));

    cobra.Initialize(ChFrame<>(ChVector<>(0, 0, 0.1), QUNIT));

    // -----------------------
    // Adding cone objects
    // -----------------------

    // Create obstacles
    std::vector<std::string> cone_meshfile = {
        "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj",  //
        "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj",  //
        "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj",  //
        "sensor/cones/green_cone.obj", "sensor/cones/red_cone.obj"   //
    };
    std::vector<ChVector<>> cone_pos = {
        ChVector<>(4.5, -0.5, 0.005), ChVector<>(4.5, 0.5, 0.005),  //
        ChVector<>(4.0, -0.5, 0.005), ChVector<>(4.0, 0.5, 0.005),  //
        ChVector<>(3.5, -0.5, 0.005), ChVector<>(3.5, 0.5, 0.005),  //
        ChVector<>(3.0, -0.5, 0.005), ChVector<>(3.0, 0.5, 0.005)   //
    };

    addCones(sys, cone_meshfile, cone_pos);

    // Create chairs
    std::vector<std::string> chair_meshfile = {
        "robot/environment/objects/scan_chair_1/textured.obj", "robot/environment/objects/chair_1/swivel_chair_.obj",  //
        "robot/environment/objects/scan_chair_1/textured.obj",
        
    };

    std::vector<ChVector<>> chair_pos = {
        ChVector<>(6.5, 2.1, 0.005), ChVector<>(10.5, 1.5, 0.005),  //
        ChVector<>(13.0, 1.3, 0.005)  //
    };

    addCones(sys, chair_meshfile, chair_pos);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({12.0, 0.0, 1.89}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    chrono::ChFrame<double> offset_pose1({0, 0, 1.2}, Q_from_AngAxis(CH_C_PI / 18, {0, 1, 0}));
 

    auto cam = chrono_types::make_shared<ChCameraSensor>(cobra.GetChassis()->GetBody(), 20, offset_pose1, 1280, 720,
                                                         CH_C_PI / 4, 1, CameraLensModelType::PINHOLE,false, 2.2);
    
    cam->PushFilter(chrono_types::make_shared<ChFilterCameraNoisePixDep>(.0004f, .0004f));
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Global //Illumination"));
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    // ------------------------------------------------
    // Create a lidar and add it to the sensor manager
    // ------------------------------------------------
    float horizontal_fov = (float)(2 * CH_C_PI);  // 360 degree scan
    float max_vert_angle = (float)CH_C_PI / 12;   // 15 degrees up
    float min_vert_angle = (float)-CH_C_PI / 6;   // 30 degrees down
    chrono::ChFrame<double> offset_pose2({0.2, 0, 0.5}, Q_from_AngAxis(0, {0, 1, 0}));
    // Number of horizontal and vertical samples
    unsigned int horizontal_samples = 256;
    unsigned int vertical_samples = 64;
    auto lidar =
        chrono_types::make_shared<ChLidarSensor>(cobra.GetChassis()->GetBody(),          // body lidar is attached to
                                                 10,                                     // scanning rate in Hz
                                                 offset_pose2,                           // offset pose
                                                 horizontal_samples,                     // number of horizontal samples
                                                 vertical_samples,                       // number of vertical channels
                                                 horizontal_fov,                         // horizontal field of view
                                                 max_vert_angle, min_vert_angle, 100.0f  // vertical field of view
        );
    lidar->SetName("Lidar Sensor");
    lidar->SetLag(.05f);
    lidar->SetCollectionWindow(.02f);

    // Provides the host access to the Depth,Intensity data
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());

    // Renders the raw lidar data
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(horizontal_samples / 2, vertical_samples * 5,
                                                                   "Raw Lidar Depth Data"));

    // Convert Depth,Intensity data to XYZI point
    // cloud data
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());

    lidar->PushFilter(chrono_types::make_shared<ChFilterLidarNoiseXYZI>(0.005f, 0.0005f, 0.0005f, 0.005f));

    // Render the point cloud
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 0.2, "Lidar Point Cloud"));

    // Access the lidar data as an XYZI buffer
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());

    // add sensor to the manager
    manager->AddSensor(lidar);

    // ----------------------------------------------
    // Create imu related sensors and add them to the sensor manager
    // ----------------------------------------------
    int imu_update_rate = 100;
    float imu_lag = 0;
    float imu_collection_time = 0;
    std::shared_ptr<ChNoiseModel> acc_noise_model;
    std::shared_ptr<ChNoiseModel> gyro_noise_model;
    std::shared_ptr<ChNoiseModel> mag_noise_model;
    // Set the imu noise model to a gaussian model
    acc_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,                          //
                                                                    ChVector<double>({0., 0., 0.}),           // mean,
                                                                    ChVector<double>({0.001, 0.001, 0.001}),  // stdev,
                                                                    .0001,             // bias_drift,
                                                                    .1);               // tau_drift,
    gyro_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,  // float updateRate,
                                                                     ChVector<double>({0., 0., 0.}),  // float mean,
                                                                     ChVector<double>({0.001, 0.001, 0.001}),  // float
                                                                     .001,  // double bias_drift,
                                                                     .1);   // double tau_drift,
    mag_noise_model =
        chrono_types::make_shared<ChNoiseNormal>(ChVector<double>({0., 0., 0.}),            // float mean,
                                                 ChVector<double>({0.001, 0.001, 0.001}));  // float stdev,

    // add an accelerometer, gyroscope, and magnetometer to one of the pendulum legs
    auto imu_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(
        cobra.GetChassis()->GetBody(),  // body to which the IMU is attached
        imu_update_rate,                // update rate
        imu_offset_pose,                // offset pose from body
        acc_noise_model);               // IMU noise model
    acc->SetName("IMU - Accelerometer");
    acc->SetLag(imu_lag);
    acc->SetCollectionWindow(imu_collection_time);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());  // Add a filter to access the imu data
    manager->AddSensor(acc);                                            // Add the IMU sensor to the sensor manager

    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(
        cobra.GetChassis()->GetBody(),  // body to which the IMU is attached
        imu_update_rate,                // update rate
        imu_offset_pose,                // offset pose from body
        gyro_noise_model);              // IMU noise model
    gyro->SetName("IMU - Accelerometer");
    gyro->SetLag(imu_lag);
    gyro->SetCollectionWindow(imu_collection_time);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());  // Add a filter to access the imu data
    manager->AddSensor(gyro);                                           // Add the IMU sensor to the sensor manager

    // Create the run-time visualization interface
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Cobra Rover on Rigid Terrain");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector<>(1.5, 1.5, 0.6));
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
            vis_vsg->AddCamera(ChVector<>(0, 0, 2));
            vis_vsg->SetWindowTitle("Cobra Rover on Rigid Terrain");
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    manager->Update();

    // ----------------

    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Create baselink lidar tf pubisher
    auto lidar_baselink_tf_rate = 50;
    auto lidar_baselink_tf_topic_name = "/tf";
    auto lidar_baselink_tf_handler = chrono_types::make_shared<ChROSTFHandler>(lidar_baselink_tf_rate, lidar, cobra.GetChassis()->GetBody(), lidar_baselink_tf_topic_name);
    ros_manager->RegisterHandler(lidar_baselink_tf_handler);

    // Create the publisher for the camera
    auto camera_rate = cam->GetUpdateRate();
    auto camera_topic_name = "~/output/camera/data/image";
    auto camera_handler = chrono_types::make_shared<ChROSCameraHandler>(camera_rate, cam, camera_topic_name);
    ros_manager->RegisterHandler(camera_handler);

    // Create the publisher for the lidar
    auto lidar_topic_name = "/input_cloud";
    auto lidar_handler = chrono_types::make_shared<ChROSLidarHandler>(lidar, lidar_topic_name);
    ros_manager->RegisterHandler(lidar_handler);

    // Create a subscriber for the driver inputs
    auto driver_inputs_rate = 25;
    auto driver_inputs_topic_name = "cmd_vel";
    auto driver_inputs_handler =
        chrono_types::make_shared<ChROSCobraSpeedDriverHandler>(driver_inputs_rate, driver, driver_inputs_topic_name);
    ros_manager->RegisterHandler(driver_inputs_handler);

    // Create a publisher for the rover state
    auto rover_state_rate = 25;
    auto rover_state_topic_name = "~/output/rover/state";
    auto rover_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
        rover_state_rate, cobra.GetChassis()->GetBody(), rover_state_topic_name);
    ros_manager->RegisterHandler(rover_state_handler);

    // Finally, initialize the ros manager
    ros_manager->Initialize();
    // ---------------

    // Simulation loop
    while (vis->Run()) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
#endif

        // Set current steering angle
        double time = cobra.GetSystem()->GetChTime();

        // if (time > 2.0 && time < 6.0)
        //     driver->SetSteering(0.5);
        // else if (time > 6.0 && time < 10.0)
        //     driver->SetSteering(-0.5);
        // else
        //     driver->SetSteering(0.0);

        // if (time > 10.0)
             driver->SetMotorSpeed(0.8);

        // Update Cobra controls
        cobra.Update();

        sys.DoStepDynamics(time_step);

        manager->Update();

        if (!ros_manager->Update(time, time_step))
            break;
    }

    return 0;
}

void addCones(ChSystem& sys, std::vector<std::string>& cone_files, std::vector<ChVector<>>& cone_pos) {
    std::vector<std::shared_ptr<ChBodyAuxRef>> cone;
    double cone_density = 900;
    std::shared_ptr<ChMaterialSurface> rock_mat = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());

    for (int i = 0; i < cone_files.size(); i++) {
        auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(cone_files[i]), true, true);

        double mass;
        ChVector<> cog;
        ChMatrix33<> inertia;
        mesh->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

        auto body = chrono_types::make_shared<ChBodyAuxRef>();
        sys.Add(body);
        body->SetBodyFixed(true);
        body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(cone_pos[i]), QUNIT));
        body->SetFrame_COG_to_REF(ChFrame<>(cog, principal_inertia_rot));
        body->SetMass(mass * cone_density);
        body->SetInertiaXX(cone_density * principal_I);
        body->SetCollide(false);

        auto mesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        mesh_shape->SetMesh(mesh);
        mesh_shape->SetBackfaceCull(true);
        body->AddVisualShape(mesh_shape);
    }
}
