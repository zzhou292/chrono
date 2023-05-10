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
// Demo to show Viper Rover operated on SCM Terrain
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/ChWorldFrame.h"

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
using namespace chrono::geometry;
using namespace chrono::viper;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

bool output = true;
const std::string out_dir = GetChronoOutputPath() + "SCM_DEF_SOIL";

// SCM grid spacing
double mesh_resolution = 0.1;

// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// -----------------------------------------------------------------------------

// Custom callback for setting location-dependent soil properties.
// Note that the location is given in the SCM reference frame.
class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector<>& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        Bekker_Kphi = 0.82e6;
        Bekker_Kc = 0.14e4;
        Bekker_n = 1.0;
        Mohr_cohesion = 0.017e4;
        Mohr_friction = 35.0;
        Janosi_shear = 1.78e-2;
        elastic_K = 2e8;
        damping_R = 3e4;
    }
};

// Use custom material for the Viper Wheel
bool use_custom_mat = false;

// -----------------------------------------------------------------------------

// Return customized wheel material parameters
std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.1f;   // coefficient of restitution
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

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    std::string path_file = GetChronoDataFile("rover_obs_4.txt");
    std::shared_ptr<ChBezierCurve> path = ChBezierCurve::read(path_file, false);
    auto tracker = chrono_types::make_shared<ChBezierCurveTracker>(path);

    // Global parameter for moving patch size:
    double wheel_range = 0.5;
    ////double body_range = 1.2;

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::CSV_writer csv(" ");

    // Create the rover
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();

    Viper viper(&sys, wheel_type);

    viper.SetDriver(driver);
    if (use_custom_mat)
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));

    ChQuaternion<> rot;
    rot.Q_from_AngZ(CH_C_PI / 6);
    viper.Initialize(ChFrame<>(ChVector<>(-5, 4, -0.2), rot));

    // Get wheels and bodies to set up SCM patches
    auto Wheel_1 = viper.GetWheel(ViperWheelID::V_LF)->GetBody();
    auto Wheel_2 = viper.GetWheel(ViperWheelID::V_RF)->GetBody();
    auto Wheel_3 = viper.GetWheel(ViperWheelID::V_LB)->GetBody();
    auto Wheel_4 = viper.GetWheel(ViperWheelID::V_RB)->GetBody();
    auto Body_1 = viper.GetChassis()->GetBody();

    // Create obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rock;
    std::vector<std::string> rock_meshfile = {
        "robot/curiosity/rocks/rock1.obj",
        "robot/curiosity/rocks/rock1.obj",  //
        "robot/curiosity/rocks/rock2.obj",
        "robot/curiosity/rocks/rock2.obj",  //
        "robot/curiosity/rocks/rock3.obj",
        "robot/curiosity/rocks/rock3.obj",  //
        "robot/curiosity/rocks/rock3.obj",
        "robot/curiosity/rocks/rock3.obj",  //
        "robot/curiosity/rocks/rock2.obj",
        "robot/curiosity/rocks/rock2.obj",  //
    };

    std::vector<ChVector<>> rock_pos = {
        ChVector<>(3.0, -3.4, -0.1), ChVector<>(-5.2, -7.8, -0.1),  //
        ChVector<>(0.0, 0.8, -0.1),  ChVector<>(7.4, -1.3, -0.1),   //
        ChVector<>(-6.8, 2.0, -0.1), ChVector<>(1.9, -7.1, -0.1),   //
        ChVector<>(6.8, 4.0, -0.1),  ChVector<>(6.9, 7.0, -0.1),    //
        ChVector<>(-3.8, 0.0, -0.1), ChVector<>(-7.2, 6.3, -0.1)    //
    };

    double rock_density = 8000;
    std::shared_ptr<ChMaterialSurface> rock_mat = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());

    for (int i = 0; i < rock_pos.size(); i++) {
        auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(rock_meshfile[i]), false, true);
        mesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1.0));

        double mass;
        ChVector<> cog;
        ChMatrix33<> inertia;
        mesh->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

        auto body = chrono_types::make_shared<ChBodyAuxRef>();
        sys.Add(body);
        body->SetBodyFixed(false);
        body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock_pos[i]), QUNIT));
        body->SetFrame_COG_to_REF(ChFrame<>(cog, principal_inertia_rot));
        body->SetMass(mass * rock_density);
        body->SetInertiaXX(rock_density * principal_I);

        body->GetCollisionModel()->ClearModel();
        body->GetCollisionModel()->AddTriangleMesh(rock_mat, mesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
        body->GetCollisionModel()->BuildModel();
        body->SetCollide(true);

        auto mesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        mesh_shape->SetMesh(mesh);
        mesh_shape->SetBackfaceCull(true);
        body->AddVisualShape(mesh_shape);

        rock.push_back(body);
    }

    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain terrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    // Note: Irrlicht uses a Y-up frame
    terrain.SetPlane(ChCoordsys<>(ChVector<>(0, 0, -0.5)));

    // Use a regular grid:
    double length = 20;
    double width = 20;
    terrain.Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    if (var_params) {
        // Here we use the soil callback defined at the beginning of the code
        auto my_params = chrono_types::make_shared<MySoilParams>();
        terrain.RegisterSoilParametersCallback(my_params);
    } else {
        // If var_params is set to be false, these parameters will be used
        terrain.SetSoilParameters(0.2e6,  // Bekker Kphi
                                  0,      // Bekker Kc
                                  1.1,    // Bekker n exponent
                                  0,      // Mohr cohesive limit (Pa)
                                  30,     // Mohr friction limit (degrees)
                                  0.01,   // Janosi shear coefficient (m)
                                  4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                  3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );
    }

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain.EnableBulldozing(true);  // inflate soil at the border of the rut
        terrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // We need to add a moving patch under every wheel
    // Or we can define a large moving patch at the pos of the rover body
    if (enable_moving_patch) {
        terrain.AddMovingPatch(Wheel_1, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_2, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_3, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_4, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        for (int i = 0; i < rock_pos.size(); i++)
            terrain.AddMovingPatch(rock[i], VNULL, ChVector<>(2.0, 2.0, 2.0));
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);

    terrain.SetMeshWireframe(true);

    // Create the run-time visualization interface
    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(1920, 1080);
            vis_irr->SetWindowTitle("Viper Rover on SCM");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector<>(1.0, 2.0, 1.4), ChVector<>(0, 0, wheel_range));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector<>(-5.0, -0.5, 8.0), ChVector<>(-1, 0, 0), 100, 1, 35, 85, 512,
                                        ChColor(0.8f, 0.8f, 0.8f));
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
            vis_vsg->SetWindowSize(800, 600);
            vis_vsg->SetWindowTitle("Viper Rover on SCM");
            vis_vsg->AddCamera(ChVector<>(1.0, 2.0, 1.4), ChVector<>(0, 0, wheel_range));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    while (vis->Run()) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(Body_1->GetPos());
        vis->Render();
        ////tools::drawColorbar(vis.get(), 0, 20000, "Pressure yield [Pa]", 1180);
        vis->EndScene();
#endif

        // calculate sentinel point
        ChVector<> sentinel = viper.GetChassis()->GetBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(
            ChVector<>(1.0, 0.0, 0.0));  // lookahead distance 2
        ChVector<> sentinel_y =
            viper.GetChassis()->GetBody()->GetFrame_REF_to_abs().TransformPointLocalToParent(ChVector<>(0.0, 1.0, 0.0));
        ChVector<> target;
        tracker->calcClosestPoint(sentinel, target);

        ChVector<> sentinel_vec = sentinel - target;
        sentinel_vec.z() = 0.0;
        ChVector<> y_global_unit = sentinel_y - viper.GetChassisPos();

        float steer_mag = 2 * sentinel_vec.Dot(y_global_unit);

        if (steer_mag < -1.0)
            steer_mag = -1.0;
        else if (steer_mag > 1.0)
            steer_mag = 1.0;
        // std::cout << "sentinel:" << sentinel << std::endl;
        // std::cout << "target:" << target << std::endl;
        // std::cout << "sentinel_vec:" << sentinel_vec << std::endl;
        std::cout << "steer_mag: " << steer_mag << std::endl;
        std::cout << "pos: " << viper.GetChassisPos() << std::endl;

        float steer_ang = steer_mag * CH_C_PI / 8;

        driver->SetSteering(-steer_ang);
        sys.DoStepDynamics(5e-4);
        viper.Update();
        ////terrain.PrintStepStatistics(std::cout);

        if (output) {
            // write drive torques of all four wheels into file
            ChVector<> viper_vel = viper.GetChassisVel();
            ChVector<> viper_pos = viper.GetChassisPos();

            csv << sys.GetChTime() << "," << viper_vel.Length() << "," << viper_pos.x() << "," << viper_pos.y() << ","
                << steer_ang << std::endl;
        }
    }

    if (output) {
        csv.write_to_file(out_dir + "/output.csv");
    }

    return 0;
}
