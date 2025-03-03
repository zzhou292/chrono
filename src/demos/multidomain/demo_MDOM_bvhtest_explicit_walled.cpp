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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about splitting a system into domains using the MULTIDOMAIN module
// using distributed memory. This needs MPI, as many executables will be spawn
// on different computing nodes.
//
// This demo shows smooth contact dynamics using explicit integration.
//
// The timestepper is switched to explicit integration with diagonal lumped
// mass solver, ChSolverLumpedMultidomain. this solver is very simple and does not
// require iterations, but the time step must be small.
//
// This also means that this solver is not capable of solving nonsmooth DVI,
// so here  contacts if any are handled using smooth contact, via  ChSystemSMC
// and ChContactMaterialSMC.
//
// We add all objects into a "master domain" that wraps all the scene, then we
// let that  DoAllDomainInitialize() will split all the items automatically into
// the corresponding domains. (This is easy and almost error-proof, but if you
// need to avoid a master domain for efficiency/space reasons, look at the
// alternative model creation mode in ...lowlevel.cpp demos.)
//
// NOTE! in MPI architectures, the executable cannot be started in a single copy as all other
// chrono demos: now you must open a shell where you can access "mpiexec" (or other commands
// depending on the MPI version that you installed, it could be "mpirun" or such) and type
// the command to start parallel computation, for example spawning 3 processes:
//
//   mpiexec -n 3 demo_MDOM_MPI_explicit.exe
//
// This demo can use -n X processes, with X=(ndomains+1master), ex -n 5 builds 4 domain slices,
// plus one process that is dedicated to the master domain. Ex:
//   mpiexec -n 2 demo_MDOM_MPI_explicit.exe   -> reference case of 1 domain
//   mpiexec -n 3 demo_MDOM_MPI_explicit.exe   -> simplest multidomain: 2 domains
//
// After running, it produces postprocessing files ready for rendering in Blender3D
// To see the result in Blender3D, you must install the add-in for loading Chrono files,
// that is  chrono\src\importer_blender\chrono_import.py. So you can do:
// - menu  file/Import/Chrono import...
// - check the "Automatic merge" in the top right panel
// - check the "Skip shared names" too, to avoid duplicates at shared bodies
// - browse to  chrono\bin\Release\DEMO_OUTPUT\MDOM_MPI_0  and press the
//   "Import Chrono simulation" button.
// Thanks to the automatic merge, also outputs in ..\MDOM_MPI_2 ..\MDOM_MPI_3 etc
// will be load automatically; the system will recognize the incremental numbering.
// (Alternatively you can load the single domain results one by one, turning on the "Merge"
// option while you call the  Chrono import...  menu multiple times.)
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveUtils.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"

#include "chrono_multidomain/ChSolverLumpedMultidomain.h"
#include "chrono_multidomain/ChDomainManagerMPI.h"
#include "chrono_multidomain/ChDomainBuilder.h"
#include "chrono_multidomain/BVH_Builder/bvh_builder.hpp"
#include "chrono_postprocess/ChBlender.h"

#include <chrono>
#include <iomanip>

using namespace chrono;
using namespace multidomain;
using namespace postprocess;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // This source generates a simple executable with basic std::cout output on the command
    // line console, enough to show basic functionality of the multidomain module.
    //
    // All the examples herein are based on the ChDomainManagerMPI domain manager, that
    // allows using MPI to communicate between processes distributed on a computer cluster
    // with many nodes connected by ethernet, or Mellanox, or similar supercomputing architectures.

    // 1- First you need a ChDomainManagerMPI. This will use MPI distributed computing
    //    as a method for parallelism (spreading processes on multiple computing nodes)

    ChDomainManagerMPI domain_manager(&argc, &argv);

    // This demo assumes 1..7 processes (ex. 3 processes =2 slices + 1 master domain, i.e. "mpiexec -n 3 ..." etc.)
    assert(domain_manager.GetMPItotranks() >= 1);
    assert(domain_manager.GetMPItotranks() < 8);

    // For debugging/logging:
    domain_manager.verbose_partition = false;         // will print  partitioning in std::cout?
    domain_manager.verbose_serialization = false;     // will print interdomain serialization in std::cout?
    domain_manager.verbose_variable_updates = false;  // will print all comm including variable updates in std::cout?
    domain_manager.serializer_type =
        DomainSerializerFormat::BINARY;  // default BINARY, use JSON or XML for readable verbose

    // 2- Now you need a domain builder.
    //    You must define how the 3D space is divided in domains.
    //    ChdomainBuilder classes help you to do this.
    //    Here we split it using parallel planes like in sliced bread:
    //    Since we use a helper master domain, [n.of MPI ranks] = [n.of slices] + 1

    auto domain_builder = std::make_shared<ChDomainBuilderBVHMPI>(domain_manager.GetMPItotranks() - 1, true);

    // 3- Create the ChDomain object and its ChSystem physical system.
    //    Only a single system is created, because this is already one of n
    //    parallel processes. One must be the master domain.

    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // 5- we populate ONLY THE MASTER domain with bodies, links, meshes, nodes, etc.
    //    At the beginning of the simulation, the master domain will break into
    //    multiple data structures and will serialize them into the proper subdomains.
    //    (Note that there is a "low level" version of this demo that shows how
    //    to bypass the use of the master domain, in case of extremely large systems
    //    where you might want to create objects directly split in the n-th computing node.)

    if (domain_manager.GetMPIrank() == domain_builder->GetMasterRank()) {
        auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
        mat->SetFriction(0.1f);
        mat->SetYoungModulus(2e8);

        // Create some bricks placed as walls, for benchmark purposes
        int n_walls = 1;
        int n_vertical = 20;
        int n_horizontal = 20;
        double size_x = 4;
        double size_y = 2;
        double size_z = 4;
        double walls_space = 9;
        double wall_corner_x = -0.5 * (size_x * n_horizontal) - 0.1;
        for (int ai = 0; ai < n_walls; ai++) {               // loop of walls
            for (int bi = 0; bi < n_vertical; bi++) {        // loop of vert. bricks
                for (int ui = 0; ui < n_horizontal; ui++) {  // loop of hor. bricks

                    auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(size_x * 0.9, size_y, size_z,
                                                                               2000,  // density
                                                                               true,  // visualization?
                                                                               true,  // collision?
                                                                               mat);  // contact material
                    mrigidBody->SetPos(ChVector3d(wall_corner_x + size_x * (ui + 0.5 * (1 + bi % 2)),
                                                  size_y * (0.5 + bi), ai * walls_space + 2.0 * ui));
                    mrigidBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_borders.png"));
                    sys.Add(mrigidBody);
                    std::cout << "Added body with index: " << mrigidBody->GetIndex() << std::endl;
                }
            }
        }

        // Create a ball that will collide with wall
        auto mrigidBall1 = chrono_types::make_shared<ChBodyEasySphere>(5.0,   // radius
                                                                       8000,  // density
                                                                       true,  // visualization?
                                                                       true,  // collision?
                                                                       mat);  // contact material
        mrigidBall1->SetPos(ChVector3d(0, 5.0, -8));
        mrigidBall1->SetPosDt(ChVector3d(0, 0, 16));  // set initial speed
        mrigidBall1->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
        sys.Add(mrigidBall1);

        auto mrigidBall2 = chrono_types::make_shared<ChBodyEasySphere>(5.0,   // radius
                                                                       8000,  // density
                                                                       true,  // visualization?
                                                                       true,  // collision?
                                                                       mat);  // contact material
        mrigidBall2->SetPos(ChVector3d(15, 5.0, -10));
        mrigidBall2->SetPosDt(ChVector3d(2, 0, 16));  // set initial speed
        mrigidBall2->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
        sys.Add(mrigidBall2);

        auto mrigidBall3 = chrono_types::make_shared<ChBodyEasySphere>(5.0,   // radius
                                                                       8000,  // density
                                                                       true,  // visualization?
                                                                       true,  // collision?
                                                                       mat);  // contact material
        mrigidBall3->SetPos(ChVector3d(-15, 5.0, -10));
        mrigidBall3->SetPosDt(ChVector3d(-2, 0, 16));  // set initial speed
        mrigidBall3->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
        sys.Add(mrigidBall3);

        // 6- Set the tag IDs for all nodes, bodies, etc.
        //    To do this, use the helper ChArchiveSetUniqueTags, that traverses all the
        //    hierarchies, sees if there is a SetTag() function in sub objects, ans sets
        //    the ID incrementally. More hassle-free than setting all IDs one by one by
        //    hand as in ...lowlevel.cpp demos
        //    Call this after you finished adding items to systems.
        ChArchiveSetUniqueTags tagger;
        tagger.skip_already_tagged = false;
        tagger << CHNVP(sys);
    }

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.1f);
    mat->SetYoungModulus(2e8);

    // Create the floor using fixed rigid body of 'box' type:
    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(130, 4, 130,  // x,y,z size
                                                                5000,         // density
                                                                true,         // visulization?
                                                                true,         // collision?
                                                                mat);         // contact material
    mrigidFloor->SetPos(ChVector3d(0, -2, 0));
    mrigidFloor->SetFixed(true);
    domain_builder->AddExcludedBody(mrigidFloor);

    sys.Add(mrigidFloor);

    std::array<ChVector3d, 4> wall_positions = {ChVector3d(0, 20, 65), ChVector3d(0, 20, -65), ChVector3d(65, 20, 0),
                                                ChVector3d(-65, 20, 0)};

    std::array<ChQuaternion<>, 4> wall_rotations = {QuatFromAngleY(CH_PI_2), QuatFromAngleY(CH_PI_2),
                                                    ChQuaternion<>(1, 0, 0, 0), ChQuaternion<>(1, 0, 0, 0)};

    for (int i = 0; i < 4; i++) {
        auto mrigidWall = chrono_types::make_shared<ChBodyEasyBox>(4, 60, 130,  // x,y,z size
                                                                   5000,        // density
                                                                   true,        // visulization?
                                                                   true,        // collision?
                                                                   mat);        // contact material
        mrigidWall->SetPos(wall_positions[i]);
        mrigidWall->SetRot(wall_rotations[i]);
        mrigidWall->SetFixed(true);
        domain_builder->AddExcludedBody(mrigidWall);

        sys.Add(mrigidWall);
    }

    // add a moving wall
    auto mrigidMovingWall = chrono_types::make_shared<ChBodyEasyBox>(4, 60, 130,  // x,y,z size
                                                                     5000,        // density
                                                                     true,        // visulization?
                                                                     true,        // collision?
                                                                     mat);        // contact material

    mrigidMovingWall->SetRot(ChQuaternion<>(1, 0, 0, 0));
    mrigidMovingWall->SetFixed(true);
    mrigidMovingWall->SetPosDt(ChVector3d(-5, 0, 2));
    sys.Add(mrigidMovingWall);
    domain_builder->AddExcludedBody(mrigidMovingWall);

    // Materdomain BVH update
    domain_builder->ComputeAndBroadcastDomainAABBs(&sys, domain_manager.GetMPIrank());

    if (domain_manager.GetMPIrank() == domain_builder->GetMasterRank()) {
        domain_manager.SetDomain(domain_builder->BuildMasterDomain(&sys  // physical system of this domain
                                                                   ));

    } else {
        domain_manager.SetDomain(domain_builder->BuildDomain(
            &sys,                        // physical system of this domain
            domain_manager.GetMPIrank()  // rank of this domain, must be unique and starting from 0
            ));
    }

    // THEN set the solver again to override the default PSOR
    // 4- Set solver, timestepper, etc. that can work in multidomain mode. Do this after SetDomain().
    //    Set the time stepper: as we demonstrate smooth contacts via ChSystemSMC, we can use an explicit time stepper.
    auto explicit_timestepper0 = chrono_types::make_shared<ChTimestepperEulerExplIIorder>(&sys);
    explicit_timestepper0->SetConstraintsAsPenaltyON(
        2e6);  // use penalty for constraints, skip linear systems completely
    sys.SetTimestepper(explicit_timestepper0);
    //    Set the solver: efficient ChSolverLumpedMultidomain (needs explicit timestepper with constraint penalty)
    auto lumped_solver0 = chrono_types::make_shared<ChSolverLumpedMultidomain>();
    sys.SetSolver(lumped_solver0);

    std::cout << "RANK:" << domain_manager.GetMPIrank() << " tp2" << std::endl;

    // OPTIONAL: POSTPROCESSING VIA BLENDER3D

    // Create an exporter to Blender
    ChBlender blender_exporter = ChBlender(&sys);
    // Set the path where it will save all files, a directory will be created if not existing.
    // The directories will have a name depending on the rank: MDOM_MPI_0, MDOM_MPI_1, MDOM_MPI_2, etc.
    blender_exporter.SetBasePath(GetChronoOutputPath() + "MDOM_MPI_" + std::to_string(domain_manager.GetMPIrank()));
    // To help with shared bodies:
    blender_exporter.SetUseTagsAsBlenderNames(true);
    // Show contacts in a nice way, or blender_exporter.SetShowContactsOff() for fast stuff
    blender_exporter.SetShowContactsVectors(
        ChBlender::ContactSymbolVectorLength::PROPERTY,  // vector symbol lenght is given by |F|  property (contact
                                                         // force)
        0.001,  // absolute length of symbol if CONSTANT, or length scaling factor if ATTR
        "F",    // name of the property to use for lenght if in ATTR mode (currently only option: "F")
        ChBlender::ContactSymbolVectorWidth::CONSTANT,  // vector symbol lenght is a constant value
        0.2,  // absolute width of symbol if CONSTANT, or width scaling factor if ATTR
        "",   // name of the property to use for width if in ATTR mode (currently only option: "F")
        ChBlender::ContactSymbolColor::CONSTANT,  // vector symbol color is a falsecolor depending on |F| property
                                                  // (contact force)
        ChColor(1, 0, 0),                         // absolute constant color if CONSTANT, not used if ATTR
        "F",      // name of the property to use for falsecolor scale if in ATTR mode (currently only option: "F")
        0, 1000,  // falsecolor scale min - max values.
        true      // show a pointed tip on the end of the arrow, otherwise leave a simple cylinder
    );
    // Initial script, save once at the beginning
    blender_exporter.ExportScript();

    std::cout << "RANK:" << domain_manager.GetMPIrank() << " tp3" << std::endl;
    // 7 - INITIAL SETUP AND OBJECT INITIAL MIGRATION!
    //     Moves all the objects in master domain to all domains, slicing the system.
    //     Also does some initializations, like collision detection AABBs.
    domain_manager.DoDomainInitialize(domain_manager.GetMPIrank());

    // The master domain does not need to communicate anymore with the domains so do:
    domain_manager.master_domain_enabled = false;

    // Variables for timing
    double total_sim_time = 0.0;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto last_report_time = start_time;
    int report_interval = 1000;  // Report RTF every 1000 steps

    // Add variables for partition timing
    double total_partition_time = 0.0;
    double total_bvh_build_time = 0.0;
    double total_dynamics_step_time = 0.0;

    // After setting up the solver
    std::cout << "Solver type: " << typeid(*sys.GetSolver()).name() << std::endl;

    // In your simulation loop
    std::cout << "Current solver type: " << typeid(*sys.GetSolver()).name() << std::endl;

    double step_size = 0.001;
    for (int i = 0; i < 14000; ++i) {
        mrigidMovingWall->SetPos(ChVector3d(60 - 6.0 * i * step_size, 20, 0 + 0.0 * i * step_size));

        if (domain_manager.GetMPIrank() == 0)
            std::cout << "\n\n\n============= Time step (explicit) " << i << std::endl << std::endl;

        if (i % 100 == 0) {
            // Time the partition update
            auto bvh_start = std::chrono::high_resolution_clock::now();
            // BVH update
            domain_builder->RebuildDomains(&sys, domain_manager.GetMPIrank());

            auto bvh_end = std::chrono::high_resolution_clock::now();
            total_bvh_build_time += std::chrono::duration<double>(bvh_end - bvh_start).count();

            // MULTIDOMAIN AUTOMATIC ITEM MIGRATION!
            domain_manager.DoDomainPartitionUpdate(domain_manager.GetMPIrank());
        }

        // JZ Async Testing Code
        // domain_builder->RebuildDomainsAsync(&sys, domain_manager.GetMPIrank(), i);

        // if (domain_builder->GetPartitionUpdateNeeded()) {
        //     domain_manager.DoDomainPartitionUpdate(domain_manager.GetMPIrank());
        // }

        // // Fail safe mechanism to update all domain AABBs without running BVH

        domain_builder->UpdateLocalDomainAABBs(&sys, domain_manager.GetMPIrank());

        // Time the partition update
        auto partition_start = std::chrono::high_resolution_clock::now();

        // MULTIDOMAIN AUTOMATIC ITEM MIGRATION!
        domain_manager.DoDomainPartitionUpdate(domain_manager.GetMPIrank());

        auto partition_end = std::chrono::high_resolution_clock::now();
        total_partition_time += std::chrono::duration<double>(partition_end - partition_start).count();

        // MULTIDOMAIN TIME INTEGRATION

        auto dynamics_start = std::chrono::high_resolution_clock::now();
        sys.DoStepDynamics(step_size);
        auto dynamics_end = std::chrono::high_resolution_clock::now();
        std::cout << "Time taken by dynamics step: "
                  << std::chrono::duration<double>(dynamics_end - dynamics_start).count() << " seconds" << std::endl;
        total_dynamics_step_time += std::chrono::duration<double>(dynamics_end - dynamics_start).count();

        total_sim_time += step_size;

        // // if (domain_manager.GetMPIrank() != domain_builder->GetMasterRank()) {
        // std::cout << "num of bodies: " << sys.GetBodies().size() << std::endl;

        // // get num of contacts
        // int num_contacts = sys.GetNumContacts();
        // std::cout << "num of contacts: " << num_contacts << std::endl;
        // //}

        // Calculate and report RTF periodically
        if (i % report_interval == 0 && i > 0) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(current_time - last_report_time).count();
            double sim_time_in_interval = report_interval * step_size;
            double rtf = sim_time_in_interval / elapsed_seconds;

            if (domain_manager.GetMPIrank() == 0) {
                std::cout << "RTF at step " << i << ": " << std::fixed << std::setprecision(3) << rtf
                          << " (sim time: " << sim_time_in_interval << "s, wall time: " << elapsed_seconds << "s)"
                          << std::endl;
            }

            last_report_time = current_time;
        }

        // OPTIONAL POSTPROCESSING FOR 3D RENDERING
        // Before ExportData(), we must do a remove-add trick as an easy way to handle the fact
        // that objects are constantly added and removed from the i-th domain when crossing boundaries.
        // Since timestep is small for explicit integration, save only every n-th time steps.
        if ((i % 100) == 0) {
            blender_exporter.RemoveAll();
            blender_exporter.AddAll();
            blender_exporter.ExportData();
        }
    }

    // Calculate overall RTF at the end
    auto end_time = std::chrono::high_resolution_clock::now();
    double total_elapsed_seconds = std::chrono::duration<double>(end_time - start_time).count();
    double overall_rtf = total_elapsed_seconds / total_sim_time;

    // Calculate partition time percentage
    double partition_percentage = (total_partition_time / total_elapsed_seconds) * 100.0;

    if (domain_manager.GetMPIrank() == 0) {
        std::cout << "\n=== Performance Summary ===" << std::endl;
        std::cout << "Total simulation time: " << total_sim_time << " seconds" << std::endl;
        std::cout << "Total wall clock time: " << total_elapsed_seconds << " seconds" << std::endl;
        std::cout << "Overall RTF: " << std::fixed << std::setprecision(3) << overall_rtf << std::endl;
        std::cout << "Time spent in DoDomainPartitionUpdate: " << std::fixed << std::setprecision(3)
                  << total_partition_time << " seconds (" << partition_percentage << "%)" << std::endl;
        std::cout << "Time spent in BVH build: " << std::fixed << std::setprecision(3) << total_bvh_build_time
                  << " seconds (" << (total_bvh_build_time / total_elapsed_seconds) * 100.0 << "%)" << std::endl;
        std::cout << "Time spent in dynamics step: " << std::fixed << std::setprecision(3) << total_dynamics_step_time
                  << " seconds (" << (total_dynamics_step_time / total_elapsed_seconds) * 100.0 << "%)" << std::endl;
    }

    return 0;
}
