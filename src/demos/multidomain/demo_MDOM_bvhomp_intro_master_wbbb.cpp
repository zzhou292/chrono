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
// =============================================================================
//
//  Demo code about splitting a system into domains using the MULTIDOMAIN module
//  using shared memory (just OpenMP multithreading, no MPI).
//
//  This demo shows basic functionality of the module: a body traveling through
//  two domains with a sphere connected to its top via a link, and hitting another
//  domain.
//
//  We add all objects into a "master domain" that wraps all the scene, then we
//  let that  DoAllDomainInitialize() will split all the items automatically into
//  the corresponding domains. (This is easy and almost error-proof, but if you
//  need to avoid a master domain for efficiency/space reasons, look at the
//  alternative model creation mode in ...lowlevel.cpp demos.)
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveUtils.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_multidomain/ChDomainManagerSharedmemory.h"
#include "chrono_multidomain/ChDomainBuilder.h"
#include "chrono_multidomain/ChSolverPSORmultidomain.h"
#include "chrono_multidomain/ChSolverBBmultidomain.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace multidomain;
using namespace chrono::irrlicht;
using namespace chrono::fea;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // This source generates a simple executable with basic std::cout output on the command
    // line console, enough to show basic functionality of the multidomain module.
    //
    // All the examples herein are based on the ChDomainManagerSharedmemory  domain manager,
    // that is, no need for MPI because the domains are split in the single executable using
    // OpenMP multithreading. This OpenMP multithreading is enough for didactical reasons,
    // then more advanced stuff must run usnig the MPI domain manager, shown in other demos.

    // 1- First you need a ChDomainManagerSharedmemory. This will use OpenMP multithreading
    //    as a method for parallelism (spreading processes on multiple computing nodes)

    ChDomainManagerSharedmemory domain_manager;

    // For debugging/logging:
    domain_manager.verbose_partition = false;         // will print partitioning in std::cout ?
    domain_manager.verbose_serialization = false;     // will print serialization buffers in std::cout ?
    domain_manager.verbose_variable_updates = false;  // will print all messages in std::cout ?
    domain_manager.serializer_type =
        DomainSerializerFormat::BINARY;  // default BINARY, use JSON or XML for readable verbose

    // 2- Now you need a domain builder.
    //    You must define how the 3D space is divided in domains.
    //    ChdomainBuilder classes help you to do this.
    //    Here we split it using parallel planes like in sliced bread.
    //    Since we use a helper master domain, [n.of threads] = [n.of slices] + 1

    ChDomainBuilderBVHOMP domain_builder(
        3,      // number of domains
        true);  // build also master domain, interfacing to all slices, for initial injection of objects

    // 4- Create and populate the MASTER domain with bodies, links, meshes, nodes, etc.
    //    At the beginning of the simulation, the master domain will break into
    //    multiple data structures and will serialize them into the proper subdomains.
    //    (Note that there is also a "low level" version of this demo that shows how
    //    to bypass the use of the master domain, in case of extremely large systems
    //    where you might want to create objects directly split in the n-th computing node.)

    // First we create the master domain and add it to the domain manager:
    ChSystemNSC sys_master;
    sys_master.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // ChCollisionModel::SetDefaultSuggestedEnvelope(0.00001);
    // ChCollisionModel::SetDefaultSuggestedMargin(0.00001);
    auto bb_solver0 = chrono_types::make_shared<ChSolverBBmultidomain>();
    sys_master.SetSolver(bb_solver0);
    // sys_master.GetSolver()->AsIterative()->SetMaxIterations(20);
    // sys_master.GetSolver()->AsIterative()->SetTolerance(1e-6);

    // Ok, now we proceed as usual in Chrono, adding items into the system :-)

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.1);
    int n_walls = 2;
    int n_vertical = 10;
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
                                                                           400,   // density
                                                                           true,  // visualization?
                                                                           true,  // collision?
                                                                           mat);  // contact material
                mrigidBody->SetPos(ChVector3d(wall_corner_x + size_x * (ui + 0.5 * (1 + bi % 2)), size_y * (0.5 + bi),
                                              ai * walls_space));
                mrigidBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_borders.png"));
                sys_master.Add(mrigidBody);
                std::cout << "Added body with index: " << mrigidBody->GetIndex() << std::endl;
            }
        }
    }

    // Create a ball that will collide with wall
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(3.5,   // radius
                                                                  8000,  // density
                                                                  true,  // visualization?
                                                                  true,  // collision?
                                                                  mat);  // contact material
    mrigidBall->SetPos(ChVector3d(0, 3.5, -8));
    mrigidBall->SetPosDt(ChVector3d(0, 0, 16));  // set initial speed
    mrigidBall->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    sys_master.Add(mrigidBall);

    // Create the floor using fixed rigid body of 'box' type:
    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
                                                                1000,         // density
                                                                true,         // visulization?
                                                                true,         // collision?
                                                                mat);         // contact material
    mrigidFloor->SetPos(ChVector3d(0, -2, 0));
    mrigidFloor->SetFixed(true);
    domain_builder.AddExcludedBody(mrigidFloor);

    sys_master.Add(mrigidFloor);

    // 5- Set the tag IDs for all nodes, bodies, etc.
    //    To do this, use the helper ChArchiveSetUniqueTags, that traverses all the
    //    hierarchies, sees if there is a SetTag() function in sub objects, and sets
    //    the ID incrementally. More hassle-free than setting all IDs one by one by
    //    hand as in ...lowlevel.cpp demos
    //    Call this after you finished adding items to systems.
    ChArchiveSetUniqueTags tagger;
    tagger.skip_already_tagged = false;
    tagger << CHNVP(sys_master);

    // 3- Create the ChDomain objects and their distinct ChSystem physical systems.
    //    Now one can know how many domains are expected to build, using domain_builder.GetTotSlices();

    domain_builder.ComputeAndBroadcastDomainAABBs(&sys_master);

    std::vector<ChSystemNSC*> sys_slices(domain_builder.GetTotRanks() - 1);

    for (int i = 0; i < domain_builder.GetTotRanks() - 1; ++i) {
        sys_slices[i] = new ChSystemNSC;
        sys_slices[i]->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

        domain_manager.AddDomain(domain_builder.BuildDomain(
            sys_slices[i],  // physical system of this domain
            i               // rank of this domain, must be unique and starting from 0!
            ));

        // Set solver, timestepper, etc. that can work in multidomain mode. Do this after AddDomain().
        // (The solver has been defaulted to ChSolverPSORmultidomain when we did domain_manager.AddDomain().)
        auto bb_solver = chrono_types::make_shared<ChSolverBBmultidomain>();
        sys_slices[i]->SetSolver(bb_solver);
        // sys_slices[i]->GetSolver()->AsIterative()->SetMaxIterations(20);
        // sys_slices[i]->GetSolver()->AsIterative()->SetTolerance(1e-6);
    }

    for (int i = 0; i < domain_builder.GetTotRanks() - 1; i++) {
        // Create the floor using fixed rigid body of 'box' type:
        auto mrigidFloor_sub = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
                                                                        1000,         // density
                                                                        true,         // visulization?
                                                                        true,         // collision?
                                                                        mat);         // contact material
        mrigidFloor_sub->SetPos(ChVector3d(0, -2, 0));
        mrigidFloor_sub->SetFixed(true);
        domain_builder.AddExcludedBody(mrigidFloor_sub);

        sys_slices[i]->Add(mrigidFloor_sub);
    }

    domain_manager.AddDomain(domain_builder.BuildMasterDomain(
        &sys_master  // physical system of the master domain (its rank automatically set to last domain+1)
        ));

    // For debugging: open two 3D realtime view windows, each per domain:

    auto vis_irr_0 = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr_0->AttachSystem(sys_slices[0]);
    vis_irr_0->SetWindowTitle("Domain 0");
    vis_irr_0->Initialize();
    vis_irr_0->AddSkyBox();
    vis_irr_0->AddCamera(ChVector3d(8, 16, 48), ChVector3d(0, 2, 0));
    vis_irr_0->AddTypicalLights();
    vis_irr_0->BindAll();
    // auto vis_irr_1 = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    // vis_irr_1->AttachSystem(sys_slices[1]);
    // vis_irr_1->SetWindowTitle("Domain 1");
    // vis_irr_1->Initialize();
    // vis_irr_1->AddSkyBox();
    // vis_irr_1->AddCamera(ChVector3d(8, 16, 48), ChVector3d(0, 2, 0));
    // vis_irr_1->AddTypicalLights();
    // vis_irr_1->BindAll();

    std::cout << "sys master total bodies init 0: " << sys_master.GetBodies().size() << std::endl;

    // 6 - INITIAL SETUP AND OBJECT INITIAL MIGRATION!
    //     Moves all the objects in master domain to all domains, slicing the system.
    //     Also does some initializations, like collision detection AABBs.
    domain_manager.DoAllDomainInitialize();

    // The master domain does not need to communicate anymore with the domains so do:
    domain_manager.master_domain_enabled = false;

    std::cout << "sys master total bodies init 1: " << sys_master.GetBodies().size() << std::endl;

    for (int i = 0; i < 800; ++i) {
        std::cout << "sys 0 total bodies: " << sys_slices[0]->GetBodies().size() << std::endl;
        std::cout << "sys 1 total bodies: " << sys_slices[1]->GetBodies().size() << std::endl;
        std::cout << "sys master total bodies: " << sys_master.GetBodies().size() << std::endl;

        std::cout << "\n\n\n============= Time step " << i << std::endl << std::endl;

        // For debugging: open two 3D realtime view windows, each per domain:
        // vis_irr_0->RemoveAllIrrNodes();
        vis_irr_0->BindAll();
        vis_irr_0->Run();
        vis_irr_0->BeginScene();
        vis_irr_0->Render();
        vis_irr_0->EndScene();
        vis_irr_0->RemoveAllIrrNodes();
        // vis_irr_1->BindAll();
        // vis_irr_1->Run();
        // vis_irr_1->BeginScene();
        // vis_irr_1->Render();
        // vis_irr_1->EndScene();

        std::cout << "test point 0 " << std::endl;

        // NOTE!!!!! The rebuild process will trigger severe impluse when domain starts switch for the multidomain PSOR
        // solver
        if (i % 10 == 0) {
            // BVH update
            domain_builder.RebuildDomains();
        } else {
            domain_builder.UpdateLocalDomainAABBs();
        }

        // MULTIDOMAIN AUTOMATIC ITEM MIGRATION!
        domain_manager.DoAllDomainPartitionUpdate();

        std::cout << "test point 2 " << std::endl;

        // // Fail safe mechanism to update all domain AABBs without running BVH

        // domain_builder.UpdateLocalDomainAABBs();

        // std::cout << "test point 3 " << std::endl;

        // // Time the MULTIDOMAIN AUTOMATIC ITEM MIGRATION!
        // auto start_time = std::chrono::high_resolution_clock::now();
        // domain_manager.DoAllDomainPartitionUpdate();
        // auto end_time = std::chrono::high_resolution_clock::now();

        // // Calculate duration in milliseconds
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // std::cout << "Domain partition update took " << duration.count() << " milliseconds" << std::endl;

        std::cout << "test point 4 " << std::endl;

        // Time the MULTIDOMAIN TIME INTEGRATION
        auto start_time_step = std::chrono::high_resolution_clock::now();
        domain_manager.DoAllStepDynamics(0.01);
        auto end_time_step = std::chrono::high_resolution_clock::now();

        std::cout << "test point 5 " << std::endl;

        // Calculate duration in microseconds
        auto duration_step = std::chrono::duration_cast<std::chrono::microseconds>(end_time_step - start_time_step);
        std::cout << "ZZH DoAllStepDynamics took " << duration_step.count() << " microseconds" << std::endl;
    }

    for (auto sys_i : sys_slices) {
        delete sys_i;
    }

    return 0;
}
