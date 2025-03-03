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
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include <chrono>  // Add this at the top of your file if not already included

using namespace chrono;
using namespace multidomain;
using namespace chrono::irrlicht;
using namespace chrono::fea;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // 4- Create and populate the MASTER domain with bodies, links, meshes, nodes, etc.
    //    At the beginning of the simulation, the master domain will break into
    //    multiple data structures and will serialize them into the proper subdomains.
    //    (Note that there is also a "low level" version of this demo that shows how
    //    to bypass the use of the master domain, in case of extremely large systems
    //    where you might want to create objects directly split in the n-th computing node.)

    // First we create the master domain and add it to the domain manager:
    ChSystemNSC sys_master;
    sys_master.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    sys_master.GetSolver()->AsIterative()->SetMaxIterations(12);
    sys_master.GetSolver()->AsIterative()->SetTolerance(1e-6);

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
                                                                           100,   // density
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

    sys_master.Add(mrigidFloor);

    // For debugging: open two 3D realtime view windows, each per domain:

    auto vis_irr_0 = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr_0->AttachSystem(&sys_master);
    vis_irr_0->SetWindowTitle("Domain 0");
    vis_irr_0->Initialize();
    vis_irr_0->AddSkyBox();
    vis_irr_0->AddCamera(ChVector3d(8, 16, 48), ChVector3d(0, 2, 0));
    vis_irr_0->AddTypicalLights();
    vis_irr_0->BindAll();

    for (int i = 0; i < 1000; ++i) {
        std::cout << "\n\n\n============= Time step " << i << std::endl << std::endl;

        vis_irr_0->BindAll();
        vis_irr_0->Run();
        vis_irr_0->BeginScene();
        vis_irr_0->Render();
        vis_irr_0->EndScene();

        // Time the dynamics step
        auto start_time = std::chrono::high_resolution_clock::now();
        sys_master.DoStepDynamics(0.01);
        auto end_time = std::chrono::high_resolution_clock::now();

        // Calculate duration in microseconds
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        std::cout << "DoStepDynamics took " << duration.count() << " microseconds" << std::endl;
    }

    return 0;
}
