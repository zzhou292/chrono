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
// Authors: Your Name
// =============================================================================
//
// Demo code for testing ChSystemDescriptorMultidomain's global vector operations
// with OpenMP
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/solver/ChVariables.h"

#include "chrono_multidomain/ChDomain.h"
#include "chrono_multidomain/ChDomainManagerSharedmemory.h"
#include "chrono_multidomain/ChSystemDescriptorMultidomain.h"
#include "chrono_multidomain/ChDomainBuilder.h"
#include "chrono_multidomain/ChSolverPSORmultidomain.h"

#include <cmath>
#include <iostream>
#include <omp.h>

using namespace chrono;
using namespace chrono::multidomain;

// Test case for validating globalVdot and globalVnorm functions with OpenMP
bool TestGlobalVectorOps() {
    bool test_passed = true;

    std::cout << "Creating domain manager..." << std::endl;
    // Create a domain manager with 2 domains for testing with shared memory
    ChDomainManagerSharedmemory domain_manager;
    domain_manager.serializer_type = DomainSerializerFormat::BINARY;
    domain_manager.verbose_partition = false;
    domain_manager.verbose_serialization = false;
    domain_manager.master_domain_enabled = false;

    std::cout << "Creating systems..." << std::endl;
    // Create systems (one for each domain)
    auto system1 = chrono_types::make_shared<ChSystemNSC>();
    auto system2 = chrono_types::make_shared<ChSystemNSC>();

    // Set system IDs
    system1->SetTag(1);
    system2->SetTag(2);

    // Create some bodies in each domain
    auto body1 = chrono_types::make_shared<ChBody>();
    body1->SetPos(ChVector3d(-2, 0, 0));
    body1->SetTag(100);  // Use a unique ID
    system1->Add(body1);

    auto body2 = chrono_types::make_shared<ChBody>();
    body2->SetPos(ChVector3d(2, 0, 0));
    body2->SetTag(200);  // Use a unique ID
    system2->Add(body2);

    std::cout << "Creating domains..." << std::endl;
    // Define bounding boxes for the domains
    ChAABB domain1_aabb(ChVector3d(-10, -10, -10), ChVector3d(0, 10, 10));
    ChAABB domain2_aabb(ChVector3d(0, -10, -10), ChVector3d(10, 10, 10));

    // Create domains
    auto domain1 = chrono_types::make_shared<ChDomainBox>(system1.get(), 0, domain1_aabb);
    auto domain2 = chrono_types::make_shared<ChDomainBox>(system2.get(), 1, domain2_aabb);

    std::cout << "Setting up domain descriptors..." << std::endl;
    // Create system descriptors with multidomain support
    auto multidomain_descriptor1 = chrono_types::make_shared<ChSystemDescriptorMultidomain>(domain1, &domain_manager);
    auto multidomain_descriptor2 = chrono_types::make_shared<ChSystemDescriptorMultidomain>(domain2, &domain_manager);

    // Set the descriptors for the systems
    system1->SetSystemDescriptor(multidomain_descriptor1);
    system2->SetSystemDescriptor(multidomain_descriptor2);

    // Set PSOR solvers for both systems
    auto solver1 = chrono_types::make_shared<ChSolverPSORmultidomain>();
    auto solver2 = chrono_types::make_shared<ChSolverPSORmultidomain>();
    system1->SetSolver(solver1);
    system2->SetSolver(solver2);

    // Enable coordinate weights for proper handling of shared nodes
    system1->EnableCoordWeightsWv(true);
    system2->EnableCoordWeightsWv(true);

    std::cout << "Setting up interfaces..." << std::endl;
    // Create domain interfaces
    ChDomainInterface interface12;
    interface12.side_IN = domain1.get();
    interface12.side_OUT = domain2;

    ChDomainInterface interface21;
    interface21.side_IN = domain2.get();
    interface21.side_OUT = domain1;

    // Add shared variables
    auto& body1_var = body1->Variables();
    auto& body2_var = body2->Variables();

    interface12.shared_vars.push_back(&body1_var);
    interface21.shared_vars.push_back(&body2_var);

    // Register interfaces
    domain1->GetInterfaces()[1] = interface12;
    domain2->GetInterfaces()[0] = interface21;

    std::cout << "Initializing systems..." << std::endl;
    // Initialize the systems
    system1->Setup();
    system2->Setup();

    std::cout << "Adding domains to manager..." << std::endl;
    // Add domains to the domain manager
    domain_manager.AddDomain(domain1);
    domain_manager.AddDomain(domain2);

    std::cout << "Domains size: " << domain_manager.domains.size() << std::endl;

    std::cout << "Initializing domains..." << std::endl;
    // Initialize all domains
    bool init_result = domain_manager.DoAllDomainInitialize();
    if (!init_result) {
        std::cout << "Error initializing domains!" << std::endl;
        return false;
    }

    // Get system descriptors
    auto descriptor1 = std::dynamic_pointer_cast<ChSystemDescriptorMultidomain>(system1->GetSystemDescriptor());
    auto descriptor2 = std::dynamic_pointer_cast<ChSystemDescriptorMultidomain>(system2->GetSystemDescriptor());

    if (!descriptor1 || !descriptor2) {
        std::cout << "ERROR: Failed to get valid descriptor pointers!" << std::endl;
        return false;
    }

    // Create test vectors (for bodies, we have 6 DOFs - 3 for position, 3 for rotation)
    // Make sure the vector size matches the DOF count
    int dof_count = body1->Variables().GetDOF();
    std::cout << "Body DOF count: " << dof_count << std::endl;

    ChVectorDynamic<> vec_a1(dof_count);
    vec_a1.setZero();
    for (int i = 0; i < std::min(dof_count, 6); i++)
        vec_a1(i) = i + 1.0;

    ChVectorDynamic<> vec_b1(dof_count);
    vec_b1.setZero();
    for (int i = 0; i < std::min(dof_count, 6); i++)
        vec_b1(i) = i + 2.0;

    ChVectorDynamic<> vec_a2(dof_count);
    vec_a2.setZero();
    for (int i = 0; i < std::min(dof_count, 6); i++)
        vec_a2(i) = i + 7.0;

    ChVectorDynamic<> vec_b2(dof_count);
    vec_b2.setZero();
    for (int i = 0; i < std::min(dof_count, 6); i++)
        vec_b2(i) = i + 8.0;

    // Create weight vectors
    ChVectorDynamic<> weight1(dof_count);
    weight1.setConstant(0.5);

    ChVectorDynamic<> weight2(dof_count);
    weight2.setConstant(0.5);

    // Compute expected results
    double expected_dot_no_weights = vec_a1.dot(vec_b1) + vec_a2.dot(vec_b2);
    double expected_dot_with_weights =
        vec_a1.dot(vec_b1.cwiseProduct(weight1)) + vec_a2.dot(vec_b2.cwiseProduct(weight2));

    double expected_norm_no_weights = std::sqrt(vec_a1.dot(vec_a1) + vec_a2.dot(vec_a2));
    double expected_norm_with_weights =
        std::sqrt(vec_a1.dot(vec_a1.cwiseProduct(weight1)) + vec_a2.dot(vec_a2.cwiseProduct(weight2)));

    // Set up OpenMP with exactly 2 threads
    omp_set_num_threads(2);
    std::cout << "Using 2 OpenMP threads for 2 domains" << std::endl;

    // Run tests one at a time without OpenMP first
    std::cout << "Testing without OpenMP..." << std::endl;

    double result_vdot1 = descriptor1->globalVdot(vec_a1, vec_b1);
    double result_vdot2 = descriptor2->globalVdot(vec_a2, vec_b2);

    std::cout << "globalVdot without weights from domain 1: " << result_vdot1 << std::endl;
    std::cout << "globalVdot without weights from domain 2: " << result_vdot2 << std::endl;
    std::cout << "Expected value: " << expected_dot_no_weights << std::endl;

    const double tolerance = 1e-10;

    // Now try with OpenMP if the previous tests passed
    std::cout << "Testing with OpenMP..." << std::endl;

    // Reset results
    result_vdot1 = 0.0;
    result_vdot2 = 0.0;

    // Test globalVdot without weights
    std::cout << "Testing globalVdot without weights..." << std::endl;
#pragma omp parallel num_threads(2)
    {
        int thread_id = omp_get_thread_num();
        std::cout << "Thread " << thread_id << " running" << std::endl;
        if (thread_id == 0) {
            result_vdot1 = descriptor1->globalVdot(vec_a1, vec_b1);
        } else if (thread_id == 1) {
            result_vdot2 = descriptor2->globalVdot(vec_a2, vec_b2);
        }
    }

    std::cout << "globalVdot without weights from domain 1: " << result_vdot1 << std::endl;
    std::cout << "globalVdot without weights from domain 2: " << result_vdot2 << std::endl;

    return test_passed;
}

int main(int argc, char* argv[]) {
    std::cout << "Testing ChSystemDescriptorMultidomain global vector operations with OpenMP..." << std::endl;

    // Display OpenMP information
    std::cout << "OpenMP version: " << _OPENMP << std::endl;
    std::cout << "Max OpenMP threads: " << omp_get_max_threads() << std::endl;

    bool test_result = TestGlobalVectorOps();

    if (test_result) {
        std::cout << "All tests PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "Some tests FAILED!" << std::endl;
        return 1;
    }
}