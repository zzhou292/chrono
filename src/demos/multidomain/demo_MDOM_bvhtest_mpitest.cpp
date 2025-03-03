// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Your Name Here
// =============================================================================
//
// Test demo for MPI functionality in the multidomain module, specifically
// testing the GatherToMaster function
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_multidomain/ChDomainManagerMPI.h"
#include "chrono_multidomain/ChDomainBuilder.h"
#include "chrono_multidomain/BVH_Builder/bvh_builder.hpp"
#include "chrono_multidomain/ChMpi.h"

using namespace chrono;
using namespace multidomain;

// Helper function to print AABB information
void PrintAABB(const AABB& aabb, const std::string& prefix = "") {
    std::cout << prefix << "AABB: min=(" << aabb.min[0] << "," << aabb.min[1] << "," << aabb.min[2] << "), max=("
              << aabb.max[0] << "," << aabb.max[1] << "," << aabb.max[2] << "), tags=[";

    for (size_t j = 0; j < aabb.tags.size(); j++) {
        std::cout << aabb.tags[j];
        if (j < aabb.tags.size() - 1) {
            std::cout << ",";
        }
    }
    std::cout << "]" << std::endl;
}

// Test function for GatherToMaster
void TestGatherToMaster(ChDomainManagerMPI& domain_manager, std::shared_ptr<ChDomainBuilderBVHMPI> domain_builder) {
    int mpi_rank = domain_manager.GetMPIrank();
    int master_rank = domain_builder->GetMasterRank();
    int num_ranks = domain_builder->GetTotRanks();

    // Create some test AABBs with different data on each rank
    std::vector<AABB> local_aabbs;

    // Each rank creates different AABBs
    for (int i = 0; i < 3; i++) {
        // Create an AABB with position based on rank
        double offset = mpi_rank * 10.0;
        ChVector3d min(offset + i * 2.0, offset, offset);
        ChVector3d max(offset + i * 2.0 + 1.0, offset + 1.0, offset + 1.0);

        // Add tags based on rank and index
        std::vector<int> tags = {mpi_rank * 100 + i};

        local_aabbs.push_back(AABB(min, max, tags));
    }

    // Print local AABBs
    std::cout << "Rank " << mpi_rank << " local AABBs:" << std::endl;
    for (size_t i = 0; i < local_aabbs.size(); i++) {
        PrintAABB(local_aabbs[i], "  ");
    }

    // Gather all AABBs to the master rank
    std::vector<AABB> gathered_aabbs;
    ChMPI::GatherToMaster(local_aabbs, gathered_aabbs, master_rank);

    // Only the master rank will have all the AABBs
    if (mpi_rank == master_rank) {
        std::cout << "\nMaster rank gathered AABBs:" << std::endl;
        for (size_t i = 0; i < gathered_aabbs.size(); i++) {
            PrintAABB(gathered_aabbs[i], "  ");
        }

        // Verify the number of gathered AABBs
        int expected_count = 3 * num_ranks;  // 3 AABBs per rank
        if (gathered_aabbs.size() == expected_count) {
            std::cout << "\nTest PASSED: Gathered " << gathered_aabbs.size() << " AABBs (expected " << expected_count
                      << ")" << std::endl;
        } else {
            std::cout << "\nTest FAILED: Gathered " << gathered_aabbs.size() << " AABBs (expected " << expected_count
                      << ")" << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;
    std::cout << "Testing MPI functionality in the multidomain module" << std::endl;

    // Create the MPI-based domain manager
    ChDomainManagerMPI domain_manager(&argc, &argv);

    // This test assumes at least 2 processes
    if (domain_manager.GetMPItotranks() < 2) {
        std::cout << "This test requires at least 2 MPI processes. Please run with 'mpiexec -n 2 ...' or more."
                  << std::endl;
        return 1;
    }

    // For debugging/logging
    domain_manager.verbose_partition = false;
    domain_manager.verbose_serialization = false;
    domain_manager.verbose_variable_updates = false;

    // Create a system for each domain
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

    // Create the domain builder
    // Number of domains = total ranks - 1 (for master)
    auto domain_builder = chrono_types::make_shared<ChDomainBuilderBVHMPI>(domain_manager.GetMPItotranks() - 1, true);

    if (domain_manager.GetMPIrank() == 0) {
        std::cout << "\n=== Testing GatherToMaster function ===" << std::endl;
    }

    // Run the test
    TestGatherToMaster(domain_manager, domain_builder);

    if (domain_manager.GetMPIrank() == 0) {
        std::cout << "=== Test completed ===" << std::endl;
    }

    return 0;
}