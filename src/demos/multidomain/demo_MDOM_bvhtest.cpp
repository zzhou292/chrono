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
// Authors:
// =============================================================================
//
// Test demo for BVH builder and domain tracking functionality
//
// =============================================================================

#include "chrono_multidomain/BVH_Builder/bvh_builder.hpp"
#include "chrono_multidomain/BVH_Builder/domain_tracker.hpp"
#include <iostream>

using namespace chrono;
using namespace multidomain;

// Helper function to print AABB information
void PrintAABB(const AABB& aabb, const std::string& name) {
    std::cout << name << " AABB: "
              << "min(" << aabb.min[0] << "," << aabb.min[1] << "," << aabb.min[2] << ") "
              << "max(" << aabb.max[0] << "," << aabb.max[1] << "," << aabb.max[2] << ")\n";
}

int main(int argc, char* argv[]) {
    std::cout << "Running BVH builder and domain tracker tests...\n";

    // Test 1: Basic BVH Construction
    std::cout << "\n=== Test 1: BVH Construction ===\n";
    std::vector<AABB> test_aabbs = {
        {{0, 0, 0}, {1, 1, 1}, {4}},       // Object 0
        {{2, 2, 2}, {3, 3, 3}, {5}},       // Object 1
        {{4, 4, 4}, {5, 5, 5}, {6}},       // Object 2
        {{6, 6, 6}, {7, 7, 7}, {7}},       // Object 3
        {{8, 8, 8}, {9, 9, 9}, {8}},       // Object 4
        {{10, 10, 10}, {11, 11, 11}, {9}}  // Object 5
    };

    BVHBuilder builder(test_aabbs);
    auto root = builder.build_top_down();

    // Verify root AABB
    AABB root_aabb = root->aabb;
    PrintAABB(root_aabb, "Root");
    if (root_aabb.min != std::array<double, 3>{0, 0, 0} || root_aabb.max != std::array<double, 3>{11, 11, 11} ||
        root_aabb.tags != std::vector<int>{4, 5, 6, 7, 8, 9}) {
        std::cout << "Test 1 FAILED: Root AABB incorrect!\n";
        // print all tags
        std::cout << "Root tags: ";
        for (int tag : root_aabb.tags)
            std::cout << tag << " ";
        std::cout << std::endl;
        return 1;
    }
    std::cout << "Test 1 PASSED: Root AABB is correct.\n";

    // Test 2: Domain Grouping
    std::cout << "\n=== Test 2: Domain Grouping ===\n";
    auto groups = builder.get_subdomains_greedy(root.get(), 3);  // Now create 3 domains instead of 2

    std::cout << "Domain groups:\n";
    for (size_t i = 0; i < groups.size(); i++) {
        std::cout << "Group " << i << " tags: ";
        for (int tag : groups[i].tags)
            std::cout << tag << " ";
        std::cout << std::endl;
    }

    if (groups.size() != 3 || groups[0].tags.size() + groups[1].tags.size() + groups[2].tags.size() != 6) {
        std::cout << "Test 2 FAILED: Invalid group partitioning!\n";
        return 1;
    }
    std::cout << "Test 2 PASSED: Domain grouping is correct.\n";

    // Test 3: Domain Tracking
    std::cout << "\n=== Test 3: Domain Tracking ===\n";
    DomainTracker tracker;

    // Get initial domain groups from the BVH
    auto initial_groups = builder.get_subdomains_greedy(root.get(), 3);

    // Track domains with the new API
    auto tracked_initial = tracker.match_domains(initial_groups);
    std::cout << "Initial tracking result:\n";
    for (size_t i = 0; i < tracked_initial.size(); ++i) {
        std::cout << "Tracked Group " << i << ": ";
        for (int tag : tracked_initial[i].tags)
            std::cout << tag << " ";
        std::cout << "\n";
    }

    // Simulate movement by updating some AABBs
    test_aabbs[0] = {{0.1, 0.1, 0.1}, {0.9, 0.9, 0.9}, {4}};
    test_aabbs[2] = {{4.1, 4.1, 4.1}, {4.9, 4.9, 4.9}, {6}};
    test_aabbs[4] = {{8.1, 8.1, 8.1}, {8.9, 8.9, 8.9}, {8}};

    // Rebuild BVH with moved objects
    BVHBuilder moved_builder(test_aabbs);
    auto moved_root = moved_builder.build_top_down();
    auto moved_groups = moved_builder.get_subdomains_greedy(moved_root.get(), 3);

    // Track domains after movement
    auto tracked_moved = tracker.match_domains(moved_groups);
    std::cout << "After movement tracking result:\n";
    for (size_t i = 0; i < tracked_moved.size(); ++i) {
        std::cout << "Tracked Group " << i << ": ";
        for (int tag : tracked_moved[i].tags)
            std::cout << tag << " ";
        std::cout << "\n";
    }

    // Verify that domain assignments are consistent
    bool consistent = true;
    if (tracked_initial.size() != tracked_moved.size()) {
        consistent = false;
    } else {
        for (size_t i = 0; i < tracked_initial.size(); ++i) {
            // Check if the same tags are in the same domains
            std::sort(tracked_initial[i].tags.begin(), tracked_initial[i].tags.end());
            std::sort(tracked_moved[i].tags.begin(), tracked_moved[i].tags.end());
            if (tracked_initial[i].tags != tracked_moved[i].tags) {
                consistent = false;
                break;
            }
        }
    }

    if (!consistent) {
        std::cout << "Test 3 FAILED: Group tracking inconsistent!\n";
        return 1;
    }

    std::cout << "Test 3 PASSED: Domain tracking maintained consistency after movement.\n";

    // Test 4: Domain Tracking with Different Domain Counts
    std::cout << "\n=== Test 4: Domain Tracking with Different Domain Counts ===\n";

    // Now try with 4 domains instead of 3
    auto groups_4domains = builder.get_subdomains_greedy(root.get(), 4);
    auto tracked_4domains = tracker.match_domains(groups_4domains);

    std::cout << "Tracking with 4 domains result:\n";
    for (size_t i = 0; i < tracked_4domains.size(); ++i) {
        std::cout << "Tracked Group " << i << ": ";
        for (int tag : tracked_4domains[i].tags)
            std::cout << tag << " ";
        std::cout << "\n";
    }

    // Verify that we have 4 domains now
    if (tracked_4domains.size() != 4) {
        std::cout << "Test 4 FAILED: Expected 4 domains but got " << tracked_4domains.size() << "!\n";
        return 1;
    }

    std::cout << "Test 4 PASSED: Domain tracking handled different domain counts correctly.\n";

    std::cout << "\nAll BVH tests passed successfully!\n";
    return 0;
}