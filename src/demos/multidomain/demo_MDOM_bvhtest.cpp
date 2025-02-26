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
        {{0, 0, 0}, {1, 1, 1}, {4}},  // Object 0
        {{2, 2, 2}, {3, 3, 3}, {5}},  // Object 1
        {{4, 4, 4}, {5, 5, 5}, {6}}   // Object 2
    };

    BVHBuilder builder(test_aabbs);
    auto root = builder.build_top_down();

    // Verify root AABB
    AABB root_aabb = root->aabb;
    PrintAABB(root_aabb, "Root");
    if (root_aabb.min != std::array<double, 3>{0, 0, 0} || root_aabb.max != std::array<double, 3>{5, 5, 5} ||
        root_aabb.tags != std::vector<int>{4, 5, 6}) {
        std::cout << "Test 1 FAILED: Root AABB incorrect!\n";
        // print all tags
        std::cout << "Root tags: ";
        for (int tag : root_aabb.tags)
            std::cout << tag << " ";
        std::cout << std::endl;
        return 1;
    }

    // Test 2: Domain Grouping
    std::cout << "\n=== Test 2: Domain Grouping ===\n";
    auto groups = builder.get_subdomains_greedy(root.get(), 2);

    std::cout << "Domain groups:\n";
    for (const auto& aabb : groups) {
        std::cout << "group tags: ";
        for (int tag : aabb.tags)
            std::cout << tag << " ";
        std::cout << std::endl;
    }

    if (groups.size() != 2 || groups[0].tags.size() + groups[1].tags.size() != 3) {
        std::cout << "Test 2 FAILED: Invalid group partitioning!\n";
        return 1;
    }

    // // Test 3: Domain Tracking
    // std::cout << "\n=== Test 3: Domain Tracking ===\n";
    // DomainTracker tracker;
    // std::vector<const Node*> initial_nodes;

    // // Collect leaf nodes
    // std::queue<const Node*> q;
    // q.push(root.get());
    // while (!q.empty()) {
    //     auto node = q.front();
    //     q.pop();
    //     if (node->is_leaf) {
    //         initial_nodes.push_back(node);
    //     } else {
    //         q.push(node->left.get());
    //         q.push(node->right.get());
    //     }
    // }

    // auto initial_groups = tracker.match_domains(groups, initial_nodes);
    // std::cout << "Initial tracking result:\n";
    // for (size_t i = 0; i < initial_groups.size(); ++i) {
    //     std::cout << "Tracked Group " << i << ": ";
    //     for (auto idx : initial_groups[i])
    //         std::cout << idx << " ";
    //     std::cout << "\n";
    // }

    // // Simulate movement
    // test_aabbs[0] = {{0.1, 0.1, 0.1}, {0.9, 0.9, 0.9}, {4}};
    // test_aabbs[2] = {{4.1, 4.1, 4.1}, {4.9, 4.9, 4.9}, {6}};

    // BVHBuilder moved_builder(test_aabbs);
    // auto moved_root = moved_builder.build_top_down();
    // auto moved_groups = moved_builder.get_subdomains_greedy(moved_root.get(), 2);

    // std::vector<const Node*> moved_nodes;
    // q.push(moved_root.get());
    // while (!q.empty()) {
    //     auto node = q.front();
    //     q.pop();
    //     if (node->is_leaf) {
    //         moved_nodes.push_back(node);
    //     } else {
    //         q.push(node->left.get());
    //         q.push(node->right.get());
    //     }
    // }

    // auto tracked_groups = tracker.match_domains(moved_groups, moved_nodes);
    // std::cout << "After movement tracking result:\n";
    // for (size_t i = 0; i < tracked_groups.size(); ++i) {
    //     std::cout << "Tracked Group " << i << ": ";
    //     for (auto idx : tracked_groups[i])
    //         std::cout << idx << " ";
    //     std::cout << "\n";
    // }

    // if (tracked_groups != initial_groups) {
    //     std::cout << "Test 3 FAILED: Group tracking inconsistent!\n";
    //     return 1;
    // }

    std::cout << "\nAll BVH tests passed successfully!\n";
    return 0;
}