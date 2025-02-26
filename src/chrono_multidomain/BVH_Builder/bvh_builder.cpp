#include "bvh_builder.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <queue>
#include <stack>

namespace chrono {
namespace multidomain {

std::unique_ptr<Node> BVHBuilder::build_top_down() {
    if (aabbs.empty())
        return nullptr;

    std::vector<std::pair<AABB, int>> indexed_aabbs;
    for (size_t i = 0; i < aabbs.size(); ++i) {
        indexed_aabbs.emplace_back(aabbs[i], static_cast<int>(i));
    }

    std::queue<std::tuple<std::vector<std::pair<AABB, int>>, Node*, bool>> q;
    std::unique_ptr<Node> root;
    q.emplace(indexed_aabbs, nullptr, false);

    while (!q.empty()) {
        auto [items, parent, is_left] = q.front();
        q.pop();

        std::unique_ptr<Node> node;
        if (items.size() == 1) {
            const auto& [aabb, idx] = items[0];
            node = std::make_unique<Node>(aabb, true, idx);
        } else {
            AABB combined = items[0].first;
            for (const auto& [aabb, _] : items) {
                combined = merge_aabbs(combined, aabb);
            }

            auto [min_pt, max_pt, tags] = combined;
            std::array<double, 3> axis_lengths = {max_pt[0] - min_pt[0], max_pt[1] - min_pt[1], max_pt[2] - min_pt[2]};
            int split_axis = static_cast<int>(
                std::distance(axis_lengths.begin(), std::max_element(axis_lengths.begin(), axis_lengths.end())));

            std::sort(items.begin(), items.end(), [split_axis](const auto& a, const auto& b) {
                double a_center = (a.first.min[split_axis] + a.first.max[split_axis]) / 2;
                double b_center = (b.first.min[split_axis] + b.first.max[split_axis]) / 2;
                return a_center < b_center;
            });

            size_t split_idx = std::max<size_t>(1, std::min<size_t>(items.size() - 1, items.size() / 2));
            std::vector<std::pair<AABB, int>> left_items(items.begin(), items.begin() + split_idx);
            std::vector<std::pair<AABB, int>> right_items(items.begin() + split_idx, items.end());

            node = std::make_unique<Node>(combined);
            q.emplace(left_items, node.get(), true);
            q.emplace(right_items, node.get(), false);
        }

        if (parent) {
            if (is_left)
                parent->left = std::move(node);
            else
                parent->right = std::move(node);
        } else {
            root = std::move(node);
        }
    }

    // Set leaf count for internal nodes
    if (!root->is_leaf) {
        root->leaf_count = 0;
        if (root->left)
            root->leaf_count += root->left->leaf_count;
        if (root->right)
            root->leaf_count += root->right->leaf_count;
    }

    return root;
}

double BVHBuilder::aabb_volume(const AABB& aabb) {
    double volume = 1.0f;
    for (int i = 0; i < 3; ++i) {
        volume *= (aabb.max[i] - aabb.min[i]);
    }
    return volume;
}

AABB BVHBuilder::merge_aabbs(const AABB& a, const AABB& b) {
    AABB merged;
    for (int i = 0; i < 3; ++i) {
        merged.min[i] = std::min(a.min[i], b.min[i]);
        merged.max[i] = std::max(a.max[i], b.max[i]);
    }
    merged.tags = a.tags;
    merged.tags.insert(merged.tags.end(), b.tags.begin(), b.tags.end());

    // remove duplicates
    std::sort(merged.tags.begin(), merged.tags.end());
    merged.tags.erase(std::unique(merged.tags.begin(), merged.tags.end()), merged.tags.end());

    return merged;
}

double BVHBuilder::calc_overlap(const AABB& a, const AABB& b) {
    double overlap = 1.0f;
    for (int i = 0; i < 3; ++i) {
        double min_max = std::min(a.max[i], b.max[i]);
        double max_min = std::max(a.min[i], b.min[i]);
        if (min_max < max_min)
            return 0.0f;
        overlap *= (min_max - max_min);
    }
    return overlap;
}

// NOTE, this function returns combined AABBs!!!!
std::vector<AABB> BVHBuilder::get_subdomains_greedy(const Node* root, int num_groups) {
    std::cout << "\nStarting greedy subdomain decomposition..." << std::endl;
    std::vector<AABB> groups;
    if (!root)
        return groups;

    // Create priority queue (max heap)
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<HeapNode>> heap;
    int counter = 0;

    // Push root node
    heap.push({-root->leaf_count, counter++, root});

    // Split largest nodes until we reach target count or can't split anymore
    while (heap.size() < static_cast<size_t>(num_groups) && !heap.empty()) {
        auto current = heap.top();
        heap.pop();

        std::cout << "Processing node with " << -current.negative_leaf_count << " leaves" << std::endl;

        if (current.node->is_leaf) {
            heap.push(current);
            continue;  // Continue instead of break to process other nodes
        }

        // Push children with their leaf counts
        if (current.node->left) {
            heap.push({-current.node->left->leaf_count, counter++, current.node->left.get()});
        }
        if (current.node->right) {
            heap.push({-current.node->right->leaf_count, counter++, current.node->right.get()});
        }
    }

    // Collect all leaves under each node in the heap
    std::cout << "Collecting leaves from " << heap.size() << " groups" << std::endl;
    while (!heap.empty()) {
        auto current = heap.top();
        heap.pop();

        // The node's AABB already contains all tags from its children
        groups.push_back(current.node->aabb);
    }

    // Fill remaining groups with empty vectors if we couldn't reach target count
    while (groups.size() < static_cast<size_t>(num_groups)) {
        groups.push_back(AABB());
        std::cout << "Added empty group to reach target count" << std::endl;
    }

    std::cout << "Final number of groups: " << groups.size() << std::endl;
    return groups;
}

void BVHBuilder::print_tree_structure(const Node* node, std::string prefix, bool isLeft) const {
    if (!node)
        return;

    std::cout << prefix;
    std::cout << (isLeft ? "├── " : "└── ");

    if (node->is_leaf) {
        std::cout << "Leaf(idx=" << node->obj_index << ")" << std::endl;
    } else {
        std::cout << "Node" << std::endl;
    }

    print_tree_structure(node->left.get(), prefix + (isLeft ? "│   " : "    "), true);
    print_tree_structure(node->right.get(), prefix + (isLeft ? "│   " : "    "), false);
}

std::unique_ptr<Node> BVHBuilder::update_incremental(std::unique_ptr<Node> root, double threshold) {
    if (!root)
        return build_top_down();

    std::vector<bool> updated(aabbs.size(), false);
    refit(root.get(), updated);

    // If too many nodes were updated, rebuild the entire tree
    int update_count = std::count(updated.begin(), updated.end(), true);
    if (static_cast<double>(update_count) / aabbs.size() > threshold) {
        return build_top_down();
    }

    return root;
}

void BVHBuilder::refit(Node* node, std::vector<bool>& updated) {
    if (!node)
        return;

    if (node->is_leaf) {
        AABB new_aabb = aabbs[node->obj_index];
        if (new_aabb != node->aabb) {
            node->aabb = new_aabb;
            updated[node->obj_index] = true;
        }
    } else {
        refit(node->left.get(), updated);
        refit(node->right.get(), updated);

        // Update internal node's AABB
        if (node->left && node->right) {
            node->aabb = merge_aabbs(node->left->aabb, node->right->aabb);
        } else if (node->left) {
            node->aabb = node->left->aabb;
        } else if (node->right) {
            node->aabb = node->right->aabb;
        }
    }

    // Set leaf count for internal nodes
    if (!node->is_leaf) {
        node->leaf_count = 0;
        if (node->left)
            node->leaf_count += node->left->leaf_count;
        if (node->right)
            node->leaf_count += node->right->leaf_count;
    }
}

std::vector<const Node*> BVHBuilder::get_group_nodes(const Node* root, const std::vector<std::vector<int>>& groups) {
    std::vector<const Node*> group_nodes;
    for (const auto& group : groups) {
        // Create a temporary node for the group
        AABB group_aabb = aabbs[group[0]];
        for (size_t i = 1; i < group.size(); ++i) {
            group_aabb = merge_aabbs(group_aabb, aabbs[group[i]]);
        }
        group_nodes.push_back(new Node(group_aabb));
    }
    return group_nodes;
}

}  // namespace multidomain
}  // namespace chrono
