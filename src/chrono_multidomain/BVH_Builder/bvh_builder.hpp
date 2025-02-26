#pragma once
#include <algorithm>
#include <array>
#include <memory>
#include <queue>
#include <stack>
#include <string>
#include <vector>

#include "chrono_multidomain/ChApiMultiDomain.h"
#include "chrono_multidomain/ChMpi.h"
#include "chrono_multidomain/ChDomainManager.h"

namespace chrono {
namespace multidomain {

struct ChApiMultiDomain AABB {
    std::array<double, 3> min;
    std::array<double, 3> max;

    std::vector<int> tags;

    AABB() = default;

    AABB(const ChVector3d& min_vec, const ChVector3d& max_vec, const std::vector<int>& tags) {
        min = {min_vec.x(), min_vec.y(), min_vec.z()};
        max = {max_vec.x(), max_vec.y(), max_vec.z()};
        this->tags = tags;
    }

    bool operator!=(const AABB& other) const { return min != other.min || max != other.max; }
};

// this is a AABB class with fixed size array for tags
template <size_t N>
struct ChApiMultiDomain AABB_static {
    std::array<double, 3> min;
    std::array<double, 3> max;
    int tag_size;
    std::array<int, N> tags;

    AABB_static() = default;

    AABB_static(const ChVector3d& min_vec, const ChVector3d& max_vec, int tag_size, const std::array<int, N>& tags) {
        min = {min_vec.x(), min_vec.y(), min_vec.z()};
        max = {max_vec.x(), max_vec.y(), max_vec.z()};
        this->tag_size = tag_size;
        std::copy_n(tags.begin(), N, this->tags.begin());
    }

    bool operator!=(const AABB_static& other) const {
        return min != other.min || max != other.max || tags != other.tags || tag_size != other.tag_size;
    }
};

class ChApiMultiDomain Node {
  public:
    AABB aabb;
    std::unique_ptr<Node> left;
    std::unique_ptr<Node> right;
    bool is_leaf;
    int obj_index;
    int leaf_count;

    Node(AABB box, bool leaf = false, int idx = -1) : aabb(box), is_leaf(leaf), obj_index(idx), leaf_count(1) {}
};

class ChApiMultiDomain BVHBuilder {
  public:
    explicit BVHBuilder(const std::vector<AABB>& aabbs) : aabbs(aabbs) {}

    std::unique_ptr<Node> build_top_down();
    std::unique_ptr<Node> update_incremental(std::unique_ptr<Node> root, double threshold = 0.3f);
    std::vector<AABB> get_subdomains_greedy(const Node* root, int num_groups);
    std::vector<const Node*> get_group_nodes(const Node* root, const std::vector<std::vector<int>>& groups);

    static double aabb_volume(const AABB& aabb);
    static AABB merge_aabbs(const AABB& a, const AABB& b);
    static double calc_overlap(const AABB& a, const AABB& b);

  private:
    const std::vector<AABB>& aabbs;  // Pairs of (AABB, tag)

    struct BuildNode {
        AABB aabb;
        int start;
        int end;
    };

    struct HeapNode {
        int negative_leaf_count;
        int counter;
        const Node* node;

        bool operator>(const HeapNode& other) const {
            if (negative_leaf_count != other.negative_leaf_count)
                return negative_leaf_count > other.negative_leaf_count;
            return counter > other.counter;
        }
    };

    std::unique_ptr<Node> recursive_build(BuildNode bnode);
    void refit(Node* node, std::vector<bool>& updated);
    void print_tree_structure(const Node* node, std::string prefix = "", bool isLeft = false) const;
};

}  // namespace multidomain
}  // namespace chrono