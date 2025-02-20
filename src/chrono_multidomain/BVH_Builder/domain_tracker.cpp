#include "domain_tracker.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <limits>

using namespace Eigen;

namespace chrono {
namespace multidomain {

void DomainTracker::update(const std::vector<std::vector<int>>& new_groups, const std::vector<const Node*>& new_nodes) {
    cost_matrix.clear();
    cost_matrix.resize(previous_groups.size(), std::vector<double>(new_groups.size(), 0));

    for (size_t i = 0; i < previous_groups.size(); ++i) {
        for (size_t j = 0; j < new_groups.size(); ++j) {
            // Calculate overlap between domain AABBs directly from nodes
            float overlap = BVHBuilder::calc_overlap(previous_nodes[i]->aabb, new_nodes[j]->aabb);

            // Use negative overlap because Hungarian algorithm minimizes cost
            cost_matrix[i][j] = -overlap;  // We want to maximize overlap
        }
    }
    hungarian_solve();
}

std::vector<std::vector<int>> DomainTracker::match_domains(const std::vector<std::vector<int>>& new_groups,
                                                           const std::vector<const Node*>& new_nodes) {
    if (previous_groups.empty()) {
        previous_groups = new_groups;
        previous_nodes = new_nodes;
        return new_groups;
    }

    update(new_groups, new_nodes);

    std::vector<std::vector<int>> ordered(new_groups.size());

    for (size_t i = 0; i < assignment.size(); ++i) {
        if (assignment[i] != -1 && assignment[i] < (int)new_groups.size()) {
            ordered[i] = new_groups[assignment[i]];
        }
    }

    for (size_t j = 0; j < new_groups.size(); ++j) {
        if (std::find(assignment.begin(), assignment.end(), j) == assignment.end()) {
            ordered.push_back(new_groups[j]);
        }
    }

    previous_groups = ordered;
    previous_nodes = new_nodes;
    return ordered;
}

void DomainTracker::hungarian_solve() {
    int n = previous_groups.size();
    int m = cost_matrix[0].size();

    std::vector<double> u(n + 1, 0);
    std::vector<double> v(m + 1, 0);
    std::vector<int> p(m + 1, 0);
    std::vector<int> way(m + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(m + 1, HUNGARIAN_INFINITY);
        std::vector<bool> used(m + 1, false);

        do {
            used[j0] = true;
            int i0 = p[j0];
            double delta = HUNGARIAN_INFINITY;
            int j1 = 0;

            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    double cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    assignment.resize(n);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            assignment[p[j] - 1] = j - 1;
        }
    }
}
}  // namespace multidomain
}  // namespace chrono