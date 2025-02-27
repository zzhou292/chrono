#include "domain_tracker.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <limits>

using namespace Eigen;

namespace chrono {
namespace multidomain {

void DomainTracker::update(const std::vector<AABB>& new_domains) {
    cost_matrix.clear();
    cost_matrix.resize(previous_domains.size(), std::vector<double>(new_domains.size(), 0));

    for (size_t i = 0; i < previous_domains.size(); ++i) {
        for (size_t j = 0; j < new_domains.size(); ++j) {
            // Calculate overlap between domain AABBs
            float overlap = BVHBuilder::calc_overlap(previous_domains[i], new_domains[j]);

            // Use negative overlap because Hungarian algorithm minimizes cost
            cost_matrix[i][j] = -overlap;  // We want to maximize overlap
        }
    }
    hungarian_solve();
}

std::vector<AABB> DomainTracker::match_domains(const std::vector<AABB>& new_domains) {
    if (previous_domains.empty()) {
        previous_domains = new_domains;
        return new_domains;
    }

    update(new_domains);

    std::vector<AABB> ordered(previous_domains.size());

    for (size_t i = 0; i < assignment.size(); ++i) {
        if (assignment[i] != -1 && assignment[i] < (int)new_domains.size()) {
            ordered[i] = new_domains[assignment[i]];
        }
    }

    // Handle any new domains that weren't matched to previous ones
    for (size_t j = 0; j < new_domains.size(); ++j) {
        if (std::find(assignment.begin(), assignment.end(), j) == assignment.end()) {
            ordered.push_back(new_domains[j]);
        }
    }

    previous_domains = ordered;
    return ordered;
}

void DomainTracker::hungarian_solve() {
    int n = previous_domains.size();
    int m = cost_matrix[0].size();

    // Ensure we have valid dimensions
    if (n == 0 || m == 0) {
        assignment.clear();
        return;
    }

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

    assignment.resize(n, -1);  // Initialize all assignments to -1
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            assignment[p[j] - 1] = j - 1;
        }
    }
}
}  // namespace multidomain
}  // namespace chrono