#pragma once
#include "bvh_builder.hpp"
#include <Eigen/Dense>
#include <vector>

#include "chrono_multidomain/ChApiMultiDomain.h"
#include "chrono_multidomain/ChMpi.h"
#include "chrono_multidomain/ChDomainManager.h"

namespace chrono {
namespace multidomain {

class ChApiMultiDomain DomainTracker {
    std::vector<AABB> previous_domains;
    std::vector<int> assignment;
    std::vector<std::vector<double>> cost_matrix;

  public:
    // New API that directly accepts vector of AABBs
    std::vector<AABB> match_domains(const std::vector<AABB>& new_domains);

  private:
    static constexpr double HUNGARIAN_INFINITY = 1e15;

    void update(const std::vector<AABB>& new_domains);
    void hungarian_solve();
};

}  // namespace multidomain
}  // namespace chrono