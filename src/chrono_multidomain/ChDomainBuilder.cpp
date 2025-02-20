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
// Authors: Alessandro Tasora
// =============================================================================

#include <array>
#include "chrono_multidomain/ChDomainBuilder.h"

namespace chrono {
namespace multidomain {

ChDomainBuilderSlices::ChDomainBuilderSlices(int tot_domains,
                                             double mmin,
                                             double mmax,
                                             ChAxis maxis,
                                             bool build_master) {
    this->axis = maxis;
    domains_bounds.push_back(-1e34);
    for (int islice = 0; islice < (tot_domains - 1); ++islice) {
        domains_bounds.push_back(mmin + (islice + 1.0) * (mmax - mmin) / (double)tot_domains);
    }
    domains_bounds.push_back(1e34);

    m_build_master = build_master;
};

ChDomainBuilderSlices::ChDomainBuilderSlices(std::vector<double> axis_cuts, ChAxis maxis, bool build_master) {
    this->axis = maxis;
    domains_bounds.push_back(-1e34);
    for (double dcut : axis_cuts) {
        domains_bounds.push_back(dcut);
    }
    domains_bounds.push_back(1e34);

    m_build_master = build_master;
}

std::shared_ptr<ChDomain> ChDomainBuilderSlices::BuildDomain(ChSystem* msys, int this_rank) {
    assert(this_rank >= 0);
    assert(this_rank < GetTotSlices());

    auto domain = chrono_types::make_shared<ChDomainSlice>(msys, this_rank, domains_bounds[this_rank],
                                                           domains_bounds[this_rank + 1], axis);

    if (this_rank > 0) {
        int low_domain_rank = this_rank - 1;
        auto low_domain = chrono_types::make_shared<ChDomainSlice>(
            nullptr, low_domain_rank, domains_bounds[this_rank - 1], domains_bounds[this_rank], axis);
        ChDomainInterface& low_interf =
            domain->GetInterfaces()[low_domain->GetRank()];  // insert domain interface in unordered map
        low_interf.side_IN = domain.get();
        low_interf.side_OUT = low_domain;
    }
    if (this_rank < domains_bounds.size() - 2) {
        int hi_domain_rank = this_rank + 1;
        auto hi_domain = chrono_types::make_shared<ChDomainSlice>(
            nullptr, hi_domain_rank, domains_bounds[this_rank + 1], domains_bounds[this_rank + 2], axis);
        ChDomainInterface& hi_interf =
            domain->GetInterfaces()[hi_domain->GetRank()];  // insert domain interface in unordered map
        hi_interf.side_IN = domain.get();
        hi_interf.side_OUT = hi_domain;
    }

    if (this->m_build_master) {
        int imaster_rank = this->GetTotRanks() - 1;
        auto master_dom = chrono_types::make_shared<ChDomainMaster>(nullptr, imaster_rank);
        ChDomainInterface& master_interf =
            domain->GetInterfaces()[imaster_rank];  // insert domain interface in unordered map
        master_interf.side_IN = domain.get();
        master_interf.side_OUT = master_dom;
    }

    return domain;
}

std::shared_ptr<ChDomain> ChDomainBuilderSlices::BuildMasterDomain(ChSystem* msys) {
    assert(this->m_build_master);
    int this_rank = this->GetTotRanks() - 1;

    auto domain = chrono_types::make_shared<ChDomainMaster>(msys, this_rank);

    for (int i = 0; i < this->GetTotSlices(); ++i) {
        int in_domain_rank = i;
        auto in_domain = chrono_types::make_shared<ChDomainSlice>(nullptr, in_domain_rank, domains_bounds[i],
                                                                  domains_bounds[i + 1], axis);
        ChDomainInterface& hi_interf =
            domain->GetInterfaces()[in_domain_rank];  // insert domain interface in unordered map
        hi_interf.side_IN = domain.get();
        hi_interf.side_OUT = in_domain;
    }

    return domain;
}

//////////////////////////////////////////////////////////////////////////////////////////

ChDomainBuilderGrid::ChDomainBuilderGrid(int tot_domains_X,
                                         double mmin_X,
                                         double mmax_X,
                                         int tot_domains_Y,
                                         double mmin_Y,
                                         double mmax_Y,
                                         int tot_domains_Z,
                                         double mmin_Z,
                                         double mmax_Z,
                                         bool build_master) {
    domains_bounds_X.push_back(-1e34);
    for (int islice = 0; islice < (tot_domains_X - 1); ++islice) {
        domains_bounds_X.push_back(mmin_X + (islice + 1.0) * (mmax_X - mmin_X) / (double)tot_domains_X);
    }
    domains_bounds_X.push_back(1e34);

    domains_bounds_Y.push_back(-1e34);
    for (int islice = 0; islice < (tot_domains_Y - 1); ++islice) {
        domains_bounds_Y.push_back(mmin_Y + (islice + 1.0) * (mmax_Y - mmin_Y) / (double)tot_domains_Y);
    }
    domains_bounds_Y.push_back(1e34);

    domains_bounds_Z.push_back(-1e34);
    for (int islice = 0; islice < (tot_domains_Z - 1); ++islice) {
        domains_bounds_Z.push_back(mmin_Z + (islice + 1.0) * (mmax_Z - mmin_Z) / (double)tot_domains_Z);
    }
    domains_bounds_Z.push_back(1e34);

    m_build_master = build_master;
};

ChDomainBuilderGrid::ChDomainBuilderGrid(std::vector<double> axis_cuts_X,
                                         std::vector<double> axis_cuts_Y,
                                         std::vector<double> axis_cuts_Z,
                                         bool build_master) {
    domains_bounds_X.push_back(-1e34);
    for (double dcut : axis_cuts_X) {
        domains_bounds_X.push_back(dcut);
    }
    domains_bounds_X.push_back(1e34);

    domains_bounds_Y.push_back(-1e34);
    for (double dcut : axis_cuts_Y) {
        domains_bounds_Y.push_back(dcut);
    }
    domains_bounds_Y.push_back(1e34);

    domains_bounds_Z.push_back(-1e34);
    for (double dcut : axis_cuts_Z) {
        domains_bounds_Z.push_back(dcut);
    }
    domains_bounds_Z.push_back(1e34);

    m_build_master = build_master;
}

std::shared_ptr<ChDomain> ChDomainBuilderGrid::BuildDomain(ChSystem* msys, int this_rank) {
    assert(this_rank >= 0);
    assert(this_rank < GetTotSlices());

    int i, j, k;
    this->FromRankToIJKindexes(this_rank, i, j, k);

    // Create this domain from rank number

    auto domain = chrono_types::make_shared<ChDomainBox>(
        msys, this_rank,
        ChAABB(ChVector3d(domains_bounds_X[i], domains_bounds_Y[j], domains_bounds_Z[k]),
               ChVector3d(domains_bounds_X[i + 1], domains_bounds_Y[j + 1], domains_bounds_Z[k + 1])));

    // Create the interfaces to 3x3=27 domains surrounding this domain, and the 27 domain placeholders

    for (int si = i - 1; si <= i + 1; ++si) {
        for (int sj = j - 1; sj <= j + 1; ++sj) {
            for (int sk = k - 1; sk <= k + 1; ++sk) {
                if (!((si == i) && (sj == j) && (sk == k))  // skip the center, i.e this domain
                    && (si >= 0) &&
                    (si < this->GetSlicesX())  // skip surrounding if this already at border of lattice (x dir)
                    && (sj >= 0) &&
                    (sj < this->GetSlicesY())  // skip surrounding if this already at border of lattice (y dir)
                    && (sk >= 0) &&
                    (sk < this->GetSlicesZ())  // skip surrounding if this already at border of lattice (z dir)
                ) {
                    int outer_domain_rank = this->FromIJKindexesToRank(si, sj, sk);
                    auto outer_domain = chrono_types::make_shared<ChDomainBox>(
                        nullptr, outer_domain_rank,
                        ChAABB(
                            ChVector3d(domains_bounds_X[si], domains_bounds_Y[sj], domains_bounds_Z[sk]),
                            ChVector3d(domains_bounds_X[si + 1], domains_bounds_Y[sj + 1], domains_bounds_Z[sk + 1])));

                    ChDomainInterface& low_interf =
                        domain->GetInterfaces()[outer_domain->GetRank()];  // insert domain interface in unordered map
                    low_interf.side_IN = domain.get();
                    low_interf.side_OUT = outer_domain;
                }
            }
        }
    }

    if (this->m_build_master) {
        int imaster_rank = this->GetTotRanks() - 1;
        auto master_dom = chrono_types::make_shared<ChDomainMaster>(nullptr, imaster_rank);
        ChDomainInterface& master_interf =
            domain->GetInterfaces()[imaster_rank];  // insert domain interface in unordered map
        master_interf.side_IN = domain.get();
        master_interf.side_OUT = master_dom;
    }

    return domain;
}

std::shared_ptr<ChDomain> ChDomainBuilderGrid::BuildMasterDomain(ChSystem* msys) {
    assert(this->m_build_master);
    int this_rank = this->GetTotRanks() - 1;

    // Create the master domain
    auto domain = chrono_types::make_shared<ChDomainMaster>(msys, this_rank);

    // Create interfaces from master to all N domains enclosed in master, and their N domain placeholders
    for (int in_domain_rank = 0; in_domain_rank < this->GetTotSlices(); ++in_domain_rank) {
        int si, sj, sk;
        this->FromRankToIJKindexes(in_domain_rank, si, sj, sk);
        auto in_domain = chrono_types::make_shared<ChDomainBox>(
            nullptr, in_domain_rank,
            ChAABB(ChVector3d(domains_bounds_X[si], domains_bounds_Y[sj], domains_bounds_Z[sk]),
                   ChVector3d(domains_bounds_X[si + 1], domains_bounds_Y[sj + 1], domains_bounds_Z[sk + 1])));
        ChDomainInterface& sub_interf =
            domain->GetInterfaces()[in_domain_rank];  // insert domain interface in unordered map
        sub_interf.side_IN = domain.get();
        sub_interf.side_OUT = in_domain;
    }

    return domain;
}

// =============================================================================

std::shared_ptr<ChDomain> ChDomainBuilderBVH::BuildDomain(ChSystem* msys, int this_rank) {
    assert(this_rank >= 0);
    assert(this_rank < num_domains_);

    // print domain_aabbs_
    std::cout << "HAHAHAHA RANK:" << this_rank << " domain_aabbs_.size(): " << domain_aabbs_.size() << std::endl;
    for (size_t i = 0; i < domain_aabbs_.size(); ++i) {
        std::cout << "HAHAHA Domain " << i << ": " << domain_aabbs_[i].min[0] << ", " << domain_aabbs_[i].min[1] << ", "
                  << domain_aabbs_[i].min[2] << " - " << domain_aabbs_[i].max[0] << ", " << domain_aabbs_[i].max[1]
                  << ", " << domain_aabbs_[i].max[2] << std::endl;
    }

    // Create domain with its assigned AABB
    auto domain = chrono_types::make_shared<ChDomainBox>(
        msys, this_rank,
        ChAABB(ChVector3d(domain_aabbs_[this_rank].min[0], domain_aabbs_[this_rank].min[1],
                          domain_aabbs_[this_rank].min[2]),
               ChVector3d(domain_aabbs_[this_rank].max[0], domain_aabbs_[this_rank].max[1],
                          domain_aabbs_[this_rank].max[2])));

    // Add interfaces to all other domains
    for (int i = 0; i < num_domains_; ++i) {
        if (i == this_rank)
            continue;  // Skip self

        auto other_dom = chrono_types::make_shared<ChDomainBox>(
            nullptr, i,
            ChAABB(ChVector3d(domain_aabbs_[i].min[0], domain_aabbs_[i].min[1], domain_aabbs_[i].min[2]),
                   ChVector3d(domain_aabbs_[i].max[0], domain_aabbs_[i].max[1], domain_aabbs_[i].max[2])));

        ChDomainInterface& interf = domain->GetInterfaces()[i];
        interf.side_IN = domain.get();
        interf.side_OUT = other_dom;
    }

    // Add master interface if enabled
    if (build_master_) {
        int imaster_rank = GetTotRanks() - 1;
        auto master_dom = chrono_types::make_shared<ChDomainMaster>(nullptr, imaster_rank);
        ChDomainInterface& master_interf = domain->GetInterfaces()[imaster_rank];
        master_interf.side_IN = domain.get();
        master_interf.side_OUT = master_dom;
    }

    current_domain_box_ = domain;

    return domain;
}

void ChDomainBuilderBVH::ComputeAndBroadcastDomainAABBs(ChSystem* msys, int mpi_rank) {
    if (mpi_rank == GetMasterRank()) {
        // First make sure collision system is up to date
        msys->GetCollisionSystem()->Initialize();
        msys->GetCollisionSystem()->Run();

        std::vector<AABB> aabbs;
        for (auto body : msys->GetBodies()) {
            if (std::find(excluded_bodies_.begin(), excluded_bodies_.end(), body) != excluded_bodies_.end()) {
                continue;
            }

            // Make sure body position is synced
            body->GetCollisionModel()->SyncPosition();

            // Now get the updated AABB
            ChAABB mabb = body->GetTotalAABB();
            aabbs.push_back(AABB(mabb.min, mabb.max, std::vector<int>{body->GetTag()}));
        }

        for (const auto& mmesh : msys->GetMeshes()) {
            ChAABB mabb = mmesh->GetTotalAABB();
            aabbs.push_back(AABB(mabb.min, mabb.max, std::vector<int>{mmesh->GetTag()}));
        }

        // Build BVH and compute domain assignments
        BVHBuilder builder(aabbs);
        auto root = builder.build_top_down();
        auto groups = builder.get_subdomains_greedy(root.get(), num_domains_);

        domain_aabbs_ = groups;

        // print out tag size for each domain
        std::cout << "Tag size for each domain: " << std::endl;
        for (size_t i = 0; i < domain_aabbs_.size(); ++i) {
            std::cout << "Domain " << i << ": " << domain_aabbs_[i].tags.size() << std::endl;
        }

        // expand x,y,z by 0.05
        for (size_t i = 0; i < domain_aabbs_.size(); ++i) {
            domain_aabbs_[i].min[0] -= 0.4;
            domain_aabbs_[i].min[1] -= 0.4;
            domain_aabbs_[i].min[2] -= 0.4;
            domain_aabbs_[i].max[0] += 0.4;
            domain_aabbs_[i].max[1] += 0.4;
            domain_aabbs_[i].max[2] += 0.4;
        }
    }

    std::cout << "Pre Broadcast:" << mpi_rank << std::endl;

    // Broadcast AABBs to all ranks
    ChMPI::ChBroadcast<AABB>(domain_aabbs_, GetMasterRank());

    std::cout << "Post Broadcast:" << mpi_rank << std::endl;

    std::cout << "rank: " << mpi_rank << " Received tag size: " << domain_aabbs_[mpi_rank].tags.size() << std::endl;
}

std::shared_ptr<ChDomain> ChDomainBuilderBVH::BuildMasterDomain(ChSystem* msys) {
    assert(this->build_master_);

    int this_rank = this->GetTotRanks() - 1;

    auto domain = chrono_types::make_shared<ChDomainMaster>(msys, this_rank);

    // Create interfaces from master to all domains
    for (int i = 0; i < num_domains_; ++i) {
        auto in_domain = chrono_types::make_shared<ChDomainBox>(
            nullptr, i,
            ChAABB(ChVector3d(domain_aabbs_[i].min[0], domain_aabbs_[i].min[1], domain_aabbs_[i].min[2]),
                   ChVector3d(domain_aabbs_[i].max[0], domain_aabbs_[i].max[1], domain_aabbs_[i].max[2])));

        ChDomainInterface& sub_interf = domain->GetInterfaces()[i];
        sub_interf.side_IN = domain.get();
        sub_interf.side_OUT = in_domain;
    }

    return domain;
}

// void ChDomainBuilderBVH::UpdateLocalDomainAABBs(ChSystem* msys, int mpi_rank) {
//     std::vector<AABB> broadcast_aabb(num_domains_);

//     if (mpi_rank != GetMasterRank()) {
//         // First make sure collision system is up to date
//         msys->GetCollisionSystem()->Initialize();
//         msys->GetCollisionSystem()->Run();

//         ChAABB total_aabb;
//         bool first = true;

//         // Get current domain
//         auto domain = std::dynamic_pointer_cast<ChDomainBox>(current_domain_box_);

//         std::cout << "pre of loop" << std::endl;

//         // // Process bodies that are either in domain or marked as overlapping
//         for (auto body : msys->GetBodies()) {
//             std::cout << "tp1" << std::endl;

//             // Skip excluded bodies
//             if (std::find(excluded_bodies_.begin(), excluded_bodies_.end(), body) != excluded_bodies_.end()) {
//                 continue;
//             }

//             std::cout << "tp2" << std::endl;

//             std::cout << "tag size: " << domain_aabbs_[mpi_rank].tags.size() << std::endl;

//             // Skip if the body's tag is not in the domain_aabbs_[mpi_rank].tags
//             if (std::find(domain_aabbs_[mpi_rank].tags.begin(), domain_aabbs_[mpi_rank].tags.end(), body->GetTag())
//             ==
//                 domain_aabbs_[mpi_rank].tags.end()) {
//                 continue;
//             }

//             std::cout << "tp3" << std::endl;

//             body->GetCollisionModel()->SyncPosition();
//             ChAABB mabb = body->GetTotalAABB();

//             std::cout << "tp4" << std::endl;

//             if (first) {
//                 total_aabb = mabb;
//                 first = false;
//             } else {
//                 total_aabb.Inflate(mabb);
//             }

//             std::cout << "tp5" << std::endl;
//         }

//         // for (const auto& mmesh : msys->GetMeshes()) {
//         //     ChAABB mabb = mmesh->GetTotalAABB();
//         //     if (first) {
//         //         total_aabb = mabb;
//         //         first = false;
//         //     } else {
//         //         total_aabb.Inflate(mabb);
//         //     }
//         // }

//         // domain_aabbs_[mpi_rank] =
//         //     AABB({total_aabb.min.x(), total_aabb.min.y(), total_aabb.min.z()},
//         //          {total_aabb.max.x(), total_aabb.max.y(), total_aabb.max.z()}, domain_aabbs_[mpi_rank].tags);
//         // broadcast_aabb[mpi_rank] = domain_aabbs_[mpi_rank];
//         // current_domain_box_->SetAABB(total_aabb);
//     }

//     std::cout << "out of loop" << std::endl;

//     // All ranks must participate in collective operation
//     // ChMPI::ChAllreduce<AABB>(broadcast_aabb, domain_aabbs_, ChMPI::ChOperation::SUM);
// }

}  // end namespace multidomain
}  // end namespace chrono
