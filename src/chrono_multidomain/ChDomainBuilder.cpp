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
#include <set>
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

    // Add tags to the domain
    current_domain_box_->tags = domain_aabbs_[this_rank].tags;

    for (auto body : excluded_bodies_) {
        current_domain_box_->excluded_body_tags.push_back(body->GetTag());
    }

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

        auto matched_groups = domain_tracker_.match_domains(groups);

        domain_aabbs_ = matched_groups;
    }

    // TODO: we MUST NOT access the master domain aabb here.
    // Broadcast AABBs to all ranks
    ChMPI::ChBroadcast<AABB>(domain_aabbs_, GetMasterRank());
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

void ChDomainBuilderBVH::UpdateLocalDomainAABBs(ChSystem* msys, int mpi_rank) {
    // copy domain_aabbs_ to send_aabbs_synced_
    domain_aabbs_synced_.resize(num_domains_);

    for (int i = 0; i < num_domains_; ++i) {
        domain_aabbs_synced_[i] =
            ChAABB(ChVector3d(domain_aabbs_[i].min[0], domain_aabbs_[i].min[1], domain_aabbs_[i].min[2]),
                   ChVector3d(domain_aabbs_[i].max[0], domain_aabbs_[i].max[1], domain_aabbs_[i].max[2]));
    }

    // Create a temporary vector to hold the send buffer
    std::vector<ChAABB> send_aabbs_synced_ = domain_aabbs_synced_;

    if (mpi_rank != GetMasterRank()) {
        // First make sure collision system is up to date
        msys->GetCollisionSystem()->Initialize();
        msys->GetCollisionSystem()->Run();

        ChAABB total_aabb;
        bool first = true;

        // Get current domain
        auto domain = std::dynamic_pointer_cast<ChDomainBox>(current_domain_box_);

        // std::cout << "update local domain aabbs: " << mpi_rank
        //           << " number of tag size = " << current_domain_box_->tags.size() << std::endl;

        // Process bodies that are either in domain or marked as overlapping
        for (auto body : msys->GetBodies()) {
            // Skip excluded bodies
            if (std::find(excluded_bodies_.begin(), excluded_bodies_.end(), body) != excluded_bodies_.end()) {
                continue;
            }

            // Skip if the body's tag is not in the domain_aabbs_[mpi_rank].tags
            if (std::find(current_domain_box_->tags.begin(), current_domain_box_->tags.end(), body->GetTag()) ==
                current_domain_box_->tags.end()) {
                continue;
            }

            body->GetCollisionModel()->SyncPosition();
            ChAABB mabb = body->GetTotalAABB();

            if (first) {
                total_aabb = mabb;
                first = false;
            } else {
                total_aabb.Inflate(mabb);
            }
        }

        for (const auto& mmesh : msys->GetMeshes()) {
            ChAABB mabb = mmesh->GetTotalAABB();
            if (first) {
                total_aabb = mabb;
                first = false;
            } else {
                total_aabb.Inflate(mabb);
            }
        }

        send_aabbs_synced_[mpi_rank] = total_aabb;
    }

    // if (mpi_rank != GetMasterRank()) {
    //     std::cout << "rank " << mpi_rank << " aabb bounds: " << domain_aabbs_[mpi_rank].min[0] << " "
    //               << domain_aabbs_[mpi_rank].min[1] << " " << domain_aabbs_[mpi_rank].min[2] << " "
    //               << domain_aabbs_[mpi_rank].max[0] << " " << domain_aabbs_[mpi_rank].max[1] << " "
    //               << domain_aabbs_[mpi_rank].max[2] << std::endl;
    // }

    // Synchronize all AABBs across ranks using Allreduce
    ChMPI::ChAllreduce(send_aabbs_synced_, domain_aabbs_synced_, ChMPI::ChOperation::MAX);

    // Update the current domain box with the new AABB
    if (mpi_rank != GetMasterRank()) {
        current_domain_box_->SetAABB(
            ChAABB(ChVector3d(domain_aabbs_synced_[mpi_rank].min[0], domain_aabbs_synced_[mpi_rank].min[1],
                              domain_aabbs_synced_[mpi_rank].min[2]),
                   ChVector3d(domain_aabbs_synced_[mpi_rank].max[0], domain_aabbs_synced_[mpi_rank].max[1],
                              domain_aabbs_synced_[mpi_rank].max[2])));
    }

    // copy domain_aabbs_synced_ to domain_aabbs_
    for (int i = 0; i < num_domains_; ++i) {
        domain_aabbs_[i].max[0] = domain_aabbs_synced_[i].max[0];
        domain_aabbs_[i].max[1] = domain_aabbs_synced_[i].max[1];
        domain_aabbs_[i].max[2] = domain_aabbs_synced_[i].max[2];
        domain_aabbs_[i].min[0] = domain_aabbs_synced_[i].min[0];
        domain_aabbs_[i].min[1] = domain_aabbs_synced_[i].min[1];
        domain_aabbs_[i].min[2] = domain_aabbs_synced_[i].min[2];
    }

    // update all interfaces
    if (mpi_rank != GetMasterRank()) {
        for (int i = 0; i < num_domains_; ++i) {
            if (i == GetMasterRank() || i == mpi_rank)
                continue;

            auto& interf = current_domain_box_->GetInterfaces()[i];
            auto domain = std::dynamic_pointer_cast<ChDomainBox>(interf.side_OUT);
            domain->SetAABB(
                ChAABB(ChVector3d(domain_aabbs_[i].min[0], domain_aabbs_[i].min[1], domain_aabbs_[i].min[2]),
                       ChVector3d(domain_aabbs_[i].max[0], domain_aabbs_[i].max[1], domain_aabbs_[i].max[2])));
        }
    }
}

void ChDomainBuilderBVH::RebuildDomains(ChSystem* system, int mpi_rank) {
    // Step 1: Each rank collects updated AABBs for ALL objects it owns
    std::vector<AABB> updated_aabbs;

    // First make sure collision system is up to date
    system->GetCollisionSystem()->Initialize();
    system->GetCollisionSystem()->Run();

    if (mpi_rank != GetMasterRank()) {
        for (auto body : system->GetBodies()) {
            // Skip excluded bodies
            if (std::find(excluded_bodies_.begin(), excluded_bodies_.end(), body) != excluded_bodies_.end()) {
                continue;
            }

            // Skip if the body's tag is not in the domain_aabbs_[mpi_rank].tags
            if (std::find(current_domain_box_->tags.begin(), current_domain_box_->tags.end(), body->GetTag()) ==
                current_domain_box_->tags.end()) {
                continue;
            }

            body->GetCollisionModel()->SyncPosition();

            // Get the updated AABB
            ChAABB mabb = body->GetTotalAABB();
            updated_aabbs.push_back(AABB(mabb.min, mabb.max, std::vector<int>{body->GetTag()}));
        }
    }

    // if (mpi_rank != GetMasterRank()) {
    //     std::cout << "mpi_rank = " << mpi_rank << " updated_aabbs.size() = " << updated_aabbs.size() << std::endl;
    // }

    // Gather all AABBs to the master rank
    std::vector<AABB> all_updated_aabbs;
    ChMPI::GatherToMaster(updated_aabbs, all_updated_aabbs, GetMasterRank());

    // Only the master rank needs to process the gathered AABBs
    if (mpi_rank == GetMasterRank()) {
        // remove duplicates with the same tags within all updated aabbs
        // the tag for each aabb is aabb.tags[0]

        // Remove duplicates with the same tags within all updated aabbs
        // The tag for each aabb is aabb.tags[0]
        std::vector<AABB> unique_aabbs;
        std::unordered_set<int> processed_tags;

        for (const auto& aabb : all_updated_aabbs) {
            // Skip if we've already processed this tag
            if (aabb.tags.empty() || processed_tags.find(aabb.tags[0]) != processed_tags.end()) {
                continue;
            }

            // Add this tag to the set of processed tags
            processed_tags.insert(aabb.tags[0]);

            // Add this AABB to the unique list
            unique_aabbs.push_back(aabb);
        }

        // Replace the original vector with the unique one
        updated_aabbs = std::move(unique_aabbs);

        std::cout << "master received total unique aabbs = " << updated_aabbs.size() << std::endl;

        // Build BVH and compute domain assignments
        BVHBuilder builder(updated_aabbs);
        auto root = builder.build_top_down();
        auto groups = builder.get_subdomains_greedy(root.get(), num_domains_);

        domain_aabbs_ = groups;
    }

    // Broadcast AABBs to all ranks
    ChMPI::ChBroadcast<AABB>(domain_aabbs_, GetMasterRank());

    // if (mpi_rank != GetMasterRank()) {
    //     std::cout << "rank " << mpi_rank << " aabb bounds: " << domain_aabbs_[mpi_rank].min[0] << " "
    //               << domain_aabbs_[mpi_rank].min[1] << " " << domain_aabbs_[mpi_rank].min[2] << " "
    //               << domain_aabbs_[mpi_rank].max[0] << " " << domain_aabbs_[mpi_rank].max[1] << " "
    //               << domain_aabbs_[mpi_rank].max[2] << std::endl;
    // }

    // Update the current domain box with the new AABB
    if (mpi_rank != GetMasterRank()) {
        current_domain_box_->SetAABB(ChAABB(
            ChVector3d(domain_aabbs_[mpi_rank].min[0], domain_aabbs_[mpi_rank].min[1], domain_aabbs_[mpi_rank].min[2]),
            ChVector3d(domain_aabbs_[mpi_rank].max[0], domain_aabbs_[mpi_rank].max[1],
                       domain_aabbs_[mpi_rank].max[2])));

        current_domain_box_->tags = domain_aabbs_[mpi_rank].tags;

        // std::cout << "rank: " << mpi_rank << " reset tags size: " << current_domain_box_->tags.size() << std::endl;
        // std::cout << "rank: " << mpi_rank << " box size: " << domain_aabbs_[mpi_rank].min[0] << " "
        //           << domain_aabbs_[mpi_rank].min[1] << " " << domain_aabbs_[mpi_rank].min[2] << " "
        //           << domain_aabbs_[mpi_rank].max[0] << " " << domain_aabbs_[mpi_rank].max[1] << " "
        //           << domain_aabbs_[mpi_rank].max[2] << std::endl;
    }

    // update all interfaces
    if (mpi_rank != GetMasterRank()) {
        for (int i = 0; i < num_domains_; ++i) {
            if (i == GetMasterRank() || i == mpi_rank)
                continue;

            auto& interf = current_domain_box_->GetInterfaces()[i];
            auto domain = std::dynamic_pointer_cast<ChDomainBox>(interf.side_OUT);
            domain->SetAABB(
                ChAABB(ChVector3d(domain_aabbs_[i].min[0], domain_aabbs_[i].min[1], domain_aabbs_[i].min[2]),
                       ChVector3d(domain_aabbs_[i].max[0], domain_aabbs_[i].max[1], domain_aabbs_[i].max[2])));

            // std::cout << "rank: " << mpi_rank << " interface " << i << " box size: " << domain_aabbs_[i].min[0] << "
            // "
            //           << domain_aabbs_[i].min[1] << " " << domain_aabbs_[i].min[2] << " " << domain_aabbs_[i].max[0]
            //           << " " << domain_aabbs_[i].max[1] << " " << domain_aabbs_[i].max[2] << std::endl;
        }
    }
}

// JZ Async Testing Code

void ChDomainBuilderBVH::RebuildDomainsAsync(ChSystem* system, int mpi_rank, int current_step) {
    // Calculate whether to start a new decomposition on all ranks
    bool should_start_decomposition = false;

    // Only the master rank decides whether to start a decomposition
    if (mpi_rank == GetMasterRank()) {
        should_start_decomposition = (current_step % decomposition_step_interval_ == 0) &&
                                     (current_step != last_decomposition_step_) && !decomposition_in_progress_;
    }

    // Broadcast the decision to all ranks
    ChMPI::ChBroadcast(&should_start_decomposition, 1, GetMasterRank());

    // If we should start a decomposition, all ranks participate in gathering AABBs
    if (should_start_decomposition) {
        std::vector<AABB> updated_aabbs;
        std::vector<AABB> all_updated_aabbs;

        // First make sure collision system is up to date
        system->GetCollisionSystem()->Initialize();
        system->GetCollisionSystem()->Run();

        // Non-master ranks collect their AABBs
        if (mpi_rank != GetMasterRank()) {
            // Collect AABBs for bodies owned by this rank
            for (auto body : system->GetBodies()) {
                // Skip excluded bodies
                if (std::find(excluded_bodies_.begin(), excluded_bodies_.end(), body) != excluded_bodies_.end()) {
                    continue;
                }

                // Skip if the body's tag is not in this domain's tags
                if (std::find(current_domain_box_->tags.begin(), current_domain_box_->tags.end(), body->GetTag()) ==
                    current_domain_box_->tags.end()) {
                    continue;
                }

                body->GetCollisionModel()->SyncPosition();

                // Get the updated AABB
                ChAABB mabb = body->GetTotalAABB();
                updated_aabbs.push_back(AABB(mabb.min, mabb.max, std::vector<int>{body->GetTag()}));
            }
        }

        // Gather all AABBs to the master rank
        ChMPI::GatherToMaster(updated_aabbs, all_updated_aabbs, GetMasterRank());

        // Master rank starts the async decomposition
        if (mpi_rank == GetMasterRank()) {
            // Mark that decomposition is in progress
            decomposition_in_progress_ = true;
            last_decomposition_step_ = current_step;

            // Launch the decomposition asynchronously
            async_decomposition_future_ =
                std::async(std::launch::async, [this, all_updated_aabbs]() -> std::vector<AABB> {
                    // Process the gathered AABBs
                    std::vector<AABB> unique_aabbs;
                    std::unordered_set<int> processed_tags;

                    for (const auto& aabb : all_updated_aabbs) {
                        // Skip if we've already processed this tag
                        if (aabb.tags.empty() || processed_tags.find(aabb.tags[0]) != processed_tags.end()) {
                            continue;
                        }

                        // Add this tag to the set of processed tags
                        processed_tags.insert(aabb.tags[0]);

                        // Add this AABB to the unique list
                        unique_aabbs.push_back(aabb);
                    }

                    // Build BVH and compute domain assignments
                    BVHBuilder builder(unique_aabbs);
                    auto root = builder.build_top_down();
                    auto groups = builder.get_subdomains_greedy(root.get(), num_domains_);

                    std::cout << "Master completed async decomposition" << std::endl;
                    return groups;
                });
        }
    }

    // Check if a decomposition has completed
    if (decomposition_in_progress_ && mpi_rank == GetMasterRank()) {
        // Check if the future is ready without blocking
        if (async_decomposition_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            // Get the results
            std::vector<AABB> new_domain_aabbs = async_decomposition_future_.get();

            // Update the domain AABBs
            domain_aabbs_ = new_domain_aabbs;

            // Mark decomposition as complete
            decomposition_in_progress_ = false;

            std::cout << "Master ready to broadcast new domain decomposition" << std::endl;
        }
    }

    // Broadcast and apply new decomposition if available
    bool has_new_decomposition = !decomposition_in_progress_ && (last_decomposition_step_ >= 0);

    // Use MPI to broadcast whether there's a new decomposition
    // We need to use a pointer for the broadcast
    bool broadcast_flag = has_new_decomposition;
    ChMPI::ChBroadcast(&broadcast_flag, 1, GetMasterRank());

    if (broadcast_flag) {
        // Broadcast the new domain AABBs to all ranks
        ChMPI::ChBroadcast<AABB>(domain_aabbs_, GetMasterRank());

        // Apply the new decomposition to all ranks
        ApplyNewDecomposition(mpi_rank);

        // Set the partition update needed flag
        partition_update_needed_ = true;

        // Reset the last decomposition step to prevent reapplying
        if (mpi_rank == GetMasterRank()) {
            last_decomposition_step_ = -1;
        }
    }
}

void ChDomainBuilderBVH::ApplyNewDecomposition(int mpi_rank) {
    if (mpi_rank != GetMasterRank()) {
        // Update the current domain box with the new AABB
        current_domain_box_->SetAABB(ChAABB(
            ChVector3d(domain_aabbs_[mpi_rank].min[0], domain_aabbs_[mpi_rank].min[1], domain_aabbs_[mpi_rank].min[2]),
            ChVector3d(domain_aabbs_[mpi_rank].max[0], domain_aabbs_[mpi_rank].max[1],
                       domain_aabbs_[mpi_rank].max[2])));

        // Update tags
        current_domain_box_->tags = domain_aabbs_[mpi_rank].tags;

        std::cout << "Rank " << mpi_rank << " applied new domain decomposition" << std::endl;

        // Update all interfaces
        for (int i = 0; i < num_domains_; ++i) {
            if (i == GetMasterRank() || i == mpi_rank)
                continue;

            auto& interf = current_domain_box_->GetInterfaces()[i];
            auto domain = std::dynamic_pointer_cast<ChDomainBox>(interf.side_OUT);
            domain->SetAABB(
                ChAABB(ChVector3d(domain_aabbs_[i].min[0], domain_aabbs_[i].min[1], domain_aabbs_[i].min[2]),
                       ChVector3d(domain_aabbs_[i].max[0], domain_aabbs_[i].max[1], domain_aabbs_[i].max[2])));
        }
    }
}

void ChDomainBuilderBVH::SetDecompositionInterval(int step_interval) {
    decomposition_step_interval_ = step_interval;
}

}  // end namespace multidomain
}  // end namespace chrono
   //