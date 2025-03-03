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

#ifndef CHDOMAINBUILDER_H
#define CHDOMAINBUILDER_H

#include "chrono/physics/ChSystem.h"
#include "chrono_multidomain/ChApiMultiDomain.h"
#include "chrono_multidomain/ChDomainManager.h"
#include "chrono_multidomain/ChDomain.h"
#include "chrono_multidomain/BVH_Builder/bvh_builder.hpp"
#include "chrono_multidomain/BVH_Builder/domain_tracker.hpp"
#include <future>

namespace chrono {
namespace multidomain {

/// Base class for all domain builders, that is, helpers that setup N ChDomain objects
/// from some geometric splitting of the 3D space. Children classes must specialize this.
/// Be sure to call the BuildDomain() for all ranks, without leaving some domain rank not
/// built.

class ChApiMultiDomain ChDomainBuilder {
  public:
    ChDomainBuilder() {};
    virtual ~ChDomainBuilder() {}

    /// Get the number of ranks needed for this domain splitting
    virtual int GetTotRanks() = 0;

    /// If there is a master rank, for global coarse operations or other needs, return its rank
    /// (if master not supported or not enabled, children classes must break via assert)
    virtual int GetMasterRank() = 0;

  private:
};

/// Helper class that setup domains for a "sliced bread" splitting of the
/// 3D space. The splitting can happen along x or y or z axis, even in non uniform spacing.
/// The first and last domain will get infinite extent in the outside direction.

class ChApiMultiDomain ChDomainBuilderSlices : ChDomainBuilder {
  public:
    ChDomainBuilderSlices(int tot_slices, double mmin, double mmax, ChAxis maxis, bool build_master = false);

    ChDomainBuilderSlices(std::vector<double> axis_cuts, ChAxis maxis, bool build_master = false);

    virtual ~ChDomainBuilderSlices() {}

    virtual int GetTotRanks() override { return GetTotSlices() + (int)m_build_master; };

    virtual int GetMasterRank() override {
        assert(m_build_master);
        return GetTotSlices();
    };

    virtual int GetTotSlices() { return (int)(domains_bounds.size() - 1); };

    std::shared_ptr<ChDomain> BuildDomain(ChSystem* msys, int this_rank);

    /// Build the master domain that encloses everything, and that can be populated
    /// with all elements, nodes, bodies etc. at the beginning. It will migrate all items into
    /// the sliced domains at the first update. You need to create ChDomainBuilderSlices with build_master as true.
    /// The master domain, if used, corresponds to the last rank, ie. GetTotRanks()-1.
    std::shared_ptr<ChDomain> BuildMasterDomain(ChSystem* msys);

  private:
    std::vector<double> domains_bounds;
    ChAxis axis = ChAxis::X;
    bool m_build_master = false;
};

/// Specialization of ChDomain: sliced space. Used by ChDomainBuilderSlices.
/// Domain with infinite extent on two axes but with finite interval on a third axis.
/// Useful when slicing horizontally something like a river, or vertically something like
/// a tall stack.
class ChApiMultiDomain ChDomainSlice : public ChDomain {
  public:
    ChDomainSlice(ChSystem* msystem, int mrank, double mmin, double mmax, ChAxis maxis) : ChDomain(msystem, mrank) {
        min = mmin;
        max = mmax;
        axis = maxis;
    }
    virtual ~ChDomainSlice() {}

    /// Test if some item, with axis-aligned bounding box, must be included in this domain
    virtual bool IsOverlap(const ChAABB& abox) const override {
        switch (axis) {
            case ChAxis::X:
                if (min <= abox.max.x() && abox.min.x() < max)
                    return true;
                else
                    return false;
            case ChAxis::Y:
                if (min <= abox.max.y() && abox.min.y() < max)
                    return true;
                else
                    return false;
            case ChAxis::Z:
                if (min <= abox.max.z() && abox.min.z() < max)
                    return true;
                else
                    return false;
            default:
                return false;
        }
    }

    /// Test if some item, represented by a single point, must be included in this domain.
    virtual bool IsInto(const ChVector3d& apoint) const override {
        switch (axis) {
            case ChAxis::X:
                if (min <= apoint.x() && apoint.x() < max)
                    return true;
                else
                    return false;
            case ChAxis::Y:
                if (min <= apoint.y() && apoint.y() < max)
                    return true;
                else
                    return false;
            case ChAxis::Z:
                if (min <= apoint.z() && apoint.z() < max)
                    return true;
                else
                    return false;
            default:
                return false;
        }
    }

  private:
    double min = 0;
    double max = 0;
    ChAxis axis = ChAxis::X;
};

////////////////////////////

/// Helper class that setup domains for a "sliced bread" splitting of the
/// 3D space. The splitting can happen along x or y or z axis, even in non uniform spacing.
/// The first and last domain will get infinite extent in the outside direction.

class ChApiMultiDomain ChDomainBuilderGrid : ChDomainBuilder {
  public:
    ChDomainBuilderGrid(int tot_slices_X,
                        double mmin_X,
                        double mmax_X,
                        int tot_slices_Y,
                        double mmin_Y,
                        double mmax_Y,
                        int tot_slices_Z,
                        double mmin_Z,
                        double mmax_Z,
                        bool build_master = false);

    ChDomainBuilderGrid(std::vector<double> axis_cuts_X,
                        std::vector<double> axis_cuts_Y,
                        std::vector<double> axis_cuts_Z,
                        bool build_master = false);

    virtual ~ChDomainBuilderGrid() {}

    virtual int GetTotRanks() override { return GetTotSlices() + (int)m_build_master; };

    virtual int GetMasterRank() override {
        assert(m_build_master);
        return GetTotSlices();
    };

    virtual int GetTotSlices() {
        return (int)(domains_bounds_X.size() - 1) * (domains_bounds_Y.size() - 1) * (domains_bounds_Z.size() - 1);
    };

    virtual int GetSlicesX() { return (int)(domains_bounds_X.size() - 1); }
    virtual int GetSlicesY() { return (int)(domains_bounds_Y.size() - 1); }
    virtual int GetSlicesZ() { return (int)(domains_bounds_Z.size() - 1); }

    virtual int FromIJKindexesToRank(int i, int j, int k) {
        return i + j * GetSlicesX() + k * GetSlicesX() * GetSlicesY();
    }

    virtual void FromRankToIJKindexes(const int nrank, int& i, int& j, int& k) {
        k = nrank / (GetSlicesX() * GetSlicesY());
        j = (nrank % (GetSlicesX() * GetSlicesY())) / GetSlicesX();
        i = nrank % GetSlicesX();
    }

    std::shared_ptr<ChDomain> BuildDomain(ChSystem* msys, int this_rank);

    /// Build the master domain that encloses everything, and that can be populated
    /// with all elements, nodes, bodies etc. at the beginning. It will migrate all items into
    /// the sliced domains at the first update. You need to create ChDomainBuilderSlices with build_master as true.
    /// The master domain, if used, corresponds to the last rank, ie. GetTotRanks()-1.
    std::shared_ptr<ChDomain> BuildMasterDomain(ChSystem* msys);

  private:
    std::vector<double> domains_bounds_X;
    std::vector<double> domains_bounds_Y;
    std::vector<double> domains_bounds_Z;
    bool m_build_master = false;
};

//  JZ - ADDED ============================================================

class ChApiMultiDomain ChDomainBuilderBVHMPI : public ChDomainBuilder {
  public:
    ChDomainBuilderBVHMPI(int num_domains, bool build_master = true) : ChDomainBuilder() {
        build_master_ = build_master;
        num_domains_ = num_domains;
    }

    virtual int GetTotRanks() override { return num_domains_ + (int)build_master_; }
    virtual int GetMasterRank() override { return GetTotRanks() - 1; }

    void ComputeAndBroadcastDomainAABBs(ChSystem* msys, int mpi_rank);

    virtual std::shared_ptr<ChDomain> BuildDomain(ChSystem* msys, int this_rank);

    /// Build the master domain that encloses everything, and that can be populated
    /// with all elements, nodes, bodies etc. at the beginning. It will migrate all items into
    /// the domains at the first update. You need to create ChDomainBuilderBVH with build_master as true.
    /// The master domain, if used, corresponds to the last rank, ie. GetTotRanks()-1.
    std::shared_ptr<ChDomain> BuildMasterDomain(ChSystem* msys);

    void UpdateLocalDomainAABBs(ChSystem* msys, int mpi_rank);

    void AddExcludedBody(std::shared_ptr<ChBody> body) { excluded_bodies_.push_back(body); }

    // Rebuild domains based on current object positions
    void RebuildDomains(ChSystem* msys, int mpi_rank);

    void ApplyNewDecomposition(int mpi_rank);

  private:
    std::vector<AABB> domain_aabbs_;           // aabb with tags
    std::vector<ChAABB> domain_aabbs_synced_;  // a aabb copy without any tags
    int num_domains_;                          // this excludes the master domain
    bool build_master_;
    std::vector<std::shared_ptr<ChBody>> excluded_bodies_;
    std::shared_ptr<ChDomainBox> current_domain_box_;

    // domain tracker
    DomainTracker domain_tracker_;
};

/// Helper class that setup domains using Bounding Volume Hierarchy (BVH) for OpenMP parallelization.
/// This class creates domains based on a BVH decomposition of the simulation space.
/// It supports dynamic domain rebuilding based on object positions.
class ChApiMultiDomain ChDomainBuilderBVHOMP : public ChDomainBuilder {
  public:
    ChDomainBuilderBVHOMP(int num_domains, bool build_master = true) : ChDomainBuilder() {
        build_master_ = build_master;
        num_domains_ = num_domains;
        domain_aabbs_.resize(num_domains);
        domain_aabbs_synced_.resize(num_domains);
        domains_.resize(num_domains);
    }

    virtual int GetTotRanks() override { return num_domains_ + (int)build_master_; }
    virtual int GetMasterRank() override {
        assert(build_master_);
        return GetTotRanks() - 1;
    }

    void ComputeAndBroadcastDomainAABBs(ChSystem* msys);

    virtual std::shared_ptr<ChDomain> BuildDomain(ChSystem* msys, int this_rank);

    /// Build the master domain that encloses everything, and that can be populated
    /// with all elements, nodes, bodies etc. at the beginning. It will migrate all items into
    /// the domains at the first update. You need to create ChDomainBuilderBVHOMP with build_master as true.
    /// The master domain, if used, corresponds to the last rank, ie. GetTotRanks()-1.
    std::shared_ptr<ChDomain> BuildMasterDomain(ChSystem* msys);

    void UpdateLocalDomainAABBs();

    void AddExcludedBody(std::shared_ptr<ChBody> body) { excluded_bodies_.push_back(body); }

    // Rebuild domains based on current object positions
    void RebuildDomains();

    void ApplyNewDecomposition();

    // Get a specific domain
    std::shared_ptr<ChDomainBox> GetDomain(int rank) {
        if (rank >= 0 && rank < domains_.size())
            return domains_[rank];
        return nullptr;
    }

  private:
    std::vector<AABB> domain_aabbs_;           // aabb with tags
    std::vector<ChAABB> domain_aabbs_synced_;  // a aabb copy without any tags
    int num_domains_;
    bool build_master_;
    std::vector<std::shared_ptr<ChBody>> excluded_bodies_;
    // Store all domains for OpenMP version
    // Note that we declare and initialize the master domain here at index len-1
    // so if we have 4 domains(excluding the master), we have 5 ChDomainBox
    std::vector<std::shared_ptr<ChDomainBox>> domains_;

    // domain tracker
    DomainTracker domain_tracker_;
};

}  // end namespace multidomain
}  // end namespace chrono

#endif
