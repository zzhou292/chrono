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
    ChDomainBuilder(){};
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

/// Specialization of ChDomain for axis-aligned grid space decomposition.
/// This is like a box.
///
class ChApiMultiDomain ChDomainBox : public ChDomain {
  public:
    ChDomainBox(ChSystem* msystem, int mrank, ChAABB domain_aabb) : ChDomain(msystem, mrank) { aabb = domain_aabb; }
    virtual ~ChDomainBox() {}

    /// Test if some item, with axis-aligned bounding box, must be included in this domain
    virtual bool IsOverlap(const ChAABB& abox) const override {
        if ((aabb.min.x() <= abox.max.x() && abox.min.x() < aabb.max.x()) &&
            (aabb.min.y() <= abox.max.y() && abox.min.y() < aabb.max.y()) &&
            (aabb.min.z() <= abox.max.z() && abox.min.z() < aabb.max.z()))
            return true;
        else
            return false;
    }

    /// Test if some item, represented by a single point, must be included in this domain.
    virtual bool IsInto(const ChVector3d& apoint) const override {
        if ((aabb.min.x() <= apoint.x() && apoint.x() < aabb.max.x()) &&
            (aabb.min.y() <= apoint.y() && apoint.y() < aabb.max.y()) &&
            (aabb.min.z() <= apoint.z() && apoint.z() < aabb.max.z()))
            return true;
        else
            return false;
    }

    /// Get the axis-aligned bounding box of this domain
    const ChAABB& GetAABB() const { return aabb; }

    /// Set the axis-aligned bounding box of this domain
    void SetAABB(const ChAABB& domain_aabb) { aabb = domain_aabb; }

    // TODO: optimize this, we need to keep tags for the BVH case, but not sure if this is absolutely necessary
    std::vector<int> tags;
    std::vector<int> excluded_body_tags;

  private:
    ChAABB aabb;
};

//  JZ - ADDED ============================================================

class ChApiMultiDomain ChDomainBuilderBVH : public ChDomainBuilder {
  public:
    ChDomainBuilderBVH(int num_domains, bool build_master = true) : ChDomainBuilder() {
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

    void SetDecompositionInterval(int step_interval);

    void RebuildDomainsAsync(ChSystem* system, int mpi_rank, int current_step);

    bool GetPartitionUpdateNeeded() { return partition_update_needed_; }

    void ResetPartitionUpdateNeeded() { partition_update_needed_ = false; }

    void ApplyNewDecomposition(int mpi_rank);

  private:
    std::vector<AABB> domain_aabbs_;           // aabb with tags
    std::vector<ChAABB> domain_aabbs_synced_;  // a aabb copy without any tags
    int num_domains_;
    bool build_master_;
    std::vector<std::shared_ptr<ChBody>> excluded_bodies_;
    std::shared_ptr<ChDomainBox> current_domain_box_;

    // domain tracker
    DomainTracker domain_tracker_;

    bool decomposition_in_progress_ = false;
    std::future<std::vector<AABB>> async_decomposition_future_;
    int decomposition_step_interval_ = 10;
    int last_decomposition_step_ = -1;

    bool partition_update_needed_ = false;
};

}  // end namespace multidomain
}  // end namespace chrono

#endif
