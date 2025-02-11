#pragma once

#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include <unordered_map>

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_collision
/// @{

/// Custom collision system for SynChrono.
/// Inherits from ChCollisionSystemBullet but filters contacts based on agent ranks.
class SYN_API ChCollisionSystemSynchrono : public ChCollisionSystemBullet {
  public:
    // Structure to hold accumulated contact data between ranks
    struct RankContactData {
        ChBody* body;
        unsigned int body_index;  // the remote body index
        unsigned int rank;        // the rank of the body
        ChVector3d total_force = ChVector3d(0, 0, 0);
        ChVector3d total_torque = ChVector3d(0, 0, 0);
    };

    ChCollisionSystemSynchrono(std::shared_ptr<SynAgent> agent, int rank)
        : ChCollisionSystemBullet(), m_agent(agent), m_my_rank(rank) {
        auto callback = chrono_types::make_shared<BroadphaseSynchrono>(m_my_rank, m_body_ranks);
        this->RegisterBroadphaseCallback(callback);
    }

    void AddBodyRank(std::shared_ptr<ChBody> body, int rank) { m_body_ranks[body.get()] = rank; }

    void AddBodyIndex(std::shared_ptr<ChBody> body, unsigned int index) { m_body_indices[body.get()] = index; }

    std::unordered_map<ChBody*, RankContactData>& GetContactData() { return m_rank_contacts; }

    virtual ~ChCollisionSystemSynchrono() {}

    virtual void Run() override;

    virtual void ReportContacts(ChContactContainer* container) override {
        // Create contact collector
        class ContactCollector : public ChContactContainer::ReportContactCallback {
          public:
            ContactCollector(int my_rank,
                             std::unordered_map<ChBody*, int>& body_ranks,
                             std::unordered_map<ChBody*, RankContactData>& rank_contacts,
                             std::unordered_map<ChBody*, unsigned int>& body_indices,
                             std::vector<ChBody*>& active_bodies)
                : m_my_rank(my_rank),
                  m_body_ranks(body_ranks),
                  m_rank_contacts(rank_contacts),
                  m_body_indices(body_indices),
                  m_active_bodies(active_bodies) {}

            virtual bool OnReportContact(const ChVector3d& pA,
                                         const ChVector3d& pB,
                                         const ChMatrix33<>& plane_coord,
                                         const double& distance,
                                         const double& eff_radius,
                                         const ChVector3d& cforce,
                                         const ChVector3d& ctorque,
                                         ChContactable* modA,
                                         ChContactable* modB) override {
                // get the bodies from the collision models
                auto bodyA = dynamic_cast<ChBody*>(modA);
                auto bodyB = dynamic_cast<ChBody*>(modB);

                if (m_body_ranks.find(bodyA) != m_body_ranks.end() || m_body_ranks.find(bodyB) != m_body_ranks.end()) {
                    if (!(m_body_ranks.find(bodyA) != m_body_ranks.end() &&
                          m_body_ranks.find(bodyB) != m_body_ranks.end())) {
                        // collision between two local bodies on the different ranks
                        // record the contact data on the body located on the remote rank, using the ChBody* as the key
                        ChBody* remote_body = (m_body_ranks.find(bodyA) != m_body_ranks.end()) ? bodyA : bodyB;

                        // add to active_bodies
                        m_active_bodies.push_back(remote_body);
                    }
                }

                // continue to report contacts
                // this func must return true
                return true;
            }

          private:
            int m_my_rank;
            std::unordered_map<ChBody*, int>& m_body_ranks;
            std::vector<ChBody*>& m_active_bodies;
            std::unordered_map<ChBody*, unsigned int>& m_body_indices;
            std::unordered_map<ChBody*, RankContactData>& m_rank_contacts;
        };

        auto collector = chrono_types::make_shared<ContactCollector>(m_my_rank, m_body_ranks, m_rank_contacts,
                                                                     m_body_indices, m_active_bodies);
        container->ReportAllContacts(collector);

        // Let parent handle normal contact reporting
        ChCollisionSystemBullet::ReportContacts(container);
    }

  protected:
    std::shared_ptr<SynAgent> m_agent;  ///< Associated SynChrono agent
    int m_my_rank;                      ///< Rank of this collision system
    std::unordered_map<ChBody*, int> m_body_ranks;
    std::vector<ChBody*> m_active_bodies;
    std::unordered_map<ChBody*, RankContactData> m_rank_contacts;
    std::unordered_map<ChBody*, unsigned int> m_body_indices;

  private:
    /// Custom callback class for culling broadphase collisions.
    class BroadphaseSynchrono : public ChCollisionSystem::BroadphaseCallback {
      public:
        BroadphaseSynchrono(int rank, std::unordered_map<ChBody*, int>& body_ranks)
            : m_rank(rank), m_body_ranks(body_ranks) {}

        virtual bool OnBroadphase(ChCollisionModel* modelA, ChCollisionModel* modelB) override {
            auto bodyA = dynamic_cast<ChBody*>(modelA->GetPhysicsItem());
            auto bodyB = dynamic_cast<ChBody*>(modelB->GetPhysicsItem());

            if (!bodyA || !bodyB)
                return false;

            // Look up ranks for both bodies
            auto rankA = m_body_ranks.find(bodyA);
            auto rankB = m_body_ranks.find(bodyB);

            // If we can't find ranks, assume they're local bodies (m_rank)
            int bodyARank = (rankA != m_body_ranks.end()) ? rankA->second : m_rank;
            int bodyBRank = (rankB != m_body_ranks.end()) ? rankB->second : m_rank;

            // collision between two local bodies on the same rank
            if (bodyARank == m_rank && bodyBRank == m_rank) {
                return true;
            }

            // collision not involving this rank
            if (bodyARank != m_rank && bodyBRank != m_rank) {
                return false;
            }

            // collision involving this rank
            // local rank is the rank of the body that is colliding with the remote body
            // remote rank is the rank of the body that is colliding with the local body
            // allow collision if local rank < remote rank
            int& local_rank = (bodyARank == m_rank) ? bodyARank : bodyBRank;
            int& remote_rank = (bodyARank == m_rank) ? bodyBRank : bodyARank;
            return local_rank < remote_rank;

            return true;
        }

      private:
        int m_rank;
        std::unordered_map<ChBody*, int>& m_body_ranks;
    };
};

/// @}

}  // namespace synchrono
}  // namespace chrono