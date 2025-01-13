#pragma once

#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_collision
/// @{

/// Custom collision system for SynChrono.
/// Inherits from ChCollisionSystemBullet but filters contacts based on agent ranks.
class SYN_API ChCollisionSystemSynchrono : public ChCollisionSystemBullet {
  public:
    ChCollisionSystemSynchrono(std::shared_ptr<SynAgent> agent, int rank)
        : ChCollisionSystemBullet(), m_agent(agent), m_my_rank(rank) {
        auto callback = chrono_types::make_shared<BroadphaseSynchrono>(m_my_rank, m_body_ranks);
        this->RegisterBroadphaseCallback(callback);
    }

    void AddBodyRank(std::shared_ptr<ChBody> body, int rank) { m_body_ranks[body.get()] = rank; }

    virtual ~ChCollisionSystemSynchrono() {}

    virtual void Run() override;

  protected:
    std::shared_ptr<SynAgent> m_agent;  ///< Associated SynChrono agent
    int m_my_rank;                      ///< Rank of this collision system
    std::unordered_map<ChBody*, int> m_body_ranks;

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
        }

      private:
        int m_rank;
        std::unordered_map<ChBody*, int>& m_body_ranks;
    };
};

/// @}

}  // namespace synchrono
}  // namespace chrono