#include "chrono_synchrono/collision/ChCollisionSystemSynchrono.h"

namespace chrono {
namespace synchrono {

void ChCollisionSystemSynchrono::Run() {
    ChCollisionSystemBullet::Run();

    m_rank_contacts.clear();

    for (auto& body : m_active_bodies) {
        m_rank_contacts[body].body = body;
        m_rank_contacts[body].body_index = m_body_indices[body];
        m_rank_contacts[body].rank = m_body_ranks[body];

        m_rank_contacts[body].total_force = body->GetContactForce();
        m_rank_contacts[body].total_torque = body->GetContactTorque();
    }
}

}  // namespace synchrono
}  // namespace chrono