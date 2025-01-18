#pragma once

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/collision/ChCollisionSystemSynchrono.h"
#include "chrono/core/ChVector3.h"

namespace chrono {
namespace synchrono {

struct RankContactData {
    unsigned int body_index;
    unsigned int rank;
    ChVector3d total_force;
    ChVector3d total_torque;
};

/// Contact message class
class SYN_API SynContactMessage : public SynMessage {
  public:
    SynContactMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey());

    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    void SetState(double time, const std::vector<RankContactData>& contacts) {
        this->time = time;
        this->vec_contacts = contacts;
        this->num_contacts = contacts.size();
    }

    double time;
    unsigned int num_contacts;
    std::vector<RankContactData> vec_contacts;
};

}  // namespace synchrono
}  // namespace chrono