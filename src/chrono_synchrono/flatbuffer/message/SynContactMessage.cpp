#include "chrono_synchrono/flatbuffer/message/SynContactMessage.h"

namespace chrono {
namespace synchrono {

SynContactMessage::SynContactMessage(AgentKey source_key, AgentKey destination_key)
    : SynMessage(source_key, destination_key) {}

void SynContactMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    auto contact_state = message->message_as_Contact_State();

    time = contact_state->time();
    num_contacts = contact_state->num_contacts();

    vec_contacts.clear();
    for (const auto& contact : *contact_state->contacts()) {
        RankContactData data;
        data.body_index = contact->body_index();
        data.rank = contact->rank();
        data.total_force = ChVector3d(contact->total_force_x(), contact->total_force_y(), contact->total_force_z());
        data.total_torque = ChVector3d(contact->total_torque_x(), contact->total_torque_y(), contact->total_torque_z());
        vec_contacts.emplace_back(data);
    }

    // print out all received contact data
    // printf("Received contact data:\n");
    // for (const auto& contact : vec_contacts) {
    //     printf("Body index: %d, Rank: %d, Force: %7.3f, %7.3f, %7.3f, Torque: %7.3f, %7.3f, %7.3f\n",
    //            contact.body_index, contact.rank, contact.total_force.x(), contact.total_force.y(),
    //            contact.total_force.z(), contact.total_torque.x(), contact.total_torque.y(),
    //            contact.total_torque.z());
    // }
}

FlatBufferMessage SynContactMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    std::vector<flatbuffers::Offset<SynFlatBuffers::Contact::ContactData>> flatbuffer_contacts;
    flatbuffer_contacts.reserve(vec_contacts.size());

    for (const auto& contact : vec_contacts) {
        flatbuffer_contacts.emplace_back(SynFlatBuffers::Contact::CreateContactData(
            builder, contact.body_index, contact.rank, contact.total_force.x(), contact.total_force.y(),
            contact.total_force.z(), contact.total_torque.x(), contact.total_torque.y(), contact.total_torque.z()));
    }

    auto contact_state = SynFlatBuffers::Contact::CreateStateDirect(builder, time, num_contacts, &flatbuffer_contacts);

    return SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Contact_State, contact_state.Union(),
                                         m_source_key.GetFlatbuffersKey(), m_destination_key.GetFlatbuffersKey());
}

}  // namespace synchrono
}  // namespace chrono