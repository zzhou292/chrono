#include "SynRobotMessage.h"

namespace chrono {
namespace synchrono {

SynRobotStateMessage::SynRobotStateMessage(AgentKey source_key, AgentKey destination_key)
    : SynMessage(source_key, destination_key) {}

void SynRobotStateMessage::SetState(double time, std::vector<SynPose> items) {
    this->time = time;
    this->items = items;
}

void SynRobotStateMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    if (message->message_type() != SynFlatBuffers::Type_Agent_State)
        return;

    m_source_key = AgentKey(message->source_key());
    m_destination_key = message->destination_key();

    auto agent_state = message->message_as_Agent_State();
    auto state = agent_state->message_as_Robot_State();

    time = state->time();
    items.clear();
    for (auto item : *state->items())
        items.emplace_back(item);
}

FlatBufferMessage SynRobotStateMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    std::vector<flatbuffers::Offset<SynFlatBuffers::Pose>> flatbuffer_items;
    flatbuffer_items.reserve(this->items.size());
    for (const auto& item : this->items)
        flatbuffer_items.push_back(item.ToFlatBuffers(builder));

    auto robot_type = SynFlatBuffers::Agent::Type_Robot_State;
    auto robot_state = SynFlatBuffers::Agent::Robot::CreateStateDirect(builder, time, &flatbuffer_items).Union();
    auto flatbuffer_state = SynFlatBuffers::Agent::CreateState(builder, robot_type, robot_state);
    auto flatbuffer_message =
        SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Agent_State, flatbuffer_state.Union(),
                                      m_source_key.GetFlatbuffersKey(), m_destination_key.GetFlatbuffersKey());

    return flatbuffers::Offset<SynFlatBuffers::Message>(flatbuffer_message);
}

SynRobotDescriptionMessage::SynRobotDescriptionMessage(AgentKey source_key, AgentKey destination_key)
    : SynMessage(source_key, destination_key) {}

void SynRobotDescriptionMessage::SetDescription(const std::vector<std::string>& collidable_files,
                                                const std::vector<std::string>& visual_files,
                                                const std::vector<SynTransform>& mesh_transforms,
                                                const std::vector<unsigned int>& body_indices) {
    this->collision_files = collision_files;
    this->visual_files = visual_files;

    this->num_collidable_items = collision_files.size();
    this->num_visual_items = visual_files.size();

    this->mesh_transforms = mesh_transforms;

    this->body_indices = body_indices;
}

void SynRobotDescriptionMessage::ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) {
    if (message->message_type() != SynFlatBuffers::Type_Agent_Description)
        return;

    auto description = message->message_as_Agent_Description();
    m_source_key = AgentKey(message->source_key());
    m_destination_key = message->destination_key();

    auto robot_description = description->description_as_Robot_Description();

    num_collidable_items = robot_description->num_collidable_items();
    num_visual_items = robot_description->num_visual_items();

    collision_files.clear();
    for (auto file : *robot_description->collidable_item_vis_file())
        collision_files.emplace_back(file->str());

    visual_files.clear();
    for (auto file : *robot_description->visual_item_vis_file())
        visual_files.emplace_back(file->str());

    mesh_transforms.clear();
    for (auto transform : *robot_description->mesh_item_transform())
        mesh_transforms.emplace_back(transform);

    body_indices.clear();
    for (auto index : *robot_description->body_indices())
        body_indices.emplace_back(index);
}

FlatBufferMessage SynRobotDescriptionMessage::ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const {
    std::vector<flatbuffers::Offset<flatbuffers::String>> flatbuffer_visual_files;
    flatbuffer_visual_files.reserve(this->visual_files.size());

    for (const auto& item_visual_file : this->visual_files)
        flatbuffer_visual_files.push_back(builder.CreateString(item_visual_file));

    std::vector<flatbuffers::Offset<flatbuffers::String>> flatbuffer_collision_files;
    flatbuffer_collision_files.reserve(this->collision_files.size());

    for (const auto& item_collision_file : this->collision_files)
        flatbuffer_collision_files.push_back(builder.CreateString(item_collision_file));

    std::vector<flatbuffers::Offset<SynFlatBuffers::Transform>> flatbuffer_mesh_transforms;
    flatbuffer_mesh_transforms.reserve(this->mesh_transforms.size());
    for (const auto& item_transform : this->mesh_transforms)
        flatbuffer_mesh_transforms.push_back(item_transform.ToFlatBuffers(builder));

    std::vector<uint32_t> flatbuffer_body_indices;
    flatbuffer_body_indices.reserve(this->body_indices.size());
    for (const auto& item_body_index : this->body_indices)
        flatbuffer_body_indices.push_back(item_body_index);

    flatbuffers::Offset<SynFlatBuffers::Agent::Robot::Description> robot_description = 0;

    auto robot_type = SynFlatBuffers::Agent::Type_Robot_Description;
    robot_description = SynFlatBuffers::Agent::Robot::CreateDescriptionDirect(
        builder, num_collidable_items, num_visual_items, &flatbuffer_collision_files, &flatbuffer_visual_files,
        &flatbuffer_mesh_transforms, &flatbuffer_body_indices);

    auto flatbuffer_description =
        SynFlatBuffers::Agent::CreateDescription(builder, robot_type, robot_description.Union());
    return SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Agent_Description,
                                         flatbuffer_description.Union(), m_source_key.GetFlatbuffersKey(),
                                         m_destination_key.GetFlatbuffersKey());
}

}  // namespace synchrono
}  // namespace chrono