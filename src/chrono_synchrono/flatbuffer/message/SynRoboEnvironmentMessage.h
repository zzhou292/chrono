#pragma once

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include <vector>
#include <string>

namespace chrono {
namespace synchrono {

class SynRoboEnvironmentStateMessage : public SynMessage {
  public:
    SynRoboEnvironmentStateMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey());

    void SetState(double time, std::vector<SynPose> items);

    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    std::vector<SynPose>& GetItems() { return items; }

  private:
    double time;
    std::vector<SynPose> items;
};

class SynRoboEnvironmentDescriptionMessage : public SynMessage {
  public:
    SynRoboEnvironmentDescriptionMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey());

    void SetDescription(const std::vector<std::string>& collidable_files,
                        const std::vector<std::string>& visual_files,
                        const std::vector<SynTransform>& mesh_transforms,
                        const std::vector<unsigned int>& body_indices);

    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    std::vector<std::string> collision_files;
    std::vector<std::string> visual_files;
    std::vector<SynTransform> mesh_transforms;
    std::vector<unsigned int> body_indices;

    int num_collidable_items;
    int num_visual_items;
};

}  // namespace synchrono
}  // namespace chrono