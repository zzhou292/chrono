#pragma once

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include <vector>
#include <string>

namespace chrono {
namespace synchrono {

class SynRobotStateMessage : public SynMessage {
  public:
    SynRobotStateMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey());

    void SetState(double time, std::vector<SynPose> items);

    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    std::vector<SynPose>& GetItems() { return items; }

  private:
    double time;
    std::vector<SynPose> items;
};

class SynRobotDescriptionMessage : public SynMessage {
  public:
    SynRobotDescriptionMessage(AgentKey source_key = AgentKey(), AgentKey destination_key = AgentKey());

    void SetDescription(const std::vector<std::string>& collidable_files,
                        const std::vector<std::string>& visual_files,
                        const std::vector<SynTransform>& mesh_transforms);

    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    std::vector<std::string> collision_files;
    std::vector<std::string> visual_files;
    std::vector<SynTransform> mesh_transforms;

    int num_collidable_items;
    int num_visual_items;
};

}  // namespace synchrono
}  // namespace chrono