// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Wrapper for several constructs that are common to many flatbuffer messages
// (Vectors, Quaternions, frames)
// See also flatbuffer/fbs/Utils.fbs
//
// =============================================================================

#ifndef SYN_MESSAGE_UTILS_H
#define SYN_MESSAGE_UTILS_H

#include "chrono_synchrono/flatbuffer/message/SynFlatBuffers_generated.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_synchrono/SynApi.h"

namespace chrono {
namespace synchrono {

class SYN_API AgentKey {
  public:
    AgentKey() : AgentKey(-1, -1) {}
    AgentKey(int node_id, int agent_id);
    AgentKey(const SynFlatBuffers::AgentKey* agent_key) : AgentKey(agent_key->node_id(), agent_key->agent_id()){};

    std::string GetKeyString() const;

    std::size_t operator()(const AgentKey& k) const { return k.m_unique_id; }
    bool operator==(const AgentKey& other) const { return m_unique_id == other.GetUniqueID(); }
    bool operator<(const AgentKey& other) const { return m_unique_id < other.GetUniqueID(); }

    int GetNodeID() const { return m_node_id; }
    int GetAgentID() const { return m_agent_id; }
    int GetUniqueID() const { return m_unique_id; }
    const SynFlatBuffers::AgentKey* const GetFlatbuffersKey() const;

  private:
    int m_node_id;
    int m_agent_id;
    int m_unique_id;
};

/// @addtogroup synchrono_flatbuffer
/// @{

/// Wrapper for several constructs that are common to many flatbuffer messages (Vectors, Quaternions, frames)
class SYN_API SynPose {
  public:
    ///@brief Construct a new Pose object
    SynPose(const ChVector3d& mv = ChVector3d(0, 0, 0), const ChQuaternion<>& mq = ChQuaternion<>(1, 0, 0, 0));

    ///@brief Construct a new Pose object
    SynPose(const ChFrameMoving<>& frame);

    ///@brief Construct a new Pose object from a FlatBuffers pose object
    ///
    ///@param pose the FlatBuffers pose object
    SynPose(const SynFlatBuffers::Pose* pose);

    ///@brief Convert this pose object to a flatbuffers pose type
    ///
    ///@param builder the FlatBuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Pose> the flatbuffer pose
    flatbuffers::Offset<SynFlatBuffers::Pose> ToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const;

    ChFrameMoving<>& GetFrame() { return m_frame; }

  private:
    ChFrameMoving<> m_frame;
};

/// @addtogroup synchrono_flatbuffer
/// @{

/// Wrapper for several constructs that are common to many flatbuffer messages (Vectors, Quaternions, frames)
class SYN_API SynTransform {
  public:
    ///@brief Construct a new Transform object
    SynTransform(const ChVector3d& mv = ChVector3d(0, 0, 0), const ChQuaternion<>& mq = ChQuaternion<>(1, 0, 0, 0));

    ///@brief Construct a new Transform object
    SynTransform(const ChFrame<>& frame);

    ///@brief Construct a new Pose object from a FlatBuffers pose object
    ///
    ///@param pose the FlatBuffers pose object
    SynTransform(const SynFlatBuffers::Transform* pose);

    ///@brief Convert this pose object to a flatbuffers pose type
    ///
    ///@param builder the FlatBuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Transform> the flatbuffer transform
    flatbuffers::Offset<SynFlatBuffers::Transform> ToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const;

    ChFrame<>& GetFrame() { return m_frame; }

  private:
    ChFrame<> m_frame;
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif