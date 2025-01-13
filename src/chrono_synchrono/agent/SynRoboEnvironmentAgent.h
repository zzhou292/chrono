#ifndef SYN_ROBOENVIRONMENT_AGENT_H
#define SYN_ROBOENVIRONMENT_AGENT_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/flatbuffer/message/SynRoboEnvironmentMessage.h"
#include "chrono_synchrono/collision/ChCollisionSystemSynchrono.h"

#include "chrono/physics/ChBodyAuxRef.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

/// Agent wrapper of a robot, in particular holds a pointer to an IRobotModel and sends out
/// SynRobotMessage-s to synchronize its state
class SYN_API SynRoboEnvironmentAgent : public SynAgent {
  public:
    ///@brief Construct a robot agent with optionally a robot model
    ///
    ///@param bodies_files the files of the bodies to be added to the environment
    SynRoboEnvironmentAgent(
        std::vector<std::pair<std::shared_ptr<ChBody>, std::pair<std::string, SynTransform>>> added_body_list = {},
        ChSystem* system = nullptr);

    ///@brief Destructor.
    virtual ~SynRoboEnvironmentAgent();

    ///@brief Initialize this agent's zombie representation
    /// Bodies are added and represented in the lead agent's world.
    ///
    ///@param system the ChSystem used to initialize the zombie
    virtual void InitializeZombie(ChSystem* system) override;

    ///@brief Synchronize this agent's zombie with the rest of the simulation.
    /// Updates agent based on the passed message.
    /// Any message can be passed, so a check should be done to ensure this message was intended for this agent.
    ///
    ///@param message the message to process and is used to update the position of the zombie
    virtual void SynchronizeZombie(std::shared_ptr<SynMessage> message) override;

    ///@brief Update this agent
    /// Typically used to update the state representation of the agent to be distributed to other agents
    ///
    virtual void Update() override;

    ///@brief Generates messages to be sent to other nodes
    /// Will create or get messages and pass them into the referenced message vector
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GatherMessages(SynMessageList& messages) override { messages.push_back(m_state); }

    ///@brief Get the description messages for this agent
    /// A single agent may have multiple description messages
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GatherDescriptionMessages(SynMessageList& messages) override { messages.push_back(m_description); }

    void SetZombieVisualizationFiles(std::vector<std::string> visual_files) {
        m_description->visual_files = visual_files;
    }

    void SetZombieCollisionFiles(std::vector<std::string> collision_files) {
        m_description->collision_files = collision_files;
    }

    void SetZombieMeshTransforms(std::vector<SynTransform> mesh_transforms) {
        m_description->mesh_transforms = mesh_transforms;
    }

    virtual void SetKey(AgentKey agent_key) override;

  protected:
    std::vector<SynPose> items;
    std::vector<std::shared_ptr<ChBody>> m_master_bodies_list;
    std::vector<std::shared_ptr<ChBodyAuxRef>> m_zombie_bodies_list;

    std::shared_ptr<SynRoboEnvironmentStateMessage> m_state;  ///< State of the robot
    std::shared_ptr<SynRoboEnvironmentDescriptionMessage>
        m_description;  ///< Description for zombie creation on discovery

    ChSystem* m_system;
};

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif