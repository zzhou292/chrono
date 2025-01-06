#include "chrono_synchrono/agent/SynRoboEnvironmentAgent.h"

#include "chrono_synchrono/utils/SynLog.h"

namespace chrono {
namespace synchrono {

SynRoboEnvironmentAgent::SynRoboEnvironmentAgent(
    std::vector<std::pair<std::shared_ptr<ChBody>, std::pair<std::string, SynTransform>>> added_body_list,
    ChSystem* system)
    : SynAgent() {
    m_state = chrono_types::make_shared<SynRoboEnvironmentStateMessage>();
    m_description = chrono_types::make_shared<SynRoboEnvironmentDescriptionMessage>();

    m_system = system;

    if (added_body_list.size() > 0) {
        std::vector<std::string> visual_files;
        std::vector<std::string> collision_files;
        std::vector<SynTransform> mesh_transforms;
        for (auto& body : added_body_list) {
            m_master_bodies_list.push_back(body.first);
            collision_files.push_back(body.second.first);
            visual_files.push_back(body.second.first);
            mesh_transforms.push_back(body.second.second);
        }

        SetZombieVisualizationFiles(visual_files);
        SetZombieCollisionFiles(collision_files);
        SetZombieMeshTransforms(mesh_transforms);
    }
}

SynRoboEnvironmentAgent::~SynRoboEnvironmentAgent() {}

void SynRoboEnvironmentAgent::SetKey(AgentKey agent_key) {
    m_description->SetSourceKey(agent_key);
    m_state->SetSourceKey(agent_key);
    m_agent_key = agent_key;
}

void SynRoboEnvironmentAgent::InitializeZombie(ChSystem* system) {
    std::vector<std::string> visual_files = m_description->visual_files;
    std::vector<std::string> collision_files = m_description->collision_files;
    std::vector<SynTransform> mesh_transforms = m_description->mesh_transforms;
    for (size_t i = 0; i < visual_files.size(); i++) {
        auto zombie_body = chrono_types::make_shared<ChBodyAuxRef>();

        // create zombie bodies
        // visualization
        auto trimesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        if (!visual_files[i].empty()) {
            auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(visual_files[i], true, true);
            mesh->Transform(mesh_transforms[i].GetFrame().GetPos(), mesh_transforms[i].GetFrame().GetRot());
            trimesh->SetMesh(mesh);
            trimesh->SetMutable(false);
            zombie_body->AddVisualShape(trimesh);
        }

        // Create collision shape from mesh file if specified
        if (!collision_files[i].empty()) {
            auto collision_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            collision_mesh->LoadWavefrontMesh(collision_files[i], true, true);
            collision_mesh->Transform(mesh_transforms[i].GetFrame().GetPos(), mesh_transforms[i].GetFrame().GetRot());
            collision_mesh->RepairDuplicateVertexes(1e-9);
            auto collision_mat = ChContactMaterial::DefaultMaterial(system->GetContactMethod());
            auto collision_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
                collision_mat, collision_mesh, false, false, 0.001);
            zombie_body->AddCollisionShape(collision_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
            zombie_body->EnableCollision(true);
        } else {
            zombie_body->EnableCollision(false);
        }

        zombie_body->SetFixed(true);
        zombie_body->SetFrameCOMToRef(ChFrame<>({0, 0, -0.2}, {1, 0, 0, 0}));
        system->Add(zombie_body);

        m_zombie_bodies_list.push_back(zombie_body);
    }
}

void SynRoboEnvironmentAgent::SynchronizeZombie(std::shared_ptr<SynMessage> message) {
    if (auto state = std::dynamic_pointer_cast<SynRoboEnvironmentStateMessage>(message)) {
        for (size_t i = 0; i < m_zombie_bodies_list.size(); i++) {
            m_zombie_bodies_list[i]->SetFrameRefToAbs(state->GetItems()[i].GetFrame());
        }
    }
}

void SynRoboEnvironmentAgent::Update() {
    if (!m_system)
        return;

    items.resize(m_master_bodies_list.size());

    for (size_t i = 0; i < m_master_bodies_list.size(); i++) {
        auto item_abs = m_master_bodies_list[i]->GetFrameRefToAbs();

        SynPose item_pose(item_abs.GetPos(), item_abs.GetRot());
        item_pose.GetFrame().SetPosDt(item_abs.GetPosDt());
        item_pose.GetFrame().SetPosDt2(item_abs.GetPosDt2());
        item_pose.GetFrame().SetRotDt(item_abs.GetRotDt());
        item_pose.GetFrame().SetRotDt2(item_abs.GetRotDt2());

        items[i] = item_pose;
    }

    auto time = m_system->GetChTime();
    m_state->SetState(time, items);
}

}  // namespace synchrono
}  // namespace chrono