#include "chrono_synchrono/agent/SynRobotAgent.h"
#include "chrono_synchrono/utils/SynLog.h"

namespace chrono {
namespace synchrono {

SynRobotAgent::SynRobotAgent(chrono::models::IRobotModel* robot) : SynAgent(), m_robot(robot) {
    m_state = chrono_types::make_shared<SynRobotStateMessage>();
    m_description = chrono_types::make_shared<SynRobotDescriptionMessage>();

    if (robot) {
        std::vector<std::string> visual_files;
        std::vector<std::string> collision_files;
        std::vector<SynTransform> mesh_transforms;
        for (auto& body : robot->GetCollidableBodiesWithPaths()) {
            collision_files.push_back(body.second);
        }
        for (auto& body : robot->GetVisualBodiesWithPaths()) {
            visual_files.push_back(body.second);
        }

        for (auto& transform : robot->GetMeshTransforms()) {
            mesh_transforms.push_back(transform.second);
        }

        SetZombieVisualizationFiles(visual_files);
        SetZombieCollisionFiles(collision_files);
        SetZombieMeshTransforms(mesh_transforms);
    }
}

SynRobotAgent::~SynRobotAgent() {}

void SynRobotAgent::SetKey(AgentKey agent_key) {
    m_description->SetSourceKey(agent_key);
    m_state->SetSourceKey(agent_key);
    m_agent_key = agent_key;
}

void SynRobotAgent::InitializeZombie(ChSystem* system) {
    // Get collision system
    auto collision_system = std::dynamic_pointer_cast<ChCollisionSystemSynchrono>(system->GetCollisionSystem());
    if (!collision_system) {
        throw std::runtime_error("SynRobotAgent requires ChCollisionSystemSynchrono");
    }

    std::vector<std::string> visual_files = m_description->visual_files;
    std::vector<std::string> collision_files = m_description->collision_files;
    std::vector<SynTransform> mesh_transforms = m_description->mesh_transforms;

    // Get the rank from the description's source key
    int source_rank = m_description->GetSourceKey().GetNodeID();

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

        // Register the body with its source rank
        collision_system->AddBodyRank(zombie_body, source_rank);

        m_zombie_bodies_list.push_back(zombie_body);
    }
    // test on force reporting
    collision_system->ReportContacts();
}

void SynRobotAgent::SynchronizeZombie(std::shared_ptr<SynMessage> message) {
    if (auto state = std::dynamic_pointer_cast<SynRobotStateMessage>(message)) {
        for (size_t i = 0; i < m_zombie_bodies_list.size(); i++) {
            m_zombie_bodies_list[i]->SetFrameRefToAbs(state->GetItems()[i].GetFrame());
        }
    }
}

void SynRobotAgent::Update() {
    if (!this->m_robot)
        return;

    items.resize(this->m_robot->GetCollidableBodiesWithPaths().size());

    for (size_t i = 0; i < this->m_robot->GetCollidableBodiesWithPaths().size(); i++) {
        auto item_abs = this->m_robot->GetCollidableBodiesWithPaths()[i].first->GetFrameRefToAbs();

        SynPose item_pose(item_abs.GetPos(), item_abs.GetRot());
        item_pose.GetFrame().SetPosDt(item_abs.GetPosDt());
        item_pose.GetFrame().SetPosDt2(item_abs.GetPosDt2());
        item_pose.GetFrame().SetRotDt(item_abs.GetRotDt());
        item_pose.GetFrame().SetRotDt2(item_abs.GetRotDt2());

        items[i] = item_pose;
    }

    auto time = this->m_robot->GetSystem()->GetChTime();
    this->m_state->SetState(time, items);
}

}  // namespace synchrono
}  // namespace chrono