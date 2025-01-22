#include "chrono_synchrono/agent/SynRoboEnvironmentAgent.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono_synchrono/utils/SynLog.h"

namespace chrono {
namespace synchrono {

SynRoboEnvironmentAgent::SynRoboEnvironmentAgent(
    std::vector<std::pair<std::shared_ptr<ChBody>, std::pair<std::string, SynTransform>>> added_body_list,
    std::vector<unsigned int> body_indices,
    ChSystem* system)
    : SynAgent() {
    m_state = chrono_types::make_shared<SynRoboEnvironmentStateMessage>();
    m_description = chrono_types::make_shared<SynRoboEnvironmentDescriptionMessage>();
    m_contact_message = chrono_types::make_shared<SynContactMessage>();

    m_system = system;

    if (added_body_list.size() > 0) {
        std::vector<std::string> vec_visual_files;
        std::vector<std::string> vec_collision_files;
        std::vector<SynTransform> vec_mesh_transforms;
        std::vector<unsigned int> vec_body_indices;

        for (size_t i = 0; i < added_body_list.size(); i++) {
            m_master_bodies_list.push_back(added_body_list[i].first);
            vec_collision_files.push_back(added_body_list[i].second.first);
            vec_visual_files.push_back(added_body_list[i].second.first);
            vec_mesh_transforms.push_back(added_body_list[i].second.second);
            vec_body_indices.push_back(body_indices[i]);
        }

        SetZombieVisualizationFiles(vec_visual_files);
        SetZombieCollisionFiles(vec_collision_files);
        SetZombieMeshTransforms(vec_mesh_transforms);
        SetZombieBodyIndices(vec_body_indices);
    }

    SetProcessMessageCallback([this](std::shared_ptr<SynMessage> message) {
        // create an unordered map to store bodies which have been added force and torque
        std::unordered_map<ChBody*, std::pair<ChVector3d, ChVector3d>> force_torque_map;

        if (auto contact = std::dynamic_pointer_cast<SynContactMessage>(message)) {
            std::cout << "msg time: " << contact->time << "system time: " << m_system->GetChTime() << std::endl;
            for (const auto& contact_data : contact->vec_contacts) {
                auto body = m_system->GetBodies()[contact_data.body_index];
                auto new_force = contact_data.total_force;
                auto new_torque = contact_data.total_torque;

                std::cout << "body index: " << contact_data.body_index << "force: " << new_force.x() << " "
                          << new_force.y() << " " << new_force.z() << "torque: " << new_torque.x() << " "
                          << new_torque.y() << " " << new_torque.z() << std::endl;

                // if body has force list with size 0, add
                if (body->GetForces().size() == 0) {
                    auto force = chrono_types::make_shared<ChForce>();

                    body->AddForce(force);
                    force->SetMode(ChForce::FORCE);  // Force mode (not torque)
                    // force->SetFrame(ChForce::WORLD);
                    force->SetDir(new_force);              // Set direction and magnitude
                    force->SetMforce(new_force.Length());  // Scale factor of 1.0

                    auto torque = chrono_types::make_shared<ChForce>();
                    body->AddForce(torque);
                    torque->SetMode(ChForce::TORQUE);
                    // torque->SetFrame(ChForce::WORLD);
                    torque->SetDir(new_torque);
                    torque->SetMforce(new_torque.Length());

                } else {
                    // if body has force list with size 2, add
                    body->GetForces()[0]->SetDir(new_force);
                    body->GetForces()[0]->SetMforce(new_force.Length());
                    body->GetForces()[1]->SetDir(new_torque);
                    body->GetForces()[1]->SetMforce(new_torque.Length());
                }

                force_torque_map[body.get()] = std::make_pair(new_force, new_torque);

                // for other bodies with no force, set them to 0
                for (auto& body : m_system->GetBodies()) {
                    if (force_torque_map.find(body.get()) == force_torque_map.end()) {
                        if (body->GetForces().size() == 2) {
                            body->GetForces()[0]->SetDir(ChVector3d(0, 0, 0));
                            body->GetForces()[0]->SetMforce(0);

                            body->GetForces()[1]->SetDir(ChVector3d(0, 0, 0));
                            body->GetForces()[1]->SetMforce(0);
                        }
                    }
                }
            }
        }

        // check body list from system and apply 0 force and 0 torque if force_torque_map doesn't have the body element
        // for (auto& body : m_system->GetBodies()) {
        //     if (!body->GetForces().size()) {
        //         if (force_torque_map.find(body.get()) == force_torque_map.end()) {
        //             body->GetForces()[0]->SetDir(ChVector3d(0, 0, 0));
        //             body->GetForces()[0]->SetMforce(0);

        //             body->GetForces()[1]->SetDir(ChVector3d(0, 0, 0));
        //             body->GetForces()[1]->SetMforce(0);
        //         }
        //     }
        // }
    });
}

SynRoboEnvironmentAgent::~SynRoboEnvironmentAgent() {}

void SynRoboEnvironmentAgent::SetKey(AgentKey agent_key) {
    m_description->SetSourceKey(agent_key);
    m_state->SetSourceKey(agent_key);
    m_contact_message->SetSourceKey(agent_key);
    m_agent_key = agent_key;
}

void SynRoboEnvironmentAgent::InitializeZombie(ChSystem* system) {
    std::vector<std::string> visual_files = m_description->visual_files;
    std::vector<std::string> collision_files = m_description->collision_files;
    std::vector<SynTransform> mesh_transforms = m_description->mesh_transforms;
    std::vector<unsigned int> body_indices = m_description->body_indices;

    // Get collision system
    auto collision_system = std::dynamic_pointer_cast<ChCollisionSystemSynchrono>(system->GetCollisionSystem());
    if (!collision_system) {
        throw std::runtime_error("SynRobotAgent requires ChCollisionSystemSynchrono");
    }

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

            // Compute mass properties
            double mass;
            ChVector3d cog;
            ChMatrix33<> inertia;
            double density = 900;  // Example density
            collision_mesh->ComputeMassProperties(true, mass, cog, inertia);
            ChMatrix33<> principal_inertia_rot;
            ChVector3d principal_I;
            ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

            zombie_body->SetMass(mass * density);
            zombie_body->SetInertiaXX(density * principal_I);
            zombie_body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));

        } else {
            zombie_body->EnableCollision(false);
        }

        zombie_body->SetFixed(true);
        system->Add(zombie_body);

        // Register the body with its source rank
        collision_system->AddBodyRank(zombie_body, source_rank);
        collision_system->AddBodyIndex(zombie_body, body_indices[i]);

        m_zombie_bodies_list.push_back(zombie_body);
    }
}

void SynRoboEnvironmentAgent::SynchronizeZombie(std::shared_ptr<SynMessage> message) {
    if (auto state = std::dynamic_pointer_cast<SynRoboEnvironmentStateMessage>(message)) {
        for (size_t i = 0; i < m_zombie_bodies_list.size(); i++) {
            m_zombie_bodies_list[i]->SetFrameRefToAbs(state->GetItems()[i].GetFrame());
        }
    }

    // if (auto contact = std::dynamic_pointer_cast<SynContactMessage>(message)) {
    //     for (size_t i = 0; i < contact->vec_contacts.size(); i++) {
    //         auto current_force = m_system->GetBodies()[contact->vec_contacts[i].body_index]->GetContactForce();
    //         auto new_force = contact->vec_contacts[i].total_force;
    //         // print out the force
    //         std::cout << " ======== " << std::endl;
    //         printf("Current force: %7.3f, %7.3f, %7.3f\n", current_force.x(), current_force.y(), current_force.z());
    //         printf("New force: %7.3f, %7.3f, %7.3f\n", new_force.x(), new_force.y(), new_force.z());
    //     }
    // }
    // std::cout << " tp2 " << std::endl;
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

    // clear contact data
    m_contact_message->vec_contacts.clear();
    m_contact_message->num_contacts = 0;
}

void SynRoboEnvironmentAgent::GatherMessages(SynMessageList& messages) {
    messages.push_back(m_state);

    // Get collision system and gather contact data
    if (m_system) {
        if (auto collision_system =
                std::dynamic_pointer_cast<ChCollisionSystemSynchrono>(m_system->GetCollisionSystem())) {
            auto contact_map = collision_system->GetContactData();
            if (!contact_map.empty()) {
                std::vector<RankContactData> contacts;
                contacts.reserve(contact_map.size());
                for (const auto& pair : contact_map) {
                    std::cout << "body index: " << pair.second.body_index << "force: " << pair.second.total_force.x()
                              << " " << pair.second.total_force.y() << " " << pair.second.total_force.z()
                              << "torque: " << pair.second.total_torque.x() << " " << pair.second.total_torque.y()
                              << " " << pair.second.total_torque.z() << std::endl;
                    contacts.push_back(
                        {pair.second.body_index, pair.second.rank, pair.second.total_force, pair.second.total_torque});
                }
                std::cout << "contacts size: " << contacts.size() << std::endl;
                m_contact_message->SetState(m_system->GetChTime(), contacts);
                messages.push_back(m_contact_message);
            }
        }
    }
}

}  // namespace synchrono
}  // namespace chrono