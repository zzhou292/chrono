// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Handler responsible for publishing information about a ChBody
//
// =============================================================================

#include "chrono_ros/handlers/ChROSTFHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


namespace chrono {
namespace ros {

ChROSTFHandler::ChROSTFHandler(double update_rate, std::shared_ptr<chrono::sensor::ChLidarSensor> lidar, std::shared_ptr<ChBody> body, const std::string& topic_name)
    : ChROSHandler(update_rate), m_lidar(lidar), m_body(body), m_topic_name(topic_name) {}

bool ChROSTFHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_publisher = node->create_publisher<geometry_msgs::msg::TransformStamped>(m_topic_name, 1);

    // m_msg.header.frame_id = ; TODO

    return true;
}

void ChROSTFHandler::Tick(double time) {
    auto cobra_pos = m_body->GetPos();
    auto lidar_pos = m_lidar->GetOffsetPose().GetPos();

    // auto rot = m_body->GetRot();
    // auto lin_vel = m_body->GetPos_dt();
    // auto ang_vel = m_body->GetWvel_loc();
    // auto lin_acc = m_body->GetPos_dtdt();
    // auto ang_acc = m_body->GetWacc_loc();

    m_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);

    // m_msg.transform.translation.x = cobra_pos[0];
    // m_msg.transform.translation.y = cobra_pos[1];
    // m_msg.transform.translation.z = cobra_pos[2];

    m_msg.transform.translation.x = cobra_pos[0] + lidar_pos[0];
    m_msg.transform.translation.y = cobra_pos[1] + lidar_pos[1];
    m_msg.transform.translation.z = cobra_pos[2] + lidar_pos[2];

    m_msg.transform.rotation.x = 0;
    m_msg.transform.rotation.y = 0;
    m_msg.transform.rotation.z = 0;
    m_msg.transform.rotation.w = 1;

    m_msg.child_frame_id = "base_link";


    m_publisher->publish(m_msg);
}

}  // namespace ros
}  // namespace chrono
