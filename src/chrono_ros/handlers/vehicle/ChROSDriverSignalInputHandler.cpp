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
// Author: Json Zhou
// =============================================================================
//
// Handler for interfacing a ChDriver to ROS
//
// =============================================================================

#include "chrono_ros/handlers/vehicle/ChROSDriverSignalInputHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

using std::placeholders::_1;

namespace chrono {
namespace ros {

ChROSDriverSignalInputHandler::ChROSDriverSignalInputHandler(int* signal,
                                                             const std::string& topic_name,
                                                             double update_rate)
    : ChROSHandler(update_rate), m_topic_name(topic_name), m_signal(signal) {}

bool ChROSDriverSignalInputHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_subscription = node->create_subscription<std_msgs::msg::Int32>(
        m_topic_name, 1, std::bind(&ChROSDriverSignalInputHandler::Callback, this, _1));

    std::cout << "in initialize" << std::endl;
    return true;
}

void ChROSDriverSignalInputHandler::Callback(const std_msgs::msg::Int32& msg) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::cout << "Received signal: " << msg.data << std::endl;
    *(m_signal) = msg.data;
    std::cout << "Done Assigning" << std::endl;
}

// redundant function
void ChROSDriverSignalInputHandler::Tick(double time) {
    std::lock_guard<std::mutex> lock(m_mutex);
}

}  // namespace ros
}  // namespace chrono
