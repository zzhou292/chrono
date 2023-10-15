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

#include "chrono_ros/handlers/ChROSLidarTFHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "rclcpp/rclcpp.hpp"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSLidarTFHandler::ChROSLidarTFHandler(std::shared_ptr<ChLidarSensor> lidar, std::shared_ptr<ChBody> body)
    : m_lidar(lidar),  m_body(body) {}

bool ChROSLidarTFHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    // Initialize the handler, e.g., create publishers if needed
    // Return true if successful, false otherwise
    return true;
}

void ChROSLidarTFHandler::Tick(double time) {
    // Get the transform from the base frame to the sensor frame
    chrono::ChFrame<double> sensor_frame = m_lidar->GetOffsetPose();
    chrono::ChFrame<double> base_frame = m_lidar->GetParentBody()->GetFrame_REF_to_abs();

    // Calculate the transform
    chrono::ChFrame<double> transform = base_frame * sensor_frame;

    // Create a tf2 transform message
    geometry_msgs::msg::TransformStamped tf_msg;
    // tf_msg.header.stamp = rclcpp::Time(time);
    // tf_msg.header.frame_id = m_base_frame;
    // tf_msg.child_frame_id = m_sensor_frame;
    // tf_msg.transform.translation.x = transform.GetPos().x();
    // tf_msg.transform.translation.y = transform.GetPos().y();
    // tf_msg.transform.translation.z = transform.GetPos().z();
    // tf_msg.transform.rotation.x = transform.GetRot().Q().x();
    // tf_msg.transform.rotation.y = transform.GetRot().Q().y();
    // tf_msg.transform.rotation.z = transform.GetRot().Q().z();
    // tf_msg.transform.rotation.w = transform.GetRot().Q().w();

    // // Publish the transform
    // m_tf_broadcaster.sendTransform(tf_msg);
}

} // namespace ros
} // namespace chrono
