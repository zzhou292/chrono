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
// Handler for interfacing a integer signal to ROS
//
// =============================================================================

#ifndef CH_ROS_DRIVER_SIGNAL_INPUT_HANDLER
#define CH_ROS_DRIVER_SIGNAL_INPUT_HANDLER

#include "chrono_ros/ChROSHandler.h"
#include <std_msgs/msg/int32.hpp>
#include <mutex>

namespace chrono {
namespace ros {

/// @addtogroup ros_vehicle_handlers
/// @{

/// This handler is responsible for interfacing a ChDriver to ROS. Will instantiate a subscriber to
/// chrono_ros_interfaces::msg::DriverInputs
class ChROSDriverSignalInputHandler : public ChROSHandler {
  public:
    /// Convenience constructor. Will set the update rate to 0, which means the Tick()
    /// function will update on each update call.
    ChROSDriverSignalInputHandler(int* signal, const std::string& topic_name, double update_rate);

    /// Initializes the handler. Creates a subscriber of chrono_ros_interfaces::msg::DriverInputs on topic
    /// "~/input/driver_inputs".
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Updates the driver with stored inputs data from Callback
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const std_msgs::msg::Int32& msg);

  private:
    const std::string m_topic_name;  ///< name of the topic to publish to
    int* m_signal;                   ///< the signal to update
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
        m_subscription;  ///< subscriber to the chrono_ros_interfaces::msg::DriverInputs

    std::mutex m_mutex;  ///< used to control access to m_inputs
};

}  // namespace ros
}  // namespace chrono

#endif
