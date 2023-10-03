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
// Authors: Sriram Ashokkumar, Aaron Young
// =============================================================================
//
// Handler responsible for receiving and updating the DC motor control commands
// for the Cobra rover
//
// =============================================================================

#ifndef CH_COBRA_DC_MOTOR_CONTROL_HANDLER_H
#define CH_COBRA_DC_MOTOR_CONTROL_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono_models/robot/cobra/Cobra.h"

#include "rclcpp/subscription.hpp"
#include "chrono_ros_interfaces/msg/cobra_speed_driver.hpp"

#include <mutex>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// This handler is responsible for interfacing a CobraSpeedDriver driver to ROS. Will instantiate a subscriber to
/// chrono_ros_interfaces::msg::CobraSpeedDriver.
class ChROSCobraSpeedDriverHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a CobraSpeedDriver driver
    ChROSCobraSpeedDriverHandler(double update_rate,
                                    std::shared_ptr<chrono::cobra::CobraSpeedDriver> driver,
                                    const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Updates the driver with stored inputs data from Callback
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const chrono_ros_interfaces::msg::CobraSpeedDriver& msg);

  private:
    std::shared_ptr<chrono::cobra::CobraSpeedDriver> m_driver;  ///< handle to the driver

    const std::string m_topic_name;                         ///< name of the topic to publish to
    chrono_ros_interfaces::msg::CobraSpeedDriver m_msg;  ///< message to publish
    rclcpp::Subscription<chrono_ros_interfaces::msg::CobraSpeedDriver>::SharedPtr
        m_subscription;  ///< the publisher for the imu message

    std::mutex m_mutex;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
