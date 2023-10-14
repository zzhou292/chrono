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
// Authors: Sriram Ashokkumar
// =============================================================================
//
// Handler responsible for publishing information about a ChBody
//
// =============================================================================

#ifndef CH_ROS_LIDAR_TF_HANDLER_H
#define CH_ROS_LIDAR_TF_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"

#include "chrono_ros_interfaces/msg/body.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "chrono_sensor/sensors/ChLidarSensor.h"



#include "chrono/physics/ChBody.h"

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

class ChROSLidarTFHandler : public ChROSHandler {
public:

    ChROSLidarTFHandler(std::shared_ptr<chrono::sensor::ChLidarSensor> lidar, std::shared_ptr<ChBody> body);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

protected:
    virtual void Tick(double time) override;

private:
    std::shared_ptr<chrono::sensor::ChLidarSensor> m_lidar; ///< The Lidar sensor to get transform for
    std::shared_ptr<ChBody> m_body; ///< The body to get transform for
    // const std::string m_base_frame; ///< The name of the base frame
    // const std::string m_sensor_frame; ///< The name of the sensor frame
    // tf2_ros::TransformBroadcaster m_tf_broadcaster; ///< TF broadcaster
};


/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
