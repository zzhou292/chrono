# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Sriram Ashokkumar
# =============================================================================
#
# Demo to show Cobra Rover operated on Rigid Terrain
#
# =============================================================================

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot_chrono
import pychrono.ros as chros
import pychrono.sensor as sens
import pychrono.vehicle as veh


try:
   from pychrono import irrlicht as chronoirr
except:
   print('Could not import ChronoIrrlicht')

# Chreate Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

contact_method = chrono.ChContactMethod_NSC
ground_mat = chrono.ChMaterialSurfaceNSC()

# room_mmesh = chrono.ChTriangleMeshConnected()
# room_mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/textures/hallway.obj"), False, True)
# room_mmesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))

# room_trimesh_shape = chrono.ChTriangleMeshShape()
# room_trimesh_shape.SetMesh(room_mmesh)
# room_trimesh_shape.SetName("Hallway Mesh")
# room_trimesh_shape.SetMutable(False)

# room_mesh_body = chrono.ChBody()
# room_mesh_body.SetPos(chrono.ChVectorD(-2, -2, -1))
# room_mesh_body.AddVisualShape(room_trimesh_shape)
# room_mesh_body.SetBodyFixed(True)
# room_mesh_body.GetCollisionModel().ClearModel()

# room_mesh_body.GetCollisionModel().AddTriangleMesh(ground_mat, room_mmesh, False, False)
# room_mesh_body.GetCollisionModel().BuildModel()
# room_mesh_body.SetCollide(True)

# # vehicle.GetSystem().Add(room_mesh_body)
# system.Add(room_mesh_body)

# Create ground body
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(30, 30, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(2, 5, -1.415))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


# Create Cobra rover
driver = robot_chrono.CobraSpeedDriver(1.0, 3.0)
rover = robot_chrono.Cobra(system, robot_chrono.CobraWheelType_SimpleWheel)
rover.SetDriver(driver)
rover.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, -0.2, -0.3), chrono.ChQuaternionD(1, 0, 0, 0)))


driver2 = robot_chrono.CobraSpeedDriver(1.0, 3.0)
rover2 = robot_chrono.Cobra(system, robot_chrono.CobraWheelType_SimpleWheel)
rover2.SetDriver(driver2)
rover2.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, -1, -0.3), chrono.ChQuaternionD(1, 0, 0, 0)))

# Create the RCCar vehicle, set parameters, and initialize
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY
time_step = 1e-3
tire_step_size = time_step

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_PRIMITIVES
tire_vis_type = veh.VisualizationType_PRIMITIVES


RC_1 = veh.RCCar(system)
RC_1.SetContactMethod(contact_method)
RC_1.SetChassisCollisionType(chassis_collision_type)
RC_1.SetChassisFixed(False)
RC_1.SetTireType(tire_model)
RC_1.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, -1, -0.3), chrono.Q_from_AngZ(1.57)))
RC_1.SetTireStepSize(tire_step_size)
RC_1.Initialize()

RC_1.SetChassisVisualizationType(chassis_vis_type)
RC_1.SetSuspensionVisualizationType(suspension_vis_type)
RC_1.SetSteeringVisualizationType(steering_vis_type)
RC_1.SetWheelVisualizationType(wheel_vis_type)
RC_1.SetTireVisualizationType(tire_vis_type)

# Create the sensor 
sens_manager = sens.ChSensorManager(system)

offset_pose = chrono.ChFrameD(chrono.ChVectorD(-8, 0, 2),
                              chrono.Q_from_AngAxis(.2, chrono.ChVectorD(0, 1, 0)))

cam = sens.ChCameraSensor(rover.GetChassis().GetBody(), 30, offset_pose, 1280,  720, 1.408)
cam.PushFilter(sens.ChFilterVisualize(1280, 720))
cam.PushFilter(sens.ChFilterRGBA8Access())
sens_manager.AddSensor(cam)

lidar = sens.ChLidarSensor(rover.GetChassis().GetBody(), 5., offset_pose, 90, 300,
                            2*chrono.CH_C_PI, chrono.CH_C_PI / 12, -chrono.CH_C_PI / 6, 100., 0)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1))
sens_manager.AddSensor(lidar)


# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Cobra rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2.5, 1.5), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

# Create ROS manager
ros_manager = chros.ChROSManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSCameraHandler(
        cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(
    lidar, "~/output/lidar/data/pointcloud"))
ros_manager.RegisterHandler(chros.ChROSCobraSpeedDriverHandler(25, driver, "~/input/driver_inputs"))
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, rover.GetChassis().GetBody(), "~/output/cobra/state"))
ros_manager.Initialize()

####vis.EnableShadows()

time_step = 1e-3

driver.SetMotorSpeed(0)

# Simulation loop
time = 0

step = 0
while (vis.Run()) :
    time = time + time_step
    steering = 0
    # if time > 7:
    # 	if abs(rover.GetTurnAngle()) < 1e-8:
    # 		steering = 0
    # 	else:
    # 		steering = -0.4
    # elif time > 1:
    # 	steering = 0.4
    # driver.SetSteering(steering)
    
    rover.Update()
    RC_1.Advance(time_step)
    sens_manager.Update()
    if not ros_manager.Update(time, time_step):
        break
    if step%20==0 :
      vis.BeginScene()
      vis.Render()
      vis.EndScene()
    
    step =  step + 1

    system.DoStepDynamics(time_step)